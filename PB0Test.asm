;
; Test_1.asm
; LCD starts with "123", PB0 press cycles to "345" then "456"
;

.include "m2560def.inc"

;---------------------------------
; Register definitions
;---------------------------------
.def temp7             = r16
.def temp8             = r17
.def delay_reg         = r18
.def debounce_temp     = r22
.def button_press_cnt  = r20
.def lcd_state         = r21
.def data              = r23
.def countL            = r12
.def countH            = r13
.def iL                = r24
.def iH                = r25

;---------------------------------
; LCD Related constants / debounce
;---------------------------------
.equ PORTFDIR       = 0xFF
.equ LCD_RS         = 7
.equ LCD_E          = 6
.equ LCD_RW         = 5
.equ LCD_N          = 3
.equ LCD_ID         = 1
.equ LCD_D          = 2
.equ LCD_BF         = 7
.equ LCD_FUNC_SET   = 0x30
.equ LCD_ENTRY_SET  = 0x04
.equ LCD_SET_ADD    = 0x80
.equ LCD_DISPLAY    = 0x08
.equ LCD_CLEAR      = 0x01
.equ bounce_loop    = 0xFFFF

;---------------------------------
; Short delay used by LCD routines
;---------------------------------
.macro delay
loop1:
	ldi delay_reg, 0x03
loop2:
	dec delay_reg
	nop
	brne loop2

	subi temp7, 1
	sbci temp8, 0
	brne loop1
.endmacro

;---------------------------------
; LCD helper macros
;---------------------------------
.macro lcd_write_com
	out PORTF, data
	ldi temp7, (0 << LCD_RS)|(0 << LCD_RW)
	out PORTA, temp7

	ldi temp7, low(100)
	ldi temp8, high(100)
	delay

	sbi PORTA, LCD_E

	ldi temp7, low(300)
	ldi temp8, high(300)
	delay

	cbi PORTA, LCD_E

	ldi temp7, low(300)
	ldi temp8, high(300)
	delay
.endmacro

.macro lcd_write_data
	out PORTF, data
	ldi temp7, (1 << LCD_RS)|(0 << LCD_RW)
	out PORTA, temp7

	ldi temp7, low(100)
	ldi temp8, high(100)
	delay

	sbi PORTA, LCD_E

	ldi temp7, low(300)
	ldi temp8, high(300)
	delay

	cbi PORTA, LCD_E

	ldi temp7, low(300)
	ldi temp8, high(300)
	delay
.endmacro

.macro lcd_wait_busy
	clr temp7
	out DDRF, temp7
	ldi temp7, (0 << LCD_RS)|(1 << LCD_RW)
	out PORTA, temp7
busy_loop:
	ldi temp7, low(100)
	ldi temp8, high(100)
	delay

	sbi PORTA, LCD_E

	ldi temp7, low(300)
	ldi temp8, high(300)
	delay

	in temp7, PINF
	cbi PORTA, LCD_E
	sbrc temp7, LCD_BF
	rjmp busy_loop
	clr temp7
	out PORTA, temp7
	ser temp7
	out DDRF, temp7
.endmacro

;---------------------------------
; Interrupt vectors
;---------------------------------
.cseg
.org 0x0000
	jmp RESET

.org INT0addr
	jmp EXT_INT0

;---------------------------------
; RESET: init MCU & LCD
;---------------------------------
RESET:
	; init stack pointer
	ldi temp7, high(RAMEND)
	out SPH, temp7
	ldi temp7, low(RAMEND)
	out SPL, temp7

	; LCD ports as outputs
	ldi temp7, PORTFDIR
	out DDRF, temp7
	out DDRA, temp7
	clr temp7
	out PORTA, temp7

	; INT0 pin input, enable pull-up
	clr temp7
	out DDRE, temp7
	ldi temp7, (1 << PE0)
	out PORTE, temp7

	; Set INT0 for falling edge and enable
	ldi temp7, (1 << ISC01)
	sts EICRA, temp7
	ldi temp7, (1 << INT0)
	out EIMSK, temp7

	clr button_press_cnt
	clr lcd_state

	rcall lcd_init_sequence
	rcall display_123

	sei

;---------------------------------
; main loop
;---------------------------------
main_loop:
pause:
    ; 等待 button_press_cnt 的 bit0 变成 1
    sbrs button_press_cnt, 0     ; 如果 bit0 ==1，跳过下一条
    rjmp pause                   ; 否则一直停在这里

    ; 用掉这一“次按键”：清 bit0
    cbr  button_press_cnt, 0x01  ; clear bit0

show_456:
	rcall display_456
	ldi lcd_state, 2
	rjmp main_loop
	delay

show_123:
	rcall display_123
	clr lcd_state
	rjmp main_loop

;---------------------------------
; INT0 ISR
;---------------------------------
EXT_INT0:
	push temp7
	in   temp7, SREG
	push temp7

	inc button_press_cnt

	pop temp7
	out SREG, temp7
	pop temp7
	reti

;---------------------------------
; LCD helper routines
;---------------------------------
lcd_init_sequence:
	ldi temp7, low(15000)
	ldi temp8, high(15000)
	delay

	ldi data, LCD_FUNC_SET | (1 << LCD_N)
	lcd_write_com

	ldi temp7, low(4100)
	ldi temp8, high(4100)
	delay

	lcd_write_com

	ldi temp7, low(100)
	ldi temp8, high(100)
	delay

	lcd_write_com
	lcd_write_com

	lcd_wait_busy
	ldi data, LCD_DISPLAY | (0 << LCD_D)
	lcd_write_com

	lcd_wait_busy
	ldi data, LCD_CLEAR
	lcd_write_com

	lcd_wait_busy
	ldi data, LCD_ENTRY_SET | (1 << LCD_ID)
	lcd_write_com

	lcd_wait_busy
	ldi data, LCD_DISPLAY | (1 << LCD_D)
	lcd_write_com
	ret

lcd_set_line0:
	ldi data, LCD_SET_ADD | 0x00
	lcd_wait_busy
	lcd_write_com
	ret  

display_123:
	rcall lcd_set_line0
	ldi data, '1'
	lcd_wait_busy
	lcd_write_data
	ldi data, '2'
	lcd_wait_busy
	lcd_write_data
	ldi data, '3'
	lcd_wait_busy
	lcd_write_data
	ret


display_456:
	rcall lcd_set_line0
	ldi data, '4'
	lcd_wait_busy
	lcd_write_data
	ldi data, '5'
	lcd_wait_busy
	lcd_write_data
	ldi data, '6'
	lcd_wait_busy
	lcd_write_data
	ret
