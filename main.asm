; ==============================================================================
; Drone Search Simulation Project Stub (Search-Path Focus)
; Target: ATmega2560 @ lab board, Microchip Studio build
; Scope limited to keypad input, search-path generation, and LCD playback.
; Altitude/speed/crash behaviour is hard-coded per Figure 4 requirements.
; ==============================================================================

.include "m2560def.inc"

; ------------------------------------------------------------------------------
; Register aliases (caller-saved temps unless stated otherwise)
; ------------------------------------------------------------------------------
.def temp0      = r2
.def temp1      = r3
.def temp2      = r4
.def temp3      = r5
.def temp4      = r6
.def temp5      = r7
.def temp6      = r8
.def temp7      = r26        ; use X low to allow SUBI/SBCI in delay
.def temp8      = r27        ; use X high to allow SUBI/SBCI in delay
.def temp9      = r11
.def temp10     = r12
.def temp11     = r13
.def temp12     = r14
.def temp13     = r15
.def lcd_data   = r16
.def data       = r30
.def workA      = r17
.def workB      = r18
.def workC      = r19
.def workD      = r20
.def workE      = r21
.def workF      = r22
.def workG      = r23
.def stateReg   = r24
.def modeReg    = r25

; ------------------------------------------------------------------------------
; Constants (update once assumptions are refined)
; ------------------------------------------------------------------------------
.equ MAP_SIZE           = 15							; maximum grid 15x15
.equ MAP_CELLS          = (MAP_SIZE*MAP_SIZE)
.equ MAX_OBS_POINTS     = (MAP_SIZE*MAP_SIZE)           ; supports multiple coverage groups
.equ OBS_POINT_STRIDE   = 3								; x, y, z (height)
.equ PATH_BUF_BYTES     = MAX_OBS_POINTS * OBS_POINT_STRIDE
.equ LCD_COLS           = 16
.equ LCD_ROWS           = 2
.equ VISIBILITY_MAX     = 15           ; displayed in two chars
.equ DEFAULT_ALT_DM     = 61           ; Figure 4(c): 61 dm
.equ DEFAULT_SPEED_DMPS = 2            ; Figure 4(c): 2 dm/s

; LCD command constants (reuse Lab 3/4 sequences)
.equ LCD_FUNC_SET       = 0x30
.equ LCD_ENTRY_SET      = 0x04
.equ LCD_SET_ADD        = 0x80
.equ LCD_DISPLAY        = 0x08
.equ LCD_CLEAR          = 0x01
.equ LCD_RETURN         = 0x02
.equ SEC_DISPLAY_ADD    = 0x40

; Port direction masks
.equ PORTFDIR           = 0xFF         ; PF7-0 outputs for LCD data
.equ PORTLDIR           = 0xF0         ; PL7-4 outputs (columns), PL3-0 inputs
.equ LED_DDR_MASK       = 0xFF

; Keypad timing
.equ KEYPAD_SETTLE_LO   = low(35000)
.equ KEYPAD_SETTLE_HI   = high(35000)

; System states
.equ STATE_IDLE         = 0
.equ STATE_CONFIG       = 1
.equ STATE_PATH_GEN     = 2
.equ STATE_SCROLL_PATH  = 3
.equ STATE_PLAYBACK     = 4
.equ STATE_DONE         = 5

; Push button bits on PINB
.equ PB0_MASK           = 0b00000001
.equ PB1_MASK           = 0b00000010

; Keypad wiring constants (reuse from lab references once finalised)
.equ ROWMASK            = 0x0F
.equ INIT_COL_MASK      = 0xEF

; LCD control bits on PORTA
.equ LCD_RS             = 7
.equ LCD_E              = 6
.equ LCD_RW             = 5
.equ LCD_BUSY_BIT       = 7
.equ LCD_N              = 3
.equ LCD_ID             = 1
.equ LCD_S              = 0
.equ LCD_D              = 2
.equ LCD_C              = 1
.equ LCD_B              = 0

; Timing constants
.equ TICK_10MS          = 100          ; placeholder for software timer loops
.equ SCROLL_PERIOD      = 5            ; number of ticks between LCD scrolls

;-------------------------------------------------------------------------------
; Macro Definition
;-------------------------------------------------------------------------------
.macro load_array_from_program
	push zh
	push zl
	push xh
	push xl
	push r16
	in r16, SREG
	push r16
	push r17
	ldi zh, high(@0 << 1)
	ldi zl, low(@0 << 1)
	ldi xh,	high(@1)
	ldi xl, low(@1)
	clr r17
load_byte_from_program:
	cpi r17, @2 
	brsh load_byte_from_program_end
	lpm r16, z+
	st x+, r16
	inc r17
	rjmp load_byte_from_program
load_byte_from_program_end:
	pop r17
	pop r16
	out SREG, r16
	pop r16
	pop xl
	pop xh
	pop zl
	pop zh
.endmacro

;void load_array_from_data(data addr1, data addr2, int length) 
; Description: data addr1 <- data addr2
;r16: temp, r17: counter
.macro load_array_from_data
	push yh
	push yl
	push xh
	push xl
	push r16
	in r16, SREG
	push r16
	push r17
	ldi yh, high(@0)
	ldi yl, low(@0)
	ldi xh,	high(@1)
	ldi xl, low(@1)
	clr r17
load_byte_from_data:
	cpi r17, @2 
	brsh load_byte_from_data_end
	ld r16, x+
	st y+, r16
	inc r17
	rjmp load_byte_from_data
load_byte_from_data_end:
	pop r17
	pop r16
	out SREG, r16
	pop r16
	pop xl
	pop xh
	pop yl
	pop yh
.endmacro

;int* array_i_j(array, int size, Rd i, Rd j)
; Description: array[j][i]
;Rd != r16/r17, r16: temp, r17: temp
.macro array_i_j
	push yh
	push yl
	push r16
	in r16, SREG
	push r16
	push r17

	ldi yh, high(@0)
	ldi yl, low(@0)
	mov r16, @3
	ldi r17, @1
	mul r16, r17
	mov r16, r0
	mov r17, @2
	neg r17
	sub r16, r17
	;dec r16
	add yl, r16
	ldi r16, 0
	adc yh, r16
	mov xl, yl
	mov xh, yh

	pop r17
	pop r16
	out SREG, r16
	pop r16
	pop yl
	pop yh
.endmacro


;int* array_i(array, Rd i)
; Description: array[j][i]
;Rd != r16/r17, r16: temp, r17: temp
.macro array_i
	push yh
	push yl
	push r16
	in r16, SREG
	push r16
	push r17

	ldi yh, high(@0)
	ldi yl, low(@0)
	add yl, @1
	ldi r16, 0
	adc yh, r16
	mov xl, yl
	mov xh, yh

	pop r17
	pop r16
	out SREG, r16
	pop r16
	pop yl
	pop yh
.endmacro

;int count_diff(c_vis, p_vis, int length, Rd)
;Description: Count the difference between two vis arrays
;r16: c_vis[j][i] r17: p_vis[j][i], r18: counter, Rd != r16/r17
.macro count_diff
	push yh
	push yl
	push xh
	push xl
	push r16
	in r16, SREG
	push r16
	push r17
	push r18

	ldi yh, high(@0)
	ldi yl, low(@0)
	ldi xh, high(@1)
	ldi xl, low(@1)
	clr r18
	clr @3
count_diff_cond:
	cpi r18, @2
	brsh count_diff_end
	ld r16, y+
	ld r17, x+
	cp r16, r17
	breq count_diff_same
	inc @3
count_diff_same:
	inc r18
	rjmp count_diff_cond
count_diff_end:
	pop r18
	pop r17
	pop r16
	out SREG, r16
	pop	r16
	pop xl
	pop xh
	pop yl
	pop yh
.endmacro

; void greedy_search()
; Description: Greedy search
; r18 = org_x, r19 = org_y, r20 = cur_x, r21 = cur_y, r22 = visability, r2 = diff, r3 = max_diff, r4 = max_x, r5 = max_y, r6 = counter, r23 = y, r24 = x, r25 = temp
.macro greedy_search
	push yh
	push yl
	push xh
	push xl

	push r2
	push r3
	push r4
	push r5
	push r6
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	push r24
	push r25
	in r25, SREG
	push r25

	; counter = 1
	clr r6
	inc r6
greedy_search_start:
	; max_diff = 0
	clr r2
	clr r3
	clr r4
	clr r5
	clr r23
	clr r24
y_for_loop:
	; for y in range(len(mountain))
	clr r24
	cpi r23, size3
	in r25, SREG
	sbrc r25, 1
	jmp y_for_loop_end
	x_for_loop:
		; for x in range(len(mountain))
		cpi r24, size3
		in r25, SREG
		sbrc r25, 1
		jmp x_for_loop_end
		if_pre_v_y_x_eq_0:
			; if pre_v[y][x] == 0
			array_i_j p_vis, size3, r24, r23
			ld r25, x
			cpi r25, 0
			in r25, SREG
			sbrs r25, 1
			jmp if_pre_v_y_x_eq_0_end
		
			; cur_v = [[pre_v[j][i] for i in range(len(mountain))] for j in range(len(mountain))]
			load_array_from_data c_vis, p_vis, l3
			; check = [[0 for _ in range(len(mountain))] for _ in range(len(mountain))]
			load_array_from_program zeros, check, l3
			; dfs(x, y, x, y, visability, p_index)
			mov r18, r24
			mov r19, r23
			mov r20, r24
			mov r21, r23
			ldi r22, 3
			call dfs
			; diff = count_diff(cur_v, pre_v)
			count_diff c_vis, p_vis, l3, r2
			if_diff_g_max_diff:
				; if diff > max_diff
				cp r3, r2
				in r25, SREG
				sbrs r25, 0
				jmp if_diff_g_max_diff_end
				; max_diff = int(diff)
				mov r3, r2
				; max_point = (x, y)
				mov r4, r24
				mov r5, r23
				; max_map = [[cur_v[j][i] for i in range(len(mountain))] for j in range(len(mountain))]
				load_array_from_data max_map, c_vis, l3
			if_diff_g_max_diff_end:
		if_pre_v_y_x_eq_0_end:
		; x++
		inc r24
		rjmp x_for_loop
	x_for_loop_end:
	; y++
	inc r23
	rjmp y_for_loop
y_for_loop_end:
	; if max_diff == 0: break
	ldi r25, 0
	cp r3, r25
	breq greedy_search_end
	; point.append(max_point)
	ldi xh, high(obs_x)
	ldi xl, low(obs_x)
	add xl, r6
	ldi r25, 0
	adc xh, r25
	st x, r4

	ldi xh, high(obs_y)
	ldi xl, low(obs_y)
	add xl, r6
	ldi r25, 0
	adc xh, r25
	st x, r5
	; pre_v = [[max_map[j][i] for i in range(len(mountain))] for j in range(len(mountain))]
	load_array_from_data p_vis, max_map, l3
	;counter++
	inc r6
	jmp greedy_search_start
greedy_search_end:
	inc r6
	ldi xh, high(obs_len)
	ldi xl, low(obs_len)
	st x, r6

	pop r25
	out SREG, r25
	pop r25
	pop r24
	pop r23
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r6
	pop r5
	pop r4
	pop r3
	pop r2
	pop xl
	pop xh
	pop yl
	pop yh
.endmacro

; ------------------------------------------------------------------------------
; Utility macros reused from Lab 3/4
; ------------------------------------------------------------------------------
.macro delay
loop1:
	; ldi requires r16..r31, so load into workA then move
	ldi workA, 0x03
	mov temp10, workA
loop2:
	dec temp10
	nop
	brne loop2
	subi temp7, 1
	sbci temp8, 0
	brne loop1
.endmacro

; Deprecated: old LCD_WRITE_CMD/LCD_WRITE_DATA macros removed.
; Use lab-compatible macros: lcd_write_com, lcd_write_data, lcd_wait_busy.

; Lab-style LCD macros (exact timing/flow)
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
	sbrc temp7, 7
	rjmp busy_loop
	clr temp7
	out PORTA, temp7
	ser temp7
	out DDRF, temp7
.endmacro

; ------------------------------------------------------------------------------
; Data memory layout
; ------------------------------------------------------------------------------
.dseg
	; ----- Part 1: platform services -----
SystemTick:            .byte 1
ScrollTimer:           .byte 1
FlashTimer:            .byte 1
KeypadSnapshot:        .byte 1
KeypadHold:            .byte 1
ButtonSnapshot:        .byte 1
; ----- Stage indicators / debug -----
LastKeyEcho:           .byte 1          ; last ASCII key seen (for S1)
StageFlags:            .byte 1          ; bit0 = S1 done, others reserved

	; ----- Part 2: user configuration -----
AccidentX:             .byte 1
AccidentY:             .byte 1
Visibility:            .byte 1
InputCursor:           .byte 1          ; which field digit is being edited
ConfigFlags:           .byte 1
; live edit buffers for two-digit entry per field
XEditVal:              .byte 1          ; 0..99 temporary value for X
XEditCnt:              .byte 1          ; digits entered (0..2)
YEditVal:              .byte 1
YEditCnt:              .byte 1
VEditVal:              .byte 1
VEditCnt:              .byte 1

	; ----- Part 3: terrain + path data -----
MountainMatrix:			.byte MAP_CELLS
Cur_CoverageMask:		.byte MAP_CELLS
CoverageMask:			.byte MAP_CELLS  ; 0 = unseen, 1 = covered
check:	.byte MAP_CELLS
max_map: .byte MAP_CELLS
obs_x:	.byte MAP_CELLS
obs_y:	.byte MAP_CELLS

ObservationPath:       .byte PATH_BUF_BYTES
PathLength:            .byte 1          ; number of stored observation points
PathIndex:             .byte 1          ; active observation point
ScrollHead:            .byte 1          ; LCD scroll pointer into path buffer
QueueHead:             .byte 1          ; BFS helper
QueueTail:             .byte 1
QueueBuffer:           .byte MAP_CELLS

	; ----- Part 4: path playback state (speed/crash fixed) -----
DroneState:            .byte 1          ; STATE_* value
PlaybackIndex:         .byte 1          ; observation index shown on LCD
PlaybackTimer:         .byte 1          ; delays between observations
AltitudeDm:            .byte 1          ; locked to DEFAULT_ALT_DM
SpeedDmPerS:           .byte 1          ; locked to DEFAULT_SPEED_DMPS
AccidentFoundFlag:     .byte 1

	; LCD strings and scratch buffers
LCDLine0:              .byte LCD_COLS
LCDLine1:              .byte LCD_COLS
TwoCharBuffer:         .byte 2
PointRenderBuf:        .byte 8          ; e.g., "3,3,6/"

.cseg

; ------------------------------------------------------------------------------
; Interrupt vector table (add ISRs as needed)
; ------------------------------------------------------------------------------
.org 0x0000
	jmp RESET
.org INT0addr
	jmp INT0_ISR            ; PB0 if used as interrupt
.org INT1addr
	jmp INT1_ISR            ; PB1 if used as interrupt
; Ensure ascending vector order to avoid .cseg overlap.
.org OVF2addr
	jmp TIMER2_OVF_ISR      ; optional LED blink or keypad debounce
.org OVF0addr
	jmp TIMER0_OVF_ISR      ; used for 10 ms tick

; ------------------------------------------------------------------------------
; ------------------------------------------------------------------------------
; Program entry with step beacons to isolate failures
; ------------------------------------------------------------------------------
RESET:
    cli
    ; IMPORTANT: set stack before any rcall/ret
    ldi workA, high(RAMEND)
    out SPH, workA
    ldi workA, low(RAMEND)
    out SPL, workA
    rcall InitRegisters
    rcall InitIOPorts
    rcall InitLCDDriver
    rcall InitKeypad
    rcall InitTimers
    rcall InitStateMachine
    sei
    rcall Beacon
    rjmp MainLoop

; ------------------------------------------------------------------------------
; Main cooperative scheduler
; ------------------------------------------------------------------------------
MainLoop:
    rcall SampleInputs        ; read buttons/keypad, update snapshots
    rcall RunStateMachine     ; central dispatcher using DroneState (stubbed)
    rcall DriveOutputs        ; LCD heartbeat + LED bar
    rjmp MainLoop

; ==============================================================================
; Part 1: Platform services (initialisation, LCD, keypad, timing)
; ==============================================================================

InitStack:
	ldi workA, high(RAMEND)
	out SPH, workA
	ldi workA, low(RAMEND)
	out SPL, workA
	ret

InitRegisters:
	clr temp0
    clr workA
	clr temp2
	clr temp3
	clr temp4
	clr temp5
	clr temp6
	clr temp7
	clr temp8
	clr temp9
	clr temp10
	clr temp11
	clr temp12
	clr temp13
	ret

InitIOPorts:
	; Configure LCD data/control ports as outputs
	ldi workA, PORTFDIR
	out DDRF, workA
	out DDRA, workA

	; LED bar on PORTC
	ldi workA, LED_DDR_MASK
	out DDRC, workA
	clr temp0
	out PORTC, temp0

	; Keypad columns on PL7-4 outputs, rows on PL3-0 inputs w/ pull-ups
	ldi workA, PORTLDIR
	sts DDRL, workA
	ldi workA, INIT_COL_MASK
	sts PORTL, workA

	; Push buttons PB0 / PB1 as inputs with pull-ups
	cbi DDRB, 0
	cbi DDRB, 1
	sbi PORTB, 0
	sbi PORTB, 1
	ret

InitLCDDriver:
	; Use exact Lab 3/4 LCD init sequence/macros
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
	ldi data, LCD_DISPLAY | (1 << LCD_D) | (0 << LCD_C)
	lcd_write_com
	ret

InitKeypad:
	ldi workA, INIT_COL_MASK
	sts PORTL, workA
	clr temp0
	sts KeypadSnapshot, temp0
	sts KeypadHold, temp0
	ret

InitTimers:
	clr temp0
	out TCCR0A, temp0
	ldi workA, 0x03		; clk/64 prescaler
	out TCCR0B, workA
	ldi workA, (1 << TOIE0)
	sts TIMSK0, workA
	clr temp0
	out TCNT0, temp0

    ; Timer2 unused for now
    clr temp0
    ; TCCR2A/TCCR2B are in extended I/O on ATmega2560 -> use sts
    sts TCCR2A, temp0
    sts TCCR2B, temp0
    sts TIMSK2, temp0
    ret

InitStateMachine:
	clr temp0
	sts SystemTick, temp0
	sts ScrollTimer, temp0
	sts FlashTimer, temp0
	sts KeypadSnapshot, temp0
	sts KeypadHold, temp0
	sts ButtonSnapshot, temp0
	sts AccidentX, temp0
	sts AccidentY, temp0
	sts Visibility, temp0
	sts InputCursor, temp0
	sts ConfigFlags, temp0
	sts PathLength, temp0
	sts PathIndex, temp0
	sts ScrollHead, temp0
	sts QueueHead, temp0
	sts QueueTail, temp0
	sts DroneState, temp0
	sts PlaybackIndex, temp0
    sts PlaybackTimer, temp0
    sts AccidentFoundFlag, temp0

    ldi workA, DEFAULT_ALT_DM
    sts AltitudeDm, workA
    ldi workA, DEFAULT_SPEED_DMPS
    sts SpeedDmPerS, workA

    ; clear edit buffers
    sts XEditVal, temp0
    sts XEditCnt, temp0
    sts YEditVal, temp0
    sts YEditCnt, temp0
    sts VEditVal, temp0
    sts VEditCnt, temp0

	; Fill LCD buffers with spaces
	ldi YL, low(LCDLine0)
	ldi YH, high(LCDLine0)
	ldi workA, LCD_COLS
	mov temp1, workA
	ldi workA, ' '
	mov temp2, workA
fill_line0:
	st Y+, temp2
	dec temp1
	brne fill_line0
	ldi YL, low(LCDLine1)
	ldi YH, high(LCDLine1)
	ldi workA, LCD_COLS
	mov temp1, workA
fill_line1:
	st Y+, temp2
	dec temp1
	brne fill_line1
	ret

SampleInputs:
    rcall ScanKeypad         ; returns ASCII in workG or 0 if none
    rcall LatchKeyEvent

	in temp0, PINB
	com temp0
	mov workA, temp0
	andi workA, (PB0_MASK | PB1_MASK)
	mov temp0, workA
	sts ButtonSnapshot, temp0

    ; ----- S1: key echo baseline -----
    ; If there is a new KeypadSnapshot event, store it as LastKeyEcho and
    ; stamp the 'S1' tag once (bit0 in StageFlags ensures one-shot).
    lds workA, KeypadSnapshot
    tst workA
    breq s1_skip_update
    sts LastKeyEcho, workA
    lds workB, StageFlags
    ; Only stamp S1 if S2 not done (bit1==0) and S1 not yet stamped (bit0==0)
    sbrs workB, 1
    rjmp s1_check_b0
    rjmp s1_skip_update
s1_check_b0:
    sbrs workB, 0
    rjmp s1_set_tag
    rjmp s1_skip_update
s1_set_tag:
    ori workB, 1
    sts StageFlags, workB
s1_skip_update:
    ; If in CONFIG and we have a new key, process editing
    lds workB, DroneState
    cpi workB, STATE_CONFIG
    brne s1_done
    lds workA, KeypadSnapshot
    tst workA
    breq s1_done
    rcall ProcessConfigKey
s1_done:
    ret

DriveOutputs:
        ; Push LCD buffers to the display (spinner and echo removed for S3)
        rcall LcdWriteBuffer

	    ; Keep LEDs on as an additional sanity indicator
	    ser workA
	    out DDRC, workA
	    out PORTC, workA
	    ret

; ------------------------------------------------------------------------------
; LED flash routine (Lab 3 style). One on/off cycle per call.
; ------------------------------------------------------------------------------
LED_Flash:
	; LED chaser across PORTC (one-hot)
	ser workB
	out DDRC, workB
	ldi workA, 0x01
	ldi workC, 8
led_step_loop2:
	out PORTC, workA
	ldi temp7, low(40000)
	ldi temp8, high(40000)
	delay
	lsl workA
	dec workC
	brne led_step_loop2
	clr workA
	out PORTC, workA
	ret

; ------------------------------------------------------------------------------
; Beacon routines: show progress through RESET with 1..5 blinks
; Each blink ~200 ms on + 200 ms off
; ------------------------------------------------------------------------------
Beacon_delay:
	ldi temp7, low(12000)
	ldi temp8, high(12000)
	delay
	ret

Beacon:
    ; show ALL LEDs briefly after enabling interrupts
    ser workB
    out DDRC, workB
    ser workA
    out PORTC, workA
    rcall Beacon_delay
    clr workA
    out PORTC, workA
    ret

; ----- LCD helper macros ------------------------------------------------------
.macro PUSH_CTX
	push temp0
	push temp1
	push temp2
	push temp3
	push temp4
	push temp5
	push temp6
	push temp7
	push temp8
	push temp9
	push temp10
	push temp11
	push temp12
	push temp13
.endmacro

.macro POP_CTX
	pop temp13
	pop temp12
	pop temp11
	pop temp10
	pop temp9
	pop temp8
	pop temp7
	pop temp6
	pop temp5
	pop temp4
	pop temp3
	pop temp2
	pop temp1
	pop temp0
.endmacro



LcdClear:
	lcd_wait_busy
	ldi data, LCD_CLEAR
	lcd_write_com
	ret

LcdSetCursor:
	; expects row in workA, column in workB
	push workC
	mov workC, workA
	cpi workC, 0
	brne set_second_row
	ldi workC, 0
	rjmp cursor_addr2
set_second_row:
	ldi workC, SEC_DISPLAY_ADD
cursor_addr2:
	add workC, workB
	ori workC, LCD_SET_ADD
	lcd_wait_busy
	mov data, workC
	lcd_write_com
	pop workC
	ret

LcdWriteBuffer:
	push temp0
	push temp1
	push workA
	push workB
	push YH
	push YL

	; Line 0
	ldi workA, 0
	ldi workB, 0
	rcall LcdSetCursor
	ldi YL, low(LCDLine0)
	ldi YH, high(LCDLine0)
    ldi workC, LCD_COLS
    mov temp0, workC
write_line0_loop:
    ld data, Y+
    lcd_wait_busy
    lcd_write_data
    dec temp0
    brne write_line0_continue
    rjmp write_line0_done
write_line0_continue:
    rjmp write_line0_loop
write_line0_done:

	; Line 1
	ldi workA, 1
	ldi workB, 0
	rcall LcdSetCursor
	ldi YL, low(LCDLine1)
	ldi YH, high(LCDLine1)
    ldi workC, LCD_COLS
    mov temp0, workC
write_line1_loop:
    ld data, Y+
    lcd_wait_busy
    lcd_write_data
    dec temp0
    brne write_line1_continue
    rjmp write_line1_done
write_line1_continue:
    rjmp write_line1_loop
write_line1_done:

	pop YL
	pop YH
	pop workB
	pop workA
	pop temp1
	pop temp0
	ret

; Numeric/text formatting helpers for Figure 4 rendering
.macro FORMAT_TWO_DIGIT
	; @0 = source register (0-99), writes 2 ASCII chars via Z pointer
	; TODO: implement BCD/decimal conversion with leading space logic
.endmacro

.macro WRITE_COORD_TRIPLE
	; @0,@1,@2 = registers holding x,y,z; writes "x,y,z" via Z pointer
	; Each component limited to two chars + comma separators
.endmacro

; ----- Keypad helper routines -------------------------------------------------
; Lab-3 style scan + decode in one routine
; Output: workG = ASCII of key (or 0 if none)
ScanKeypad:
    ldi workC, INIT_COL_MASK   ; colmask (like lab colmask)
    clr workB                  ; col index (0..3)
    clr workG                  ; default: no key
scan_column_loop:
    cpi workB, 4
    breq scan_exit_none
    sts PORTL, workC           ; drive this column low, others high; rows pulled up
    ldi temp7, KEYPAD_SETTLE_LO
    ldi temp8, KEYPAD_SETTLE_HI
    delay
    ; read rows
    lds workE, PINL
    andi workE, ROWMASK        ; keep only PL3..PL0
    cpi workE, ROWMASK
    breq next_column           ; all 1s => no key in this column
    ; find row index (0..3), first 0 bit from LSB
    clr workD
row_loop:
    sbrs workE, 0
    rjmp have_row
    inc workD
    lsr workE
    rjmp row_loop
have_row:
    ; ignore letter column 3
    cpi workB, 3
    breq scan_exit_none
    ; map to ASCII
    cpi workD, 3
    breq sym_row
    ; digit '1'..'9': ascii = 3*row + col + '1'
    mov workG, workD
    lsl workG
    add workG, workD
    add workG, workB
    subi workG, -'1'
    rjmp scan_exit
sym_row:
    ; row==3 => symbols: col 0 '*' , col 1 '0', col 2 '#'
    cpi workB, 0
    breq sym_star
    cpi workB, 1
    breq sym_zero
    cpi workB, 2
    breq sym_pound
    rjmp scan_exit_none
sym_star:
    ldi workG, '*'
    rjmp scan_exit
sym_zero:
    ldi workG, '0'
    rjmp scan_exit
sym_pound:
    ldi workG, '#'
    rjmp scan_exit
next_column:
    lsl workC                   ; next column
    inc workB
    rjmp scan_column_loop
scan_exit_none:
    clr workG
scan_exit:
    ; restore columns default
    ldi workA, INIT_COL_MASK
    sts PORTL, workA
    ret

LatchKeyEvent:
	push temp0
	push temp1
	mov temp0, workG
	lds temp1, KeypadHold
	cp temp0, temp1
	brne latch_change
	clr temp0
	sts KeypadSnapshot, temp0
	rjmp latch_done
latch_change:
	sts KeypadHold, temp0
	tst temp0
	breq clear_event
	sts KeypadSnapshot, temp0
	rjmp latch_done
clear_event:
	clr temp0
	sts KeypadSnapshot, temp0
latch_done:
	pop temp1
	pop temp0
	ret

; ==============================================================================
; Part 2: User configuration workflow
; ==============================================================================

EnterConfigMode:
	; reserved for future use (manual entry trigger)
	ret

FinalizeConfig:
	; TODO: validate fields, show on LCD per spec
	ret

HandlePB0PathGen:
	; TODO: respond to PB0 press to trigger Part 3
	ret

BeginScrollPreview:
	; TODO: executed after PB0 path generation to enter STATE_SCROLL_PATH
	ret

BeginPlaybackRun:
	; TODO: executed when PB1 pressed, seed PlaybackIndex/timer and move to STATE_PLAYBACK
	ret

RunStateMachine:
    ; Central dispatcher by DroneState
    lds workA, DroneState
    cpi workA, STATE_IDLE
    breq RS_IDLE
    cpi workA, STATE_CONFIG
    breq RS_CONFIG
    cpi workA, STATE_PATH_GEN
    breq RS_PATHGEN
    cpi workA, STATE_SCROLL_PATH
    breq RS_SCROLL
    cpi workA, STATE_PLAYBACK
    breq RS_PLAY
    cpi workA, STATE_DONE
    breq RS_DONE
    ret
RS_IDLE:
    rcall HandleIdleState
    ret
RS_CONFIG:
    rcall HandleConfigState
    ret
RS_PATHGEN:
    rcall HandlePathGenState
    ret
RS_SCROLL:
    rcall HandleScrollState
    ret
RS_PLAY:
    rcall HandlePlaybackState
    ret
RS_DONE:
    rcall HandleDoneState
    ret

HandleIdleState:
    ; For now, auto-enter CONFIG on first tick for bring-up
    ldi workA, STATE_CONFIG
    sts DroneState, workA
    rcall UpdateLCDForConfig
    ret

HandleConfigState:
    ; If all three fields are set and S4 not yet stamped, run minimal path gen
    lds workD, ConfigFlags
    andi workD, 0x07
    cpi workD, 0x07
    brne hc_check_pb0
    ; all set; if S4 not set, build path and mark S4
    lds workB, StageFlags
    sbrs workB, 3
    rjmp hc_do_s4
    rjmp hc_check_pb0
hc_do_s4:
    rcall ResetCoverageMap
    rcall GenerateSearchPath
    lds workA, PathLength
    cpi workA, 0
    breq hc_check_pb0
    ori workB, (1<<3)
    sts StageFlags, workB
    ; auto-transition to scroll
    ldi workA, STATE_SCROLL_PATH
    sts DroneState, workA
    ret
hc_check_pb0:
    ; If PB0 pressed, transition to path generation state (will go to scroll)
    lds workB, ButtonSnapshot
    sbrs workB, 0              ; PB0 bit
    rjmp hc_render
    ldi workA, STATE_PATH_GEN
    sts DroneState, workA
    ret
hc_render:
    ; Ensure CONFIG screen reflects latest edits/commits
    rcall UpdateLCDForConfig
    ret

HandlePathGenState:
		; S4: reset coverage and build a trivial non-empty path
		rcall ResetCoverageMap
		rcall GenerateSearchPath
		; if PathLength > 0, set S4 flag and move to scroll
		lds workA, PathLength
		cpi workA, 0
		breq hpg_done
		; mark S4 in StageFlags (bit3)
		lds workB, StageFlags
		ori workB, (1<<3)
		sts StageFlags, workB
		; transition to scroll state (S5 will render)
		ldi workA, STATE_SCROLL_PATH
		sts DroneState, workA
hpg_done:
		ret

HandleScrollState:
		rcall UpdateLCDForScroll
		ret

HandlePlaybackState:
	; TODO: step through ObservationPath, highlight current point, display
	; TODO: second line "P 61 2" with bold coordinates per Figure 4(c)
	ret

HandleDoneState:
	; TODO: show final point + accident status as in Figure 4(d)
	ret

AdvanceScrollWindow:
	; TODO: use ScrollTimer to shift ScrollHead and rebuild LCDLine0
	ret

AdvancePlaybackStep:
	; TODO: increment PlaybackTimer and advance PlaybackIndex when elapsed
	ret

UpdateLCDForConfig:
    ; S2/S3: render two lines: "loc(__, __)" and "visib: __"
    ; Fill both lines with spaces
    ldi YL, low(LCDLine0)
    ldi YH, high(LCDLine0)
    ldi workA, LCD_COLS
    ldi workB, ' '
ulc_fill0:
    st Y+, workB
    dec workA
    brne ulc_fill0
    ldi YL, low(LCDLine1)
    ldi YH, high(LCDLine1)
    ldi workA, LCD_COLS
ulc_fill1:
    st Y+, workB
    dec workA
    brne ulc_fill1
    ; Build line 0: loc(__, __)
    ldi workC, 'l'
    sts LCDLine0+0, workC
    ldi workC, 'o'
    sts LCDLine0+1, workC
    ldi workC, 'c'
    sts LCDLine0+2, workC
    ldi workC, '('
    sts LCDLine0+3, workC
    ; X two-digit slot at [4],[5]
    lds workD, ConfigFlags
    sbrs workD, 0
    rjmp x_pending
    ; confirmed X -> show two digits
    lds workE, AccidentX
    cpi workE, 10
    brlo x_conf_lt10
    ldi workC, '1'
    sts LCDLine0+4, workC
    subi workE, 10
    rjmp x_conf_ones
x_conf_lt10:
    ldi workC, ' '
    sts LCDLine0+4, workC
x_conf_ones:
    ldi workC, '0'
    add workE, workC
    sts LCDLine0+5, workE
    rjmp after_x
x_pending:
    ; show edit buffer underscores/digits
    lds workE, XEditCnt
    cpi workE, 0
    breq x_unders
    cpi workE, 1
    breq x_one
    ; two digits in XEditVal
    lds workE, XEditVal
    cpi workE, 10
    brlo x_two_lt10
    ldi workC, '1'
    sts LCDLine0+4, workC
    subi workE, 10
    rjmp x_two_ones
x_two_lt10:
    ldi workC, ' '
    sts LCDLine0+4, workC
x_two_ones:
    ldi workC, '0'
    add workE, workC
    sts LCDLine0+5, workE
    rjmp after_x
x_one:
    ldi workC, '_'
    sts LCDLine0+4, workC
    lds workE, XEditVal
    ldi workC, '0'
    add workE, workC
    sts LCDLine0+5, workE
    rjmp after_x
x_unders:
    ldi workC, '_'
    sts LCDLine0+4, workC
    sts LCDLine0+5, workC
after_x:
    ; comma + space
    ldi workC, ','
    sts LCDLine0+6, workC
    ldi workC, ' '
    sts LCDLine0+7, workC
    ; Y two-digit slot at [8],[9]
    lds workD, ConfigFlags
    sbrs workD, 1
    rjmp y_pending
    lds workE, AccidentY
    cpi workE, 10
    brlo y_conf_lt10
    ldi workC, '1'
    sts LCDLine0+8, workC
    subi workE, 10
    rjmp y_conf_ones
y_conf_lt10:
    ldi workC, ' '
    sts LCDLine0+8, workC
y_conf_ones:
    ldi workC, '0'
    add workE, workC
    sts LCDLine0+9, workE
    rjmp after_y
y_pending:
    lds workE, YEditCnt
    cpi workE, 0
    breq y_unders
    cpi workE, 1
    breq y_one
    lds workE, YEditVal
    cpi workE, 10
    brlo y_two_lt10
    ldi workC, '1'
    sts LCDLine0+8, workC
    subi workE, 10
    rjmp y_two_ones
y_two_lt10:
    ldi workC, ' '
    sts LCDLine0+8, workC
y_two_ones:
    ldi workC, '0'
    add workE, workC
    sts LCDLine0+9, workE
    rjmp after_y
y_one:
    ldi workC, '_'
    sts LCDLine0+8, workC
    lds workE, YEditVal
    ldi workC, '0'
    add workE, workC
    sts LCDLine0+9, workE
    rjmp after_y
y_unders:
    ldi workC, '_'
    sts LCDLine0+8, workC
    sts LCDLine0+9, workC
after_y:
    ldi workC, ')'
    sts LCDLine0+10, workC
    ; Build line 1: vis: __ (two-digit slot at [4],[5])
    ldi workC, 'v'
    sts LCDLine1+0, workC
    ldi workC, 'i'
    sts LCDLine1+1, workC
    ldi workC, 's'
    sts LCDLine1+2, workC
    ldi workC, ':'
    sts LCDLine1+3, workC
    ; visibility two-digit or underscores at [4],[5]
    lds workD, ConfigFlags
    sbrs workD, 2
    rjmp vis_pending
    lds workE, Visibility
    cpi workE, 10
    brlo vis_lt10
    ldi workC, '1'
    sts LCDLine1+4, workC
    subi workE, 10
    rjmp vis_ones
vis_lt10:
    ldi workC, ' '
    sts LCDLine1+4, workC
vis_ones:
    ldi workC, '0'
    add workE, workC
    sts LCDLine1+5, workE
    rjmp after_vis
vis_pending:
    lds workE, VEditCnt
    cpi workE, 0
    breq vis_unders
    cpi workE, 1
    breq vis_one
    ; two digits in VEditVal
    lds workE, VEditVal
    cpi workE, 10
    brlo vis_two_lt10
    ldi workC, '1'
    sts LCDLine1+4, workC
    subi workE, 10
    rjmp vis_two_ones
vis_two_lt10:
    ldi workC, ' '
    sts LCDLine1+4, workC
vis_two_ones:
    ldi workC, '0'
    add workE, workC
    sts LCDLine1+5, workE
    rjmp after_vis
vis_one:
    ldi workC, '_'
    sts LCDLine1+4, workC
    lds workE, VEditVal
    ldi workC, '0'
    add workE, workC
    sts LCDLine1+5, workE
    rjmp after_vis
vis_unders:
    ldi workC, '_'
    sts LCDLine1+4, workC
    sts LCDLine1+5, workC
after_vis:
    ; If all three fields set (bits 0..2), echo two-digit values with spacing
    lds workD, ConfigFlags
    andi workD, 0x07
    cpi workD, 0x07
    brne no_cfg_echo
    ; X at [8],[9]
    lds workE, AccidentX
    ldi workC, ' '
    cpi workE, 10
    brlo echo_x_tens_done
    ldi workC, '1'
    subi workE, 10
echo_x_tens_done:
    sts LCDLine1+8, workC
    ldi workG, '0'
    add workE, workG
    sts LCDLine1+9, workE
    ; space at [10]
    ldi workE, ' '
    sts LCDLine1+10, workE
    ; Y at [11],[12]
    lds workE, AccidentY
    ldi workC, ' '
    cpi workE, 10
    brlo echo_y_tens_done
    ldi workC, '1'
    subi workE, 10
echo_y_tens_done:
    sts LCDLine1+11, workC
    ldi workG, '0'
    add workE, workG
    sts LCDLine1+12, workE
    ; space at [13]
    ldi workE, ' '
    sts LCDLine1+13, workE
    ; Vis at [14],[15]
    lds workE, Visibility
    ldi workC, ' '
    cpi workE, 10
    brlo echo_v_tens_done
    ldi workC, '1'
    subi workE, 10
echo_v_tens_done:
    sts LCDLine1+14, workC
    ldi workG, '0'
    add workE, workG
    sts LCDLine1+15, workE
no_cfg_echo:
    ; Stage stamp at end of first line (line0[14..15]): prefer highest stage set
    lds workD, StageFlags
    ; check S4
    sbrs workD, 3
    rjmp stamp_check_s3
    ldi workC, 'S'
    sts LCDLine0+14, workC
    ldi workC, '4'
    sts LCDLine0+15, workC
    rjmp stamp_done
stamp_check_s3:
    ; check S3
    sbrs workD, 2
    rjmp stamp_check_s2
    ldi workC, 'S'
    sts LCDLine0+14, workC
    ldi workC, '3'
    sts LCDLine0+15, workC
    rjmp stamp_done
stamp_check_s2:
    sbrs workD, 1
    rjmp stamp_check_s1
    ldi workC, 'S'
    sts LCDLine0+14, workC
    ldi workC, '2'
    sts LCDLine0+15, workC
    rjmp stamp_done
stamp_check_s1:
    sbrs workD, 0
    rjmp stamp_done
    ldi workC, 'S'
    sts LCDLine0+14, workC
    ldi workC, '1'
    sts LCDLine0+15, workC
stamp_done:
    ret


; S3: process keypad input for config editing
ProcessConfigKey:
    ; workA holds the ASCII key from KeypadSnapshot
    ; '#' advances cursor; digits set current field; '*' clears current field
    ; Cursor mapping: 0=X, 1=Y, 2=Visibility
    ; '#' advances, '*' clears (use near branches + rjmp for long targets)
    cpi workA, '#'
    brne key_not_hash
    rjmp cfg_next
key_not_hash:
    cpi workA, '*'
    brne key_not_star
    rjmp cfg_clear
key_not_star:
    ; digits '0'..'9' gate using near branches
    cpi workA, '0'
    brsh cfg_digit_ge0        ; if >= '0' continue
    rjmp cfg_ret              ; else reject
cfg_digit_ge0:
    cpi workA, '9'+1          ; compare to ':' (one past '9')
    brlo cfg_digit_islte9     ; if < ':' then it's <= '9'
    rjmp cfg_ret              ; else reject
cfg_digit_islte9:
    ; convert to numeric in workE
    mov workE, workA
    subi workE, '0'
    ; fetch cursor
    lds workB, InputCursor
    cpi workB, 0
    breq edit_x
    cpi workB, 1
    breq edit_y
    ; else visibility editing
edit_vis:
    ; append digit into VEditVal/VEditCnt if room
    lds workD, VEditCnt
    cpi workD, 0
    breq v_first
    cpi workD, 1
    breq v_second
    rjmp cfg_refresh
v_first:
    sts VEditVal, workE
    ldi workD, 1
    sts VEditCnt, workD
    rjmp cfg_refresh
v_second:
    ; new = old*10 + digit
    lds workF, VEditVal   ; old
    mov workG, workF      ; *2
    lsl workG
    mov workA, workF      ; *8
    lsl workA
    lsl workA
    lsl workA
    add workA, workG      ; *10
    add workA, workE      ; +digit
    sts VEditVal, workA
    ldi workD, 2
    sts VEditCnt, workD
    rjmp cfg_refresh
edit_y:
    lds workD, YEditCnt
    cpi workD, 0
    breq y_first
    cpi workD, 1
    breq y_second
    rjmp cfg_refresh
y_first:
    sts YEditVal, workE
    ldi workD, 1
    sts YEditCnt, workD
    rjmp cfg_refresh
y_second:
    lds workF, YEditVal
    mov workG, workF
    lsl workG              ; *2
    mov workA, workF
    lsl workA
    lsl workA
    lsl workA              ; *8
    add workA, workG       ; *10
    add workA, workE
    sts YEditVal, workA
    ldi workD, 2
    sts YEditCnt, workD
    rjmp cfg_refresh
edit_x:
    lds workD, XEditCnt
    cpi workD, 0
    breq x_first
    cpi workD, 1
    breq x_second
    rjmp cfg_refresh
x_first:
    sts XEditVal, workE
    ldi workD, 1
    sts XEditCnt, workD
    rjmp cfg_refresh
x_second:
    lds workF, XEditVal
    mov workG, workF
    lsl workG
    mov workA, workF
    lsl workA
    lsl workA
    lsl workA
    add workA, workG
    add workA, workE
    sts XEditVal, workA
    ldi workD, 2
    sts XEditCnt, workD
    rjmp cfg_refresh
cfg_next:
    ; confirm current field (use edit buffers) then advance cursor 0->1->2
    lds workB, InputCursor
    cpi workB, 0
    breq confirm_x
    cpi workB, 1
    breq confirm_y
    rjmp confirm_vis
confirm_x:
    lds workD, XEditCnt
    cpi workD, 0
    breq advance_cursor
    lds workE, XEditVal
    cpi workE, 15
    brlo x_inrange
    ldi workE, 14
x_inrange:
    sts AccidentX, workE
    lds workD, ConfigFlags
    ori workD, (1<<0)
    sts ConfigFlags, workD
    clr workD
    sts XEditCnt, workD
    sts XEditVal, workD
    rjmp advance_cursor
confirm_y:
    lds workD, YEditCnt
    cpi workD, 0
    breq advance_cursor
    lds workE, YEditVal
    cpi workE, 15
    brlo y_inrange
    ldi workE, 14
y_inrange:
    sts AccidentY, workE
    lds workD, ConfigFlags
    ori workD, (1<<1)
    sts ConfigFlags, workD
    clr workD
    sts YEditCnt, workD
    sts YEditVal, workD
    rjmp advance_cursor
confirm_vis:
    lds workD, VEditCnt
    cpi workD, 0
    breq advance_cursor
    lds workE, VEditVal
    cpi workE, 16
    brlo v_inrange
    ldi workE, 15
v_inrange:
    sts Visibility, workE
    lds workD, ConfigFlags
    ori workD, (1<<2)
    sts ConfigFlags, workD
    clr workD
    sts VEditCnt, workD
    sts VEditVal, workD
advance_cursor:
    lds workB, InputCursor
    inc workB
    cpi workB, 3
    brlo cfg_store_cursor
    ldi workB, 2
cfg_store_cursor:
    sts InputCursor, workB
    rjmp cfg_tag
cfg_clear:
    ; clear current field and its flag
    lds workB, InputCursor
    cpi workB, 0
    breq clr_x
    cpi workB, 1
    breq clr_y
    ; visibility
    clr workE
    sts Visibility, workE
    lds workD, ConfigFlags
    andi workD, 0b11111011
    sts ConfigFlags, workD
    rjmp cfg_refresh
clr_y:
    clr workE
    sts YEditVal, workE
    sts YEditCnt, workE
    lds workD, ConfigFlags
    andi workD, 0b11111101
    sts ConfigFlags, workD
    rjmp cfg_refresh
clr_x:
    clr workE
    sts XEditVal, workE
    sts XEditCnt, workE
    lds workD, ConfigFlags
    andi workD, 0b11111110
    sts ConfigFlags, workD
    rjmp cfg_refresh
cfg_tag:
    ; Stamp S3 once
    lds workD, StageFlags
    sbrs workD, 2
    rjmp do_s3
    rjmp cfg_refresh
do_s3:
    ori workD, 0x04
    sts StageFlags, workD
cfg_refresh:      ; re-render config UI after a change
    rcall UpdateLCDForConfig
cfg_ret:          ; common return
    ret

; =============================================================================
; Part 3: Terrain modelling and search-path generation
; =============================================================================

BuildMountainModel:
	load_array_from_program m, MountainMatrix, MAP_SIZE
	ret

ResetCoverageMap:
	; Clear CoverageMask array and path indices
	load_array_from_program zeros, Cur_CoverageMask, MAP_SIZE
	load_array_from_program zeros, CoverageMask, MAP_SIZE
	ret

GenerateSearchPath:
	; Build a trivial non-empty path: (X,Y,0) and (X,Y,Vis)
	push YL
	push YH
	ldi YL, low(ObservationPath)
	ldi YH, high(ObservationPath)
	; Load current config
	lds workC, AccidentX
	lds workD, AccidentY
	lds workE, Visibility
	; Point 0: (X,Y,0)
	st Y+, workC
	st Y+, workD
	clr workA
	st Y+, workA
	; Point 1: (X,Y,Vis)
	st Y+, workC
	st Y+, workD
	st Y+, workE
	; PathLength = 2, PathIndex=0
	ldi workA, 2
	sts PathLength, workA
	clr workA
	sts PathIndex, workA
	pop YH
	pop YL
	ret

FindNextObservation:
	; TODO: evaluate remaining cells and pick best observation point
	ret

UpdateCoverageForPoint:
	; TODO: mark CoverageMask entries covered from current observation
	ret

StoreObservationPoint:
	; TODO: push x,y,z into ObservationPath buffer
	ret

PreparePathScrollData:
	; TODO: format buffer for LCD scrolling (line 0) "0,0,0 / 3,3,6 / ..."
	; TODO: ensure segments fit 16 chars and wrap like Figure 4(b)
	ret

; =============================================================================
; Part 4: Search-path playback and LCD formatting (no dynamic speed/crash)
; =============================================================================
UpdateLCDForScroll:
		; Render first two path points as "xx,yy,zz / xx,yy" (fits 16 cols)
		push YL
		push YH
        ; Clear BOTH lines to prevent leftover S3 text (e.g., "loc(x,y)")
        ; line 0
        ldi YL, low(LCDLine0)
        ldi YH, high(LCDLine0)
        ldi workA, LCD_COLS
        ldi workB, ' '
uls_fill0:
                st Y+, workB
                dec workA
                brne uls_fill0
        ; line 1
        ldi YL, low(LCDLine1)
        ldi YH, high(LCDLine1)
        ldi workA, LCD_COLS
uls_fill1:
                st Y+, workB
                dec workA
                brne uls_fill1
		; load first two points from ObservationPath
		ldi YL, low(ObservationPath)
		ldi YH, high(ObservationPath)
		ld workC, Y+   ; x0
		ld workD, Y+   ; y0
		ld workE, Y+   ; z0
		ld workF, Y+   ; x1
		ld workG, Y+   ; y1
		; x0 -> [0],[1]
		mov workA, workC
		cpi workA, 10
		brlo uls_x0_lt10
		ldi workB, '1'
		subi workA, 10
		rjmp uls_x0_ones
uls_x0_lt10:
		ldi workB, ' '
uls_x0_ones:
		ldi workC, '0'
		add workA, workC
                sts LCDLine1+0, workB
                sts LCDLine1+1, workA
		; comma
		ldi workA, ','
                sts LCDLine1+2, workA
		; y0 -> [3],[4]
		mov workA, workD
		cpi workA, 10
		brlo uls_y0_lt10
		ldi workB, '1'
		subi workA, 10
		rjmp uls_y0_ones
uls_y0_lt10:
		ldi workB, ' '
uls_y0_ones:
		ldi workC, '0'
		add workA, workC
                sts LCDLine1+3, workB
                sts LCDLine1+4, workA
		; comma
		ldi workA, ','
                sts LCDLine1+5, workA
		; z0 -> [6],[7]
		mov workA, workE
		cpi workA, 10
		brlo uls_z0_lt10
		ldi workB, '1'
		subi workA, 10
		rjmp uls_z0_ones
uls_z0_lt10:
		ldi workB, ' '
uls_z0_ones:
		ldi workC, '0'
		add workA, workC
                sts LCDLine1+6, workB
                sts LCDLine1+7, workA
		; space, '/', space
		ldi workA, ' '
                sts LCDLine1+8, workA
		ldi workA, '/'
                sts LCDLine1+9, workA
		ldi workA, ' '
                sts LCDLine1+10, workA
		; x1 -> [11],[12]
		mov workA, workF
		cpi workA, 10
		brlo uls_x1_lt10
		ldi workB, '1'
		subi workA, 10
		rjmp uls_x1_ones
uls_x1_lt10:
		ldi workB, ' '
uls_x1_ones:
		ldi workC, '0'
		add workA, workC
                sts LCDLine1+11, workB
                sts LCDLine1+12, workA
		; comma
		ldi workA, ','
                sts LCDLine1+13, workA
		; y1 -> [14],[15]
		mov workA, workG
		cpi workA, 10
		brlo uls_y1_lt10
		ldi workB, '1'
		subi workA, 10
		rjmp uls_y1_ones
uls_y1_lt10:
		ldi workB, ' '
uls_y1_ones:
		ldi workC, '0'
		add workA, workC
                sts LCDLine1+14, workB
                sts LCDLine1+15, workA
                ; Stamp stage on line 0 end (14..15) after clearing
                lds workD, StageFlags
                ; S4
                sbrs workD, 3
                rjmp uls_stamp_s3
                ldi workC, 'S'
                sts LCDLine0+14, workC
                ldi workC, '4'
                sts LCDLine0+15, workC
                rjmp uls_stamp_done
uls_stamp_s3:
                sbrs workD, 2
                rjmp uls_stamp_s2
                ldi workC, 'S'
                sts LCDLine0+14, workC
                ldi workC, '3'
                sts LCDLine0+15, workC
                rjmp uls_stamp_done
uls_stamp_s2:
                sbrs workD, 1
                rjmp uls_stamp_s1
                ldi workC, 'S'
                sts LCDLine0+14, workC
                ldi workC, '2'
                sts LCDLine0+15, workC
                rjmp uls_stamp_done
uls_stamp_s1:
                sbrs workD, 0
                rjmp uls_stamp_done
                ldi workC, 'S'
                sts LCDLine0+14, workC
                ldi workC, '1'
                sts LCDLine0+15, workC
uls_stamp_done:
                pop YH
                pop YL
                ret

UpdateLCDForPlayback:
	; TODO: line0 emphasises current point, line1 prints state+alt+speed
	ret

INT0_ISR:
	; TODO: optional PB0 interrupt handling
	reti

INT1_ISR:
	; TODO: optional PB1 interrupt handling
	reti

TIMER0_OVF_ISR:
	push temp0
	push temp1
	in temp0, SREG
	push temp0
	lds temp0, SystemTick
	inc temp0
	sts SystemTick, temp0
	lds temp1, ScrollTimer
	inc temp1
	sts ScrollTimer, temp1
	lds temp1, PlaybackTimer
	inc temp1
	sts PlaybackTimer, temp1
	pop temp0
	out SREG, temp0
	pop temp1
	pop temp0
	reti

TIMER2_OVF_ISR:
	; TODO: LED blink timing or additional periodic work
	reti


;-------------------------------------------------------------------------------
; Program Data Definition
;-------------------------------------------------------------------------------
m: .db	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
		0,2,2,2,2,2,2,2,0,0,0,0,0,0,0, \
		0,2,4,4,4,4,4,2,2,2,2,2,2,2,2, \
		0,2,4,6,6,6,4,2,4,4,4,4,4,4,4, \
		0,2,4,6,8,6,4,2,4,6,6,6,6,6,4, \
		0,2,4,6,6,6,4,2,4,6,8,8,8,6,4, \
		0,2,4,4,4,4,4,2,4,6,8,10,8,6,4, \
		0,2,2,2,2,2,2,2,4,6,8,8,8,6,4, \
		0,0,2,2,2,2,2,2,4,6,6,6,6,6,4, \
		0,0,2,4,4,4,2,2,4,4,4,4,4,4,4, \
		0,0,2,4,6,4,2,2,2,2,2,2,2,2,2, \
		0,0,2,4,4,4,2,0,0,0,0,0,0,0,0, \
		0,0,2,2,2,2,2,0,0,0,0,0,0,0,0, \
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

zeros: .db	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

; ==============================================================================
; End of stub
; ==============================================================================
