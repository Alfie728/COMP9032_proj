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
.equ MAP_SIZE           = 7
.equ MAP_CELLS          = 49
.equ MAX_OBS_POINTS     = 16           ; supports multiple coverage groups
.equ OBS_POINT_STRIDE   = 3            ; x, y, z (height)
.equ PATH_BUF_BYTES     = MAX_OBS_POINTS * OBS_POINT_STRIDE
.equ LCD_COLS           = 16
.equ LCD_ROWS           = 2
.equ VISIBILITY_MAX     = 9            ; displayed in two chars
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
.equ KEYPAD_SETTLE_LO   = low(3000)
.equ KEYPAD_SETTLE_HI   = high(3000)

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

	; ----- Part 2: user configuration -----
AccidentX:             .byte 1
AccidentY:             .byte 1
Visibility:            .byte 1
InputCursor:           .byte 1          ; which field digit is being edited
ConfigFlags:           .byte 1

	; ----- Part 3: terrain + path data -----
MountainMatrix:        .byte MAP_CELLS
CoverageMask:          .byte MAP_CELLS  ; 0 = unseen, 1 = covered
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
; Program entry
; ------------------------------------------------------------------------------
RESET:
	cli
	rcall InitStack
	rcall InitRegisters
	rcall InitIOPorts
	rcall DisableJTAG
	; LED sanity check identical to Lab3 LED_flash
	rcall LED_Flash
	rcall LED_Port_Sweep
	rcall InitLCDDriver
	rcall InitKeypad
	rcall InitTimers
	rcall InitStateMachine
	sei
	rjmp MainLoop

; ------------------------------------------------------------------------------
; Main cooperative scheduler
; ------------------------------------------------------------------------------
MainLoop:
	rcall SampleInputs        ; read buttons/keypad, update snapshots
	rcall RunStateMachine     ; central dispatcher using DroneState
	rcall DriveOutputs        ; LCD, LED bar, buzzer if any
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

	; Sanity: write a visible character to DDRAM
	lcd_wait_busy
	ldi data, '('
	lcd_write_data
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
	rcall ScanKeypad
	rcall DecodeKey
	rcall LatchKeyEvent

	in temp0, PINB
	com temp0
	mov workA, temp0
	andi workA, (PB0_MASK | PB1_MASK)
	mov temp0, workA
	sts ButtonSnapshot, temp0
	ret

DriveOutputs:
    ; Force LEDs on as a hardware sanity check
    ser workA
    out PORTC, workA

    ; Heartbeat spinner at LCD [0,0] using ScrollTimer bit 7
    lds workB, ScrollTimer
    sbrs workB, 7
    rjmp hb_minus
    ldi workC, '|'
    rjmp hb_store
hb_minus:
    ldi workC, '-'
hb_store:
    sts LCDLine0, workC

	rcall LcdWriteBuffer
	ret

; ------------------------------------------------------------------------------
; Lab 3 style LED flash (sanity check: verifies clock + PORTC wiring)
; ------------------------------------------------------------------------------
LED_Flash:
	; Use two high registers distinct from temp7/temp8
	; workA (r17) drives LED value, workB (r18) assists with delays
	ser workA
	out PORTC, workA
	ldi workA, low(55000)
	ldi workB, high(55000)
	mov temp7, workA
	mov temp8, workB
	delay
	clr workA
	out PORTC, workA
	ldi workA, low(55000)
	ldi workB, high(55000)
	mov temp7, workA
	mov temp8, workB
	delay
	ser workA
	out PORTC, workA
	ldi workA, low(55000)
	ldi workB, high(55000)
	mov temp7, workA
	mov temp8, workB
	delay
	clr workA
	out PORTC, workA
	ldi workA, low(55000)
	ldi workB, high(55000)
	mov temp7, workA
	mov temp8, workB
	delay
	ret

; ------------------------------------------------------------------------------
; Sweep candidate LED ports (C -> H -> B) with visible 1s on/off
; ------------------------------------------------------------------------------
LED_Port_Sweep:
	; constants for ~1s delay
	ldi temp7, low(60000)
	ldi temp8, high(60000)
	; --- PORTC ---
	ser workA
	out DDRC, workA
	out PORTC, workA
	push temp7
	push temp8
	ldi temp7, low(60000)
	ldi temp8, high(60000)
	delay
	clr workA
	out PORTC, workA
	ldi temp7, low(60000)
	ldi temp8, high(60000)
	delay
	pop temp8
	pop temp7

	; --- PORTH --- (extended I/O -> use sts)
	ser workA
	sts DDRH, workA
	sts PORTH, workA
	ldi temp7, low(60000)
	ldi temp8, high(60000)
	delay
	clr workA
	sts PORTH, workA
	ldi temp7, low(60000)
	ldi temp8, high(60000)
	delay

	; --- PORTB ---
	ser workA
	out DDRB, workA
	out PORTB, workA
	ldi temp7, low(60000)
	ldi temp8, high(60000)
	delay
	clr workA
	out PORTB, workA
	ldi temp7, low(60000)
	ldi temp8, high(60000)
	delay
	ret

; ------------------------------------------------------------------------------
; Disable JTAG so PORTF/PORTC pins operate as GPIO (two writes to MCUCR)
; ------------------------------------------------------------------------------
DisableJTAG:
	lds workB, MCUCR
	ori workB, (1<<JTD)
	sts MCUCR, workB
	sts MCUCR, workB
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
ScanKeypad:
	ldi workA, INIT_COL_MASK
	clr workB                 ; column index
	ldi workF, 0xFF           ; assume no key
	sts PORTL, workA
scan_column_loop:
	cpi workB, 4
	breq scan_done
	sts PORTL, workA
	ldi workC, KEYPAD_SETTLE_LO
	mov temp7, workC
	ldi workC, KEYPAD_SETTLE_HI
	mov temp8, workC
	delay
	lds workC, PINL
	andi workC, ROWMASK
	cpi workC, ROWMASK
	breq next_column
	clr workD                 ; row index
scan_row_loop:
	sbrs workC, 0
	rjmp key_identified
	inc workD
	lsr workC
	rjmp scan_row_loop
key_identified:
	mov workE, workD
	mov workF, workB
	rjmp scan_exit
next_column:
	lsl workA
	inc workB
	rjmp scan_column_loop
scan_done:
	ldi workF, 0xFF
scan_exit:
	ldi workC, INIT_COL_MASK
	sts PORTL, workC
	ret

DecodeKey:
	cpi workF, 0xFF
	breq no_key_pressed
	cpi workF, 3
	breq no_key_pressed      ; ignore letter column for now
	cpi workE, 3
	breq keypad_symbols
	mov workG, workE
	lsl workG
	add workG, workE
	add workG, workF
	subi workG, -'1'
	rjmp decoded
keypad_symbols:
	cpi workF, 0
	breq symbol_star
	cpi workF, 1
	breq symbol_zero
	cpi workF, 2
	breq symbol_pound
	rjmp no_key_pressed
symbol_star:
	ldi workG, '*'
	rjmp decoded
symbol_zero:
	ldi workG, '0'
	rjmp decoded
symbol_pound:
	ldi workG, '#'
	rjmp decoded
no_key_pressed:
	clr workG
decoded:
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
	; TODO: invoked after reset button, gather accident location & visibility
	ret

ProcessConfigKey:
	; TODO: update InputCursor, store digits in AccidentX/Y/Visibility
	ret

FinalizeConfig:
	; TODO: validate fields, show on LCD per spec
	ret

HandlePB0PathGen:
	; TODO: respond to PB0 press to trigger Part 3
	ret

; ==============================================================================
; Part 3: Terrain modelling and search-path generation
; ==============================================================================

BuildMountainModel:
	; TODO: populate MountainMatrix with chosen primitive shapes
	ret

ResetCoverageMap:
	; TODO: clear CoverageMask, PathLength, PathIndex
	ret

GenerateSearchPath:
	; TODO: observation planning loop based on visibility distance
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

; ==============================================================================
; Part 4: Search-path playback and LCD formatting (no dynamic speed/crash)
; ==============================================================================

BeginScrollPreview:
	; TODO: executed after PB0 path generation to enter STATE_SCROLL_PATH
	ret

BeginPlaybackRun:
	; TODO: executed when PB1 pressed, seed PlaybackIndex/timer and move to STATE_PLAYBACK
	ret

RunStateMachine:
	; TODO: branch on DroneState and call config/path/playback handlers
	ret

HandleIdleState:
	; TODO: wait for reset input, clear LCD, display prompt
	ret

HandleConfigState:
	; TODO: update LCD with "loc:(x,y)" and "visib: d" per Figure 4(a)
	ret

HandlePathGenState:
	; TODO: call GenerateSearchPath and transition to STATE_SCROLL_PATH
	ret

HandleScrollState:
	; TODO: show observation list ("0,0,0 / 3,3,6 / ...") scrolling right->left
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
	; TODO: write pre-search info lines (loc/visib) to LCDLine buffers
	ret

UpdateLCDForScroll:
	; TODO: convert ObservationPath into slash-separated ASCII chunk
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

; ==============================================================================
; End of stub
; ==============================================================================
