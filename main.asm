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
.def temp7      = r9
.def temp8      = r10
.def temp9      = r11
.def temp10     = r12
.def temp11     = r13
.def temp12     = r14
.def temp13     = r15
.def lcd_data   = r16
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

; Timing constants
.equ TICK_10MS          = 100          ; placeholder for software timer loops
.equ SCROLL_PERIOD      = 5            ; number of ticks between LCD scrolls

; ------------------------------------------------------------------------------
; Data memory layout
; ------------------------------------------------------------------------------
.dseg
	; ----- Part 1: platform services -----
SystemTick:            .byte 1
ScrollTimer:           .byte 1
FlashTimer:            .byte 1
KeypadSnapshot:        .byte 1
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
	rjmp RESET
.org INT0addr
	rjmp INT0_ISR            ; PB0 if used as interrupt
.org INT1addr
	rjmp INT1_ISR            ; PB1 if used as interrupt
.org OVF0addr
	rjmp TIMER0_OVF_ISR      ; used for 10 ms tick
.org OVF2addr
	rjmp TIMER2_OVF_ISR      ; optional LED blink or keypad debounce

; ------------------------------------------------------------------------------
; Program entry
; ------------------------------------------------------------------------------
RESET:
	cli
	rcall InitStack
	rcall InitRegisters
	rcall InitIOPorts
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
	ldi temp0, high(RAMEND)
	out SPH, temp0
	ldi temp0, low(RAMEND)
	out SPL, temp0
	ret

InitRegisters:
	clr temp0
	clr temp1
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
	; TODO: configure LCD, keypad, push buttons, LED bar ports
	ret

InitLCDDriver:
	; TODO: send initialisation sequence
	ret

InitKeypad:
	; TODO: set DDR for keypad rows/columns
	ret

InitTimers:
	; TODO: configure Timer0 for periodic tick, Timer2 for PWM/LED/scrolling
	ret

InitStateMachine:
	; TODO: zero SRAM state, set DroneState to STATE_IDLE
	ret

SampleInputs:
	; TODO: poll keypad by scanning columns, store result in KeypadSnapshot
	; TODO: read push buttons PB0/PB1 into ButtonSnapshot
	ret

DriveOutputs:
	; TODO: push buffered LCD lines, optionally blink LED bar when playback ends
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

.macro LCD_WRITE_CMD
	; @0 = command byte in register
	out PORTF, @0
	cbi PORTA, LCD_RS
	cbi PORTA, LCD_RW
	sbi PORTA, LCD_E
	nop
	cbi PORTA, LCD_E
.endmacro

.macro LCD_WRITE_DATA
	out PORTF, @0
	sbi PORTA, LCD_RS
	cbi PORTA, LCD_RW
	sbi PORTA, LCD_E
	nop
	cbi PORTA, LCD_E
.endmacro

WaitLcdBusy:
	; TODO: poll busy flag using PINF when DDRF configured as input
	ret

LcdClear:
	; TODO: wrap LCD_WRITE_CMD with LCD clear constant
	ret

LcdSetCursor:
	; TODO: accept row/col in registers and compute DDRAM address
	ret

LcdWriteBuffer:
	; TODO: push contents of LCDLine0/LCDLine1 to display
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
	; TODO: drive columns sequentially and read rows
	ret

DecodeKey:
	; TODO: translate matrix code into symbolic key (digits, Ua/Da/etc.)
	ret

LatchKeyEvent:
	; TODO: debounce and store into KeypadSnapshot
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
	; TODO: software tick increment, keypad debounce scheduler
	reti

TIMER2_OVF_ISR:
	; TODO: LED blink timing or additional periodic work
	reti

; ==============================================================================
; End of stub
; ==============================================================================
