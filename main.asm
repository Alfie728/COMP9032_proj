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
.equ size3 = 15
.equ l3 = size3 * size3
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

;-------------------------------------------------------------------------------
; Macro Definition
;-------------------------------------------------------------------------------
;void load_array_from_program(program addr, data addr, int size)
;Description: data addr <- program addr
;r16: temp, r17: counter
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

;void load_array_from_program(data addr1, data addr2, int size) 
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

	; counter = 0
	clr r6
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
map:	.byte l3
c_vis:	.byte l3
p_vis:	.byte l3
check:	.byte l3
max_map: .byte l3
obs_x:	.byte l3
obs_y:	.byte l3
obs_z:	.byte l3
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

; void dfs(org_x, org_y, cur_x, cur_y, visability)
; Description: dfs
; r18 = org_x, r19 = org_y, r20 = cur_x, r21 = cur_y, r22 = visability, r23, r24 = temp
dfs:
	push yh
	push yl
	push xh
	push xl
	push r23
	in r23, SREG
	push r23
	push r24
	in yl, SPL
	in yh, SPH
	sbiw y, 5
	out SPH, yh
	out SPL, yl
	std y+1, r18
	std y+2, r19
	std y+3, r20
	std y+4, r21
	std y+5, r22
	
	ldd r22, y+5
	ldd r21, y+4
	ldd r20, y+3
	ldd r19, y+2
	ldd r18, y+1

	; if check[cur_y][cur_x] == 1:
	array_i_j check, size3, r20, r21
	ld r23, x
	cpi r23, 1
	in r24, SREG
	sbrc r24, 1
	jmp dfs_end

	; if (org_x - cur_x)**2 + (org_y - cur_y)**2 + (mountain[org_y][org_x] - mountain[cur_y][cur_x])**2 <= v**2:
	array_i_j map, size3, r18, r19
	ld r23, x
	array_i_j map, size3, r20, r21
	ld r24, x
	sub r23, r24
	brpl positive_dz
	neg r23
	positive_dz:

	sub r18, r20
	brpl positive_dx
	neg r18
	positive_dx:

	sub r19, r21
	brpl positive_dy
	neg r19
	positive_dy:

	mul r18, r18
	mov r18, r0

	mul r19, r19
	mov r19, r0

	mul r23, r23
	mov r23, r0

	mul r22, r22
	mov r22, r0

	add r18, r19
	in r24, SREG
	sbrc r24, 3
	jmp dfs_end

	add r18, r23
	in r24, SREG
	sbrc r24, 3
	jmp dfs_end

	cp r22, r18
	in r24, SREG
	sbrc r24, 0
	jmp dfs_end

	; c_vis[cur_y][cur_x] = 1
	array_i_j c_vis, size3, r20, r21
	ldi r23, 1
	st x, r23

	; check[cur_y][cur_x] = 1
	array_i_j check, size3, r20, r21
	ldi r23, 1
	st x, r23

left_if:
	ldd r22, y+5
	ldd r21, y+4
	ldd r20, y+3
	ldd r19, y+2
	ldd r18, y+1
	; if cur_x - 1 >= 0:
	subi r20, 1
	brmi right_if
	rcall dfs
right_if:
	ldd r22, y+5
	ldd r21, y+4
	ldd r20, y+3
	ldd r19, y+2
	ldd r18, y+1
	; if cur_x + 1 < len(mountain):
	inc r20
	cpi r20, size3
	brsh down_if
	rcall dfs
down_if:
	ldd r22, y+5
	ldd r21, y+4
	ldd r20, y+3
	ldd r19, y+2
	ldd r18, y+1
	; if cur_y - 1 >= 0:
	subi r21, 1
	brmi up_if
	rcall dfs
up_if:
	ldd r22, y+5
	ldd r21, y+4
	ldd r20, y+3
	ldd r19, y+2
	ldd r18, y+1
	; if cur_y + 1 < len(mountain):
	inc r21
	cpi r21, size3
	brsh dfs_end
	rcall dfs
dfs_end:
	adiw y, 5
	out SPH, yh
	out SPL, yl
	pop r24
	pop r23
	out SREG, r23
	pop r23
	pop xl
	pop xh
	pop yl
	pop yh
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
