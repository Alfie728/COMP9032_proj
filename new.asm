;
; alg.asm
;
; Created: 2025/11/21 9:48:51
; Author : hsk
;
.include "m2560def.inc"

.equ MAP_SIZE           = 15							; maximum grid 15x15
.equ MAP_CELLS          = (MAP_SIZE*MAP_SIZE)
.equ MAX_OBS_POINTS     = (MAP_SIZE*MAP_SIZE)           ; supports multiple coverage groups
.equ OBS_POINT_STRIDE   = 3								; x, y, z (height)
.equ PATH_BUF_BYTES     = MAX_OBS_POINTS * OBS_POINT_STRIDE

; Replace with your application code
;void load_array_from_program(program addr, data addr, int length)
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
; Description: array[i]
;Rd != r16/r17, r16: temp
.macro array_i
	push yh
	push yl
	push r16
	in r16, SREG
	push r16

	ldi yh, high(@0)
	ldi yl, low(@0)
	add yl, @1
	ldi r16, 0
	adc yh, r16
	mov xl, yl
	mov xh, yh

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


.dseg
MountainMatrix:			.byte MAP_CELLS
Cur_CoverageMask:		.byte MAP_CELLS
Pre_CoverageMask:		.byte MAP_CELLS  ; 0 = unseen, 1 = covered
Max_CoverageMask:		.byte MAP_CELLS
PathLength:				.byte 1          ; number of stored observation points
ObservationPoints:		.byte PATH_BUF_BYTES
Visted:					.byte MAX_OBS_POINTS
ObservationPath:       .byte PATH_BUF_BYTES
Visibility:            .byte 1

.cseg
load_array_from_program m, MountainMatrix, MAP_CELLS
load_array_from_program zeros, Cur_CoverageMask, MAP_CELLS
load_array_from_program zeros, Pre_CoverageMask, MAP_CELLS
ldi xh, high(Visibility)
ldi xl, low(Visibility)
ldi r16, 7
st x, r16
ldi r16, 0
mov r2, r16
mov r3, r16
rcall Search
load_array_from_data Pre_CoverageMask, Cur_CoverageMask, MAP_CELLS
rcall Greedy_search
rcall Path_generation

end:
	rjmp end

; void Path_generation()
; Description: Path_generation
; r2 = last_x, r3 = last_y, r4 = cur_x, r5 = cur_y, r6 = des_x, r7 = des_y, r8 = PathLength, r9 = counter, r18 = i, r19 = l, r20 = min_l, r21 = min_index, r22, r23 = temp
Path_generation:
	push xh
	push xl
	push r2
	push r3
	push r4
	push r5
	push r6
	push r7
	push r8
	push r9
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	in r23, SREG
	push r23
	
	ldi xh, high(PathLength)
	ldi xl, low(PathLength)
	ld r8, x

	; Visted = [0 for _ in range(len(points))]
	load_array_from_program zeros, Visted, MAP_CELLS
	; Visted[0] = 1
	clr r22
	array_i Visted, r22
	ldi r22, 1
	st x, r22

	ldi xh, high(ObservationPoints)
	ldi xl, low(ObservationPoints)
	; last_x = int(points[0][0])
	ld r2, x+
	; last_y = int(points[0][1])
	ld r3, x+
	ld r22, x
	; order = [points[0]]
	ldi xh, high(ObservationPath)
	ldi xl, low(ObservationPath)
	st x+, r2
	st x+, r3
	st x, r22
	; counter = 1
	ldi r22, 1
	mov r9, r22
	Path_generation_while:
	; while True
		;min_l = 99
		ldi r20, 99
		;min_index = 0
		ldi r21, 0
		clr r18
		path_i_for_loop:
		; for i in range(len(points)):
			cp r18, r8
			in r22, SREG
			sbrc r22, 1
			jmp path_i_for_loop_end
			; l = 0
			clr r19
			if_Visted_eq_0:
			; if Visted[i] == 0:
				array_i Visted, r18
				ld r22, x
				cpi r22, 0
				in r22, SREG
				sbrs r22, 1
				jmp if_Visted_eq_0_end
				; cur_x = int(last_x)
				mov r4, r2
				; cur_y = int(last_y)
				mov r5, r3
				; des_x = int(points[i][0])
                ; des_y = int(points[i][1])
				ldi xh, high(ObservationPoints)
				ldi xl, low(ObservationPoints)
				ldi r22, OBS_POINT_STRIDE
				mul r18, r22
				mov r22, r0
				add xl, r22
				ldi r22, 0
				adc xh, r22 
				ld r6, x+
				ld r7, x
				light_while2:
				;while cur_x != des_x or cur_y != des_y:
					cp r6, r4
					brne light_while_body2
					cp r7, r5
					brne light_while_body2
					jmp light_while_end2
				light_while_body2:
					if_cur_x_g_des_x2:
					; if cur_x > des_x:
						cp r6, r4
						brsh if_cur_x_g_des_x_end2
						;cur_x = cur_x - 1
						dec r4
						if_cur_y_des_y_12:
						; if cur_y > des_y:
							cp r7, r5
							brsh if_cur_y_des_y_1_elif2
							;cur_y = cur_y - 1
							dec r5
							rjmp if_cur_y_des_y_1_end2
						if_cur_y_des_y_1_elif2:
						; elif cur_y < des_y:
							cp r5, r7
							brsh if_cur_y_des_y_1_end2
							; cur_y = cur_y + 1
							inc r5
						if_cur_y_des_y_1_end2:
						jmp light_if_end2
					if_cur_x_g_des_x_end2:

					if_cur_x_lo_des_x2:
					; elif cur_x < des_x:
						cp r4, r6
						brsh if_cur_x_lo_des_x_end2
						;cur_x = cur_x + 1
						inc r4
						if_cur_y_des_y_22:
						; if cur_y > des_y:
							cp r7, r5
							brsh if_cur_y_des_y_2_elif2
							;cur_y = cur_y - 1
							dec r5
							rjmp if_cur_y_des_y_2_end2
						if_cur_y_des_y_2_elif2:
						; elif cur_y < des_y:
							cp r5, r7
							brsh if_cur_y_des_y_2_end2
							; cur_y = cur_y + 1
							inc r5
						if_cur_y_des_y_2_end2:
						jmp light_if_end2
					if_cur_x_lo_des_x_end2:

					if_cur_x_des_x_else2:
						if_cur_y_des_y_32:
							; if cur_y > des_y:
							cp r7, r5
							brsh if_cur_y_des_y_3_elif2
							;cur_y = cur_y - 1
							dec r5
							rjmp if_cur_y_des_y_3_end2
						if_cur_y_des_y_3_elif2:
						; elif cur_y < des_y:
							cp r5, r7
							brsh if_cur_y_des_y_3_end2
							; cur_y = cur_y + 1
							inc r5
						if_cur_y_des_y_3_end2:
					if_cur_x_des_x_else_end2:
					light_if_end2:
					;l = l + 1
					inc r19
					jmp light_while2
				light_while_end2:
			
				if_min_l_g_l:
				;if min_l > l:
					cp r19, r20
					brsh if_min_l_g_l_end
					; min_l = int(l)
					mov r20, r19
					; min_index = i
					mov r21, r18
				if_min_l_g_l_end:
			if_Visted_eq_0_end:
			inc r18
			jmp path_i_for_loop
		path_i_for_loop_end:

		if_min_l_eq_99:
		;if min_l == 99:
			cpi r20, 99
			brne if_min_l_eq_99_end
			jmp Path_generation_while_end
		if_min_l_eq_99_end:
		; last_x = point[min_index][0]
        ; last_y = point[min_index][1]
		ldi xh, high(ObservationPoints)
		ldi xl, low(ObservationPoints)
		ldi r22, OBS_POINT_STRIDE
		mul r21, r22
		mov r22, r0
		add xl, r22
		ldi r22, 0
		adc xh, r22
		ld r2, x+
		ld r3, x+
		ld r23, x

		; order.append(point[min_index])
		ldi xh, high(ObservationPath)
		ldi xl, low(ObservationPath)
		ldi r22, OBS_POINT_STRIDE 
		mul r9, r22
		mov r22, r0
		add xl, r22
		ldi r22, 0
		adc xh, r22
		st x+, r2
		st x+, r3
		st x, r23
		; Visted[min_index] = 1
		array_i Visted, r21
		ldi r22, 1
		st x, r22
		; counter += 1
		inc r9
		jmp Path_generation_while
	Path_generation_while_end:

	pop r23
	out SREG, r23
	pop r23
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r9
	pop r8
	pop r7
	pop r6
	pop r5
	pop r4
	pop r3
	pop r2
	pop xl
	pop xh
	ret

; void greedy_search()
; Description: Greedy search
; r2 = org_x, r3 = org_y, r4 = max_x, r5 = max_y, r6 = counter, r18 = max_diff, r19 = diff, r20 = x, r21 = y, r22, r23 = temp
Greedy_search:
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
	in r23, SREG
	push r23

	; counter = 1
	clr r6
	inc r6
	
	greedy_while:
	; while True:
		; max_diff = 0
		clr r18
		clr r21
		greedy_y_for_loop:
		; for y in range(len(cur_cover)):
			cpi r21, MAP_SIZE
			in r22, SREG
			sbrc r22, 1
			jmp greedy_y_for_loop_end
			clr r20
			greedy_x_for_loop:
			;for x in range(len(cur_cover[y])):
				cpi r20, MAP_SIZE
				in r22, SREG
				sbrc r22, 1
				jmp greedy_x_for_loop_end
				if_pre_cov_eq_0:
				; if pre_cover[y][x] == 0:
					array_i_j Pre_CoverageMask, MAP_SIZE, r20, r21
					ld r22, x
					cpi r22, 0
					in r22, SREG
					sbrs r22, 1
					jmp if_pre_cov_eq_0_end
					; cur_cover = [[pre_cover[j][i] for i in range(len(mountain))] for j in range(len(mountain))]
					load_array_from_data Cur_CoverageMask, Pre_CoverageMask, MAP_CELLS
					; search((x,y))
					mov r2, r20
					mov r3, r21
					rcall search
					; diff = count_diff(cur_cover, pre_cover)
					count_diff Cur_CoverageMask, Pre_CoverageMask, MAP_CELLS, r19
					if_diff_g_max_diff:
					; if diff > max_diff:
						cp r18, r19
						brsh if_diff_g_max_diff_end
						mov r18, r19 ; max_diff = int(diff)
						mov r4, r20	; max_point = (x, y)
						mov r5, r21
						; max_map = [[cur_cover[j][i] for i in range(len(mountain))] for j in range(len(mountain))]
						load_array_from_data Max_CoverageMask, Cur_CoverageMask, MAP_CELLS
					if_diff_g_max_diff_end:
				if_pre_cov_eq_0_end:
				inc r20
				jmp greedy_x_for_loop
			greedy_x_for_loop_end:
			inc r21
			jmp greedy_y_for_loop
		greedy_y_for_loop_end:
		if_max_diff_eq_0:
			cpi r18, 0
			in r22, SREG
			sbrc r22, 1
			jmp greedy_while_end

		;point.append(max_point)
		array_i_j MountainMatrix, MAP_SIZE, r4, r5
		ld r23, x
		ldi	xh, high(ObservationPoints)
		ldi xl, low(ObservationPoints)
		ldi r22, 3
		mul r6, r22
		add xl, r0
		ldi r22, 0
		adc xh, r22
		st x+, r4
		st x+, r5
		st x+, r23
		; pre_cover = [[max_map[j][i] for i in range(len(mountain))] for j in range(len(mountain))]
		load_array_from_data Pre_CoverageMask, Max_CoverageMask, MAP_CELLS

		inc r6
		jmp greedy_while
	greedy_while_end:

	ldi xh, high(PathLength)
	ldi xl, low(PathLength)
	st x, r6

	pop r23
	out SREG, r23
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
	ret

; void search(org_x, org_y)
; Description: search all light path
; r2 = org_x, r3 = org_y, r4 = v, r18 = x, r19 = y, r20 = des_x, r21 = des_y, r22 = temp
Search:
	push xh
	push xl
	push r2
	push r3
	push r4
	push r18
	push r19
	push r20
	push r21
	push r22
	in r22, SREG
	push r22

	;cur_cover[start[1]][start[0]] = 1
	array_i_j Cur_CoverageMask, MAP_SIZE, r2, r3
	ldi r22, 1
	st x, r22

	ldi xh, high(Visibility)
	ldi xl, low(Visibility)
	ld r4, x

	mov r19, r4
	neg r19
	search_y_for_loop:
	; for y in range(-v, v + 1):
		cp r4, r19
		brlt search_y_for_loop_end
		mov r18, r4
		neg r18
		search_x_for_loop:
		; for x in range(-v, v + 1):
			cp r4, r18
			brlt search_x_for_loop_end
			; des_x = start[0] + x
			mov r20, r2
			add r20, r18
			; des_y = start[1] + y
			mov r21, r3
			add r21, r19
			if_not_in_array:
			; if 0 <= des_x < len(mountain) and 0 <= des_y < len(mountain):
				cpi r20, 0
				brlt if_not_in_array_end
				cpi r20, MAP_SIZE
				brge if_not_in_array_end
				cpi r21, 0
				brlt if_not_in_array_end
				cpi r21, MAP_SIZE
				brge if_not_in_array_end
				if_cur_cover_eq_0:
				; if cur_cover[des_y][des_x] == 0:
					array_i_j Cur_CoverageMask, MAP_SIZE, r20, r21
					ld r22, x
					cpi r22, 0
					brne if_cur_cover_eq_0_end
					mov r6, r20
					mov r7, r21
					rcall Light_path
				if_cur_cover_eq_0_end:
			if_not_in_array_end:
			inc r18
			jmp search_x_for_loop
		search_x_for_loop_end:
		inc r19
		jmp search_y_for_loop
	search_y_for_loop_end:

	pop r22
	out SREG, r22
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r4
	pop r3
	pop r2
	pop xl
	pop xh
	ret


; void Light_path(org_x, org_y, des_x, des_y)
; Description: Scan like light
; r2 = org_x, r3 = org_y, r4 = cur_x, r5 = cur_y, r6 = des_x, r7 = des_y, r8 = l, r18 = cur_grad_nu, r19 = cur_grad_de, r20 = max_grad_nu, r21 = max_grad_de , r22, r23, r24, r25= temp
Light_path:
	push xh
	push xl
	push r2
	push r3
	push r4
	push r5
	push r6
	push r7
	push r8
	push r18
	push r19
	push r20
	push r21
	push r22
	in r22, SREG
	push r22
	push r23
	push r24
	push r25

	; cur_x = int(org_x)
	mov r4, r2
	; cur_y = int(org_y)
	mov r5, r3
	; max_grad_nu = -8
	ldi r22, -8
	mov r20, r22
    ; max_grad_de = 1
	ldi r22, 1
	mov r21, r22
	; l = np.int8(0)
	clr r8
	light_while:
	;while cur_x != des_x or cur_y != des_y:
		cp r6, r4
		brne light_while_body
		cp r7, r5
		brne light_while_body
		jmp light_while_end
	light_while_body:
		if_cur_x_g_des_x:
		; if cur_x > des_x:
			cp r6, r4
			brsh if_cur_x_g_des_x_end
			;cur_x = cur_x - 1
			dec r4
			if_cur_y_des_y_1:
			; if cur_y > des_y:
				cp r7, r5
				brsh if_cur_y_des_y_1_elif
				;cur_y = cur_y - 1
				dec r5
				rjmp if_cur_y_des_y_1_end
			if_cur_y_des_y_1_elif:
			; elif cur_y < des_y:
				cp r5, r7
				brsh if_cur_y_des_y_1_end
				; cur_y = cur_y + 1
				inc r5
			if_cur_y_des_y_1_end:
			jmp light_if_end
		if_cur_x_g_des_x_end:

		if_cur_x_lo_des_x:
		; elif cur_x < des_x:
			cp r4, r6
			brsh if_cur_x_lo_des_x_end
			;cur_x = cur_x + 1
			inc r4
			if_cur_y_des_y_2:
			; if cur_y > des_y:
				cp r7, r5
				brsh if_cur_y_des_y_2_elif
				;cur_y = cur_y - 1
				dec r5
				rjmp if_cur_y_des_y_2_end
			if_cur_y_des_y_2_elif:
			; elif cur_y < des_y:
				cp r5, r7
				brsh if_cur_y_des_y_2_end
				; cur_y = cur_y + 1
				inc r5
			if_cur_y_des_y_2_end:
			jmp light_if_end
		if_cur_x_lo_des_x_end:

		if_cur_x_des_x_else:
			if_cur_y_des_y_3:
				; if cur_y > des_y:
				cp r7, r5
				brsh if_cur_y_des_y_3_elif
				;cur_y = cur_y - 1
				dec r5
				rjmp if_cur_y_des_y_3_end
			if_cur_y_des_y_3_elif:
			; elif cur_y < des_y:
				cp r5, r7
				brsh if_cur_y_des_y_3_end
				; cur_y = cur_y + 1
				inc r5
			if_cur_y_des_y_3_end:
		if_cur_x_des_x_else_end:
		light_if_end:
		;l = l + 1
		inc r8

		; cur_grad_nu = np.int8(mountain[cur_y][cur_x] - mountain[org_y][org_x])
		array_i_j MountainMatrix, MAP_SIZE, r4, r5
		ld r18, x
		array_i_j MountainMatrix, MAP_SIZE, r2, r3
		ld r22, x
		sub r18, r22
		; cur_grad_de = np.int8(l)
		mov r19, r8

	if_cur_grad_great_max_grad:
		; if cur_grad_nu * max_grad_de >= cur_grad_de * max_grad_nu:
		muls r18, r21
		mov r22, r0
		muls r19, r20
		mov r23, r0
		cp r22, r23
		brlt if_cur_grad_great_max_grad_end
		; max_grad_nu = np.int8(cur_grad_nu)
		mov r20, r18
		; max_grad_de = np.int8(cur_grad_de)
		mov r21, r19
	if_cur_grad_great_max_grad_end:
	if_cur_grad_sh_max_grad:
		;if cur_grad_nu * max_grad_de >= max_grad_nu * cur_grad_de:
		muls r18, r21
		mov r22, r0
		muls r19, r20
		mov r23, r0
		cp r22, r23
		
		in r24, SREG
		in r25, SREG
		lsr r25
		eor r24, r25
		sbrc r24, 2
		jmp if_cur_grad_sh_max_grad_end
		if_visiable:
			; v**2 >= (org_x - cur_x)**2 + (org_y - cur_y)**2 + (mountain[org_y][org_x] - mountain[cur_y][cur_x])**2:
			array_i_j MountainMatrix, MAP_SIZE, r2, r3
			ld r22, x
			array_i_j MountainMatrix, MAP_SIZE, r4, r5
			ld r23, x
			sub r22, r23
			brpl light_positive_dz
			neg r22
			light_positive_dz:

			mov r23, r2
			mov r24, r4
			sub r23, r24
			brpl light_positive_dx
			neg r23
			light_positive_dx:

			mov r24, r3
			mov r25, r5
			sub r24, r25
			brpl light_positive_dy
			neg r24
			light_positive_dy:

			mul r22, r22
			mov r22, r0

			mul r23, r23
			mov r23, r0

			mul r24, r24
			mov r24, r0

			ldi xh, high(Visibility)
			ldi xl, low(Visibility)
			ld r25, x
			mul r25, r25
			mov r25, r0

			add r22, r23
			in r23, SREG
			sbrc r23, 3
			jmp if_cur_grad_sh_max_grad_end

			add r22, r24
			in r23, SREG
			sbrc r23, 3
			jmp if_cur_grad_sh_max_grad_end

			cp r25, r22
			in r23, SREG
			sbrc r23, 0
			jmp if_cur_grad_sh_max_grad_end

			; cur_cover[cur_y][cur_x] = 1
			array_i_j Cur_CoverageMask, MAP_SIZE, r4, r5
			ldi r23, 1
			st x, r23
	if_cur_grad_sh_max_grad_end:
		jmp light_while
	light_while_end:

	pop r25
	pop r24
	pop r23
	pop r22
	out SREG, r22
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r8
	pop r7
	pop r6
	pop r5
	pop r4
	pop r3
	pop r2
	pop xl
	pop xh
	ret


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
		0,1,2,0,0,0,0,0,0,0,0,0,0,0,0

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
