.include "m2560def.inc"

;---------------------------------
; Constant & register definitions
;---------------------------------
.equ loop_count = 0xFFFF

.def pattern1         = r10
.def pattern2         = r11
; .def pattern3      = r12
.def button_press_cnt = r19
.def iH               = r25
.def iL               = r24
.def countH           = r17
.def countL           = r16
.def temp             = r18

;---------------------------------
; smallDelay: 短延时
;---------------------------------
.macro smallDelay
    ldi  countL, low(loop_count)
    ldi  countH, high(loop_count)
    clr  iH
    clr  iL
loop1:
    cp   iL, countL
    cpc  iH, countH
    brsh done
    adiw iH:iL, 1
    nop
    rjmp loop1
done:
    nop
.endmacro

;---------------------------------
; largeDelay: 大延时
;---------------------------------
.macro largeDelay
    ldi  temp, 0x0F
loop2:
    smallDelay
    dec  temp
    brne loop2
.endmacro

;---------------------------------
; mydisplay: 按一次按钮才继续显示 pattern
;   调用例子：mydisplay pattern1
;---------------------------------
.macro mydisplay
pause:
    ; 等待 button_press_cnt 的 bit0 变成 1
    sbrs button_press_cnt, 0     ; 如果 bit0 ==1，跳过下一条
    rjmp pause                   ; 否则一直停在这里

    ; 用掉这一“次按键”：清 bit0
    cbr  button_press_cnt, 0x01  ; clear bit0

    ; 在 LED BAR 显示 pattern
    out  PORTC, @0
    largeDelay
.endmacro


;---------------------------------
; Interrupt vectors
;---------------------------------
.cseg
.org 0x0000
    jmp RESET

.org INT0addr               ; INT0 中断向量
    jmp EXT_INT0


;---------------------------------
; RESET: 初始化
;---------------------------------
RESET:
    ; 载入要显示的 pattern
    ldi  r16, 0xAA
    mov  pattern1, r16
    ldi  r16, 0xF0
    mov  pattern2, r16
    ; ldi  r16, 0xCC
    ; mov  pattern3, r16

    ; Port C 作为 LED BAR 输出
    ser  temp
    out  DDRC, temp          ; DDRC = 0xFF, 全部输出
    clr  temp
    out  PORTC, temp         ; 先全部关灯

    ; 设置 INT0 为“下降沿触发”
    ; ISC01=1, ISC00=0 → (2 << ISC00)
    ldi  temp, (2 << ISC00)
    sts  EICRA, temp

    ; 使能 INT0
    in   temp, EIMSK
    ori  temp, (1 << INT0)
    out  EIMSK, temp

    ; 清 button_press_cnt
    clr  button_press_cnt

    sei                      ; 全局开中断

    rjmp main


;---------------------------------
; 外部中断 INT0 服务程序
;---------------------------------
EXT_INT0:
    push temp                ; 保存通用寄存器
    in   temp, SREG          ; 保存状态寄存器
    push temp

    inc  button_press_cnt    ; 记录按键次数（只用它的 bit0）

    pop  temp
    out  SREG, temp
    pop  temp
    reti


;---------------------------------
; main program
;---------------------------------
main:
    mydisplay pattern1       ; 等你按一次按钮 → 显示 pattern1
    mydisplay pattern2       ; 再按一次按钮 → 显示 pattern2
    ; mydisplay pattern3

    rjmp main

