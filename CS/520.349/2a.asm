        base !10
        pagewidth 127

;
; Put your variables here
;
        org $90                 ;ioJK3.asm uses RAM from $80-$8F

var1:   rmb 1
var2:   rmb 1

        org $FFFE
        fdb start               ;Reset Vector

        include "ioJK3.asm"     ;provides input/output subroutines
;
; ioJK3.asm orgs its code to $EC00, the start of flash ROM
; Code here continues in memory at the end of the ioJK3.asm code
;

start:  mov #%00010000,ptb      ;all low except for /Input_Strobe
        mov #%00111110,ddrb     ;b1, b0, com inputs, others output
                                ;ptd is already in input mode from reset, no need to alter
        bset 0,config1          ;disable Computer Operating Properly watchdog timer
        rsp                     ;initialize stack pointer to $00FF
        clra
        jsr digit               ;turn off current sink

;
;Put your code here
;
loop:   jsr input               ;sample code
        jsr leds                ;to get
        bra loop                ;you started
