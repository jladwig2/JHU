;7 Segment LED Display Scroller
;520.349 Microprocessor Lab I
;Jacob Ladwig
;10/13/26
;
;Scrolls the 7-segment display, putting each new key entered in the rightmost
;display. Waits for input (and full depress-release) before accepting a new
;key entry.
;
        base !10
        pagewidth 127

;
; Put your variables here
;
        org $90                 ;ioJK3.asm uses RAM from $80-$8F

; We don't need anything other than digit0-7 in ioJK3

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
	clr digit0		;clear the display
	clr digit1
	clr digit2
	clr digit3
	clr digit4
	clr digit5
	clr digit6
	clr digit7
	
scroll_loop:
	jsr new_key
	jsr ascii_7seg		
	psha			;save A contents
	lda #20			
	jsr tone		;25 ms 800 Hz tone
	pula
	jsr scroll_left
	bra scroll_loop
;
; Scroll the 7 segment display to the left and enter the accumulator character
; code into digit0.
;
scroll_left:
	mov digit6,digit7
	mov digit5,digit6
	mov digit4,digit5
	mov digit3,digit4
	mov digit2,digit3
	mov digit1,digit2
	mov digit0,digit1
	sta digit0		;new character to display
	rts
