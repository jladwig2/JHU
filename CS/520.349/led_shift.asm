;LED Shift Register Code
;520.349 Microprocessor Lab I
;Jacob Ladwig
;10/13/26
;
;Reads the 4x4 digital matrix keypad for 0 or 1 to set the value to shift in,
;A or B to set the direction of the shift, and finally F depression to signal
;a shift. Will not shift again until F is fully released and depressed again. 
;Debounces with a ~20 ms delay on both edges (released to depressed and depressed
;back to released). Displays the current state of the shifted value on 8 LEDs.
;
        base !10
        pagewidth 127
;
;
; Variables 
;
        org $90                 ;ioJK3.asm uses RAM from $80-$8F
;
pattern:
	rmb 1						;current pattern on the LEDs
flags:  rmb 1					;flags for direction and value
temp:	rmb 1					;throwaway temp var
;
; Constants for flag bits
;
FLAG_SHIFT_LEFT	equ 0
FLAG_SHIFT_VAL	equ 1
FLAG_SHIFT_SET 	equ 2
FLAG_INNER	equ 3
;
        org $FFFE
        fdb start               ;Reset Vector
;
        include "ioJK3.asm"     ;provides input/output subroutines
;
; ioJK3.asm orgs its code to $EC00, the start of flash ROM
; Code here continues in memory at the end of the ioJK3.asm code
;
start:  clra
		jsr leds	  	 		;init to clear leds
		bset 0,config1          ;disable Computer Operating Properly watchdog timer
        rsp                     ;initialize stack pointer to $00FF
        clra
        jsr digit               ;turn off current sink
		clr pattern				;clear vars
		clr flags
		clr temp
;
;
wait_shift:
	;must ensure delay ALWAYS happens after scanning
	;delay first ensures we always delay after exiting inner loop
	jsr delay
	brset FLAG_SHIFT_SET,flags,shift
	jsr scan_matrix				;if shift flag 0 then scan, always sets
	bra wait_shift
shift:
	;perform the shift, don't loop this part
	brset FLAG_SHIFT_LEFT,flags,left
	lsr pattern
	brclr FLAG_SHIFT_VAL,flags,display
	bset 7,pattern
	bra display
left:
	lsl pattern
	brclr FLAG_SHIFT_VAL,flags,display
	bset 0,pattern
display:
	;update leds here
	lda pattern
	jsr leds
inner:
	;loop this part same as wait_shift
	jsr delay
	brclr FLAG_SHIFT_SET,flags,wait_shift
	jsr scan_matrix
	bra inner
;
;Delay about 20 ms
;At 6 Mhz we need to waste 120000 cycles
;This requires 16 bits to track cycle count, so we use H:X
;Takes no input and destroys H:X
;
delay:
	ldhx #15000		;3 cycles (3 + 14999 * 8 is about 120000)
delay_loop:
	aix #-1			;2 cycles
	cphx #0			;3 cycles
	bne delay_loop		;3 cycles when taken
	rts	
;
;Scan the digital matrix 
;Takes no input and destroys A, modifies flags
;
scan_matrix:
	;check 0 (col 0 row 0)
	lda #%00000001 		;col bit
	jsr digit		;set latch
	jsr digit_delay
	jsr input		;get output
	sta temp
	brclr 4,temp,check_1	;check row bit %00010000 branch if 0
	bclr FLAG_SHIFT_VAL,flags
check_1:
	;check 1 (col 1 row 0)
	lda #%00000010
	jsr digit
	jsr digit_delay
	jsr input
	sta temp
	brclr 4,temp,check_A
	bset FLAG_SHIFT_VAL,flags
check_A:
	;check A (col 2 row 2)
	lda #%00000100
	jsr digit
	jsr digit_delay
	jsr input
	sta temp
	brclr 6,temp,check_col3
	bset FLAG_SHIFT_LEFT,flags 
check_col3:
	;check col3 for B and F (col 3 rows 2 and 3)
	lda #%00001000
	jsr digit
	jsr digit_delay
	jsr input
	sta temp
	brclr 6,temp,check_F
	bclr FLAG_SHIFT_LEFT,flags
check_F:
	;check F (already have row 6 bit in temp)
	brclr 7,temp,released
	bset FLAG_SHIFT_SET,flags
	bra return
released:
	bclr FLAG_SHIFT_SET,flags
return:
	clra
	jsr digit		;unlatch
	rts