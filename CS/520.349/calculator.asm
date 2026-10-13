        base !10
        pagewidth 127
;
; Variables
;
BYTE_LENGTH EQU 20
;
        org $90                 ;ioJK3.asm uses RAM from $80-$8F
;
input1:	rmb BYTE_LENGTH
input2: rmb BYTE_LENGTH
sum:	rmb BYTE_LENGTH
active_buf:
	rmb 1			;0 for in1, 1 for in2, 2 for sum
display_ptr:
	rmb 1
;
        org $FFFE
        fdb start               ;Reset Vector
;
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

	;clear all RAM blocks we will use
	ldhx #$0090
init:	clr ,X
	aix #1
	cphx #{display_ptr + 1}
	blo init
	
	jsr display		;init display state
	
collect_input:
	jsr new_key		;wait for input
	
	;now we cover our branching cases
	;these should be self-explanatory
	cmp #$09
	bls handle_digit
	
	cmp #$0A
	beq key_slide_left
	
	cmp #$0B
	beq key_slide_right
	
	cmp #$0C
	beq key_backspace
	
	cmp #$0D
	beq key_clear
	
	cmp #$0E
	beq key_plus
	
	bra key_equals		;last case, no check needed
	
handle_digit:
	;check if we are at max digit capacity
	psha			;save raw digit
	
	;sum is completed, need to hit '+'
	lda active_buf
	cmp #2
	beq handle_digit_err
	
	;get the current input (1 or 2) base in H:X
	bsr get_active_base
	
	;bounds check that we have not filled input buffer
	lda {BYTE_LENGTH - 1},X
	and #$F0		;check upper nibble 
	bne handle_digit_err
	
	pula
	bsr shift_nibbles_left
	
	clr display_ptr		;window pinned to LSD if typing
	jsr display
	bra collect_input
	
handle_digit_err:
	pula
	bra invalid_op
	
key_slide_left:
	;check capacity
	lda display_ptr
	cmp #{2*BYTE_LENGTH - 8}
	bhs invalid_op
	
	;check nonzero above display_ptr + 8 before sliding
	bsr check_msd
	beq invalid_op
	
	inc display_ptr
	jsr display
	bra collect_input

key_slide_right:
	;check boundary
	lda display_ptr
	beq invalid_op
	
	dec display_ptr
	jsr display
	bra collect_input
	
key_backspace:
	;check completed sum (no backspace in this case)
	lda active_buf
	cmp #2
	beq invalid_op
	
	bsr get_active_base
	
	;check empty (doesn't make sense to delete empty)
	bsr check_empty
	beq invalid_op
	
	bsr shift_nibbles_right
	
	jsr display
	bra collect_input
	
key_clear:
	;get display_ptr to lsd and active buffer to all zeros
	bsr get_active_base
	lda #BYTE_LENGTH
	psha
clear_loop:
	clr ,X
	aix #1
	dec 1,sp
	bne clear_loop
	pula
	
	clr display_ptr
	jsr display
	bra collect_input
	
key_plus:
	;check not already on input2 or sum
	lda active_buf
	bne invalid_op
	
	mov #1,active_buf
	
	clr display_ptr
	jsr display
	bra collect_input
	
key_equals:
	lda active_buf
	cmp #1
	bne invalid_op		;should be completing input2
	
	jsr add
	mov #2,active_buf
	
	clr display_ptr
	jsr display
	bra collect_input
	
invalid_op:
	lda #20
	jsr tone
	bra collect_input
;
;
;
get_active_base:
	lda active_buf
	beq in1
	cmp #1
	beq in2
	ldhx #sum
	rts
in1:	ldhx #input1
	rts
in2:	ldhx #input2
	rts
;
; Helper to check if the current buffer is empty. Scans using index register
; across BYTE_LENGTH bytes. Sets Z=1 if all zeros, Z=0 if any digits present.
;
check_empty:
	pshx
	pshh
	bsr get_active_base
	lda #BYTE_LENGTH
	psha			;1,sp = loop counter
	clra
chk_loop:
	ora ,X
	aix #1
	dec 1,sp
	bne chk_loop
	pula
	pulh
	pulx
	tsta
	rts
	
; From this point on, all remaining subroutines are key to this program
;
; Add
;
add:	clc
	clrh
	clrx
add_loop:
	lda input1,X
	adc input2,X		;add with carry
	daa			;fix A so soum nibbles are correct
	sta sum,X
	
	aix #1
	cphx #BYTE_LENGTH
	blo add_loop
	rts
;
; Display
;
display:
	jsr eight_digits
	;incomplete: blank leading zeros
	rts
;
; Gets the ascii masks for the 7-seg display in each digit currently shown
; in the sliding window. Starts from lsd = display_ptr
;
eight_digits:
	lda display_ptr
	psha			;save this to be our digit ptr
	ldhx #digit0
eight_loop:
	lda 1,sp		;read the ptr from stack
	lsra
	bcs get_high_nibble
get_low_nibble:
	bsr read_byte
	and #$0F		;lower nibble
	bra store_digit
get_high_nibble:
	bsr read_byte
	nsa			;swap high and low nibble
	and #$0F		;makes this mask work again
store_digit:
	jsr ascii_7seg
	sta ,X
	aix #1
	inc 1,sp
	cphx #{digit7 + 1}
	blo eight_loop
	
	pula
	rts

	
	
	
	