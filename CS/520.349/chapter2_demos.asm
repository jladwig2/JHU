        base !10
        pagewidth 127
;
;Stuff to reuse
;
	PTB 	EQU 	$0001
	PTD 	EQU 	$0003
	DDRB 	EQU	$0005
	DDRD	EQU	$0007
;
	org $FFFE
        fdb mainA               ;Reset Vector
	org $EC00		;Flash
;
;Turn on some LEDs
;
mainA:	mov #%11111100,DDRD	;bits 7-2 outputs for PTD
	mov #%11000010,DDRB	;bits 1-0 and output_strobe
	
	;now do the LED pattern (just alternate)
	mov #%10101000,PTD	;upper 6 bits alternate
	mov #%10000000,PTB	;lower 2 bits alternate (but bits 7 and 6 here)
	
	;output strobe (just pulse on off once)
	bset 1,PTB
	bclr 1,PTB
	halt
;
;
;
mainB:	mov #%00000000,DDRD	;clear for all inputs
	mov #%00000011,DDRB	;input and output strobe as outputs
	
	mov #%00000000,PTB	;set both strobes low (bits 0 and 1)
	
	