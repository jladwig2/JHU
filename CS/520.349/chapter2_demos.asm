        base !10
        pagewidth 127
;
;Stuff to reuse
;
PTB	EQU $0001
PTD EQU $0003
DDRB EQU $0005
DDRD EQU $0007
;
	   	org $FFFE
        fdb mainA               ;Reset Vector
			org $EC00				;Flash
;
;Turn on some LEDs
;
mainA:	mov #%11111100,DDRD		;bits 7-2 outputs for PTD
			mov #%11000010,DDRB		;bits 1-0 and output_strobe
	
			;now do the LED pattern (just alternate)
			mov #%10101000,PTD		;upper 6 bits alternate
			mov #%10000000,PTB		;lower 2 bits alternate (but bits 7 and 6 here)
	
			;output strobe (just pulse on off once)
			bset 1,PTB
			bclr 1,PTB
			stop
;
;Control 6 LEDs with column 1 of the keypad and the upper two switches.
;
mainB:	mov #%11111100,DDRD	  	;drive bus and scan_sink latch
			mov #%11010110,DDRB
		 
		 	clr PTD					;b7 through b2 = 0
        bclr 7,PTB				;b1 = 0
        bset 6,PTB				;latch col1

			bset 2,PTB				;Scan_Sink to 1
        bclr 2,PTB              ;and now 0
	
			bset 4,DDRB   			;make /Input_Strobe an output pin
        bset 1,DDRB    			;make Output_Strobe an output pin
	
loopB:	mov #%11111100,DDRD	    ;setup to latch col1 for input
			mov #%11010110,DDRB
	
			clr PTD
	
			bclr 7,PTB  
        bset 6,PTB
	
			bset 2,PTB   
        bclr 2,PTB
	
			clr DDRD  	 	   		;start of actual ops (after setup)

			bclr 4,PTB
			bclr 1,PTB
	
			lda PTD
	
			bset 4,PTB
			bclr 1,PTB
	
			mov #%11111100,DDRD 	;PTD bits 7-2 are now output drivers
	
			sta PTD
	
			bset 1,PTB
	
			bclr 1,PTB				;closes latch, freezing LED states
        bset 4,PTB   
	
			bra loopB
;
;Sound a tone
;
mainC:
	
	