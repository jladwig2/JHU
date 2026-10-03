;Data List Code
;520.349 Microprocessor Lab I
;Jacob Ladwig
;09/22/26
;
;Scans a list of bytes starting at $0090 until it reads $FF. Writes the number of bytes found to
;$0080, the minimum value found to $0081, the maximum value found to $0082, the 16-bit sum of the
;data values to $0083 (high) and $0084 (low), and the mean rounded to the nearest integer to $0085.
;
	BASE !10
	PAGEWIDTH 127
	ORG $80		;Start of RAM
;
;Variable Assignment -
;
COUNT:  RMB 1
MIN:    RMB 1
MAX:    RMB 1
SUM:    RMB 2
MEAN:   RMB 1
TEMP:   RMB 2		;Second byte is not for anything, this is used to get just the contents of H
;
	ORG $90
SCAN:   RMB $70		;$FF - $90 + $01 = !112 = $70 addresses inclusive
;
	ORG $FFFE	;Reset Vector
	FDB START
;
	ORG $EC00	;Start of flash ROM
;
START:
	LDX #COUNT	;Start of our variables to clear
	CLRH		;All of our memory will work in X, set H to 0
CLEAR:
	CLR ,X		;Clear byte index reg points to
	AIX #1		;Move index reg to the next byte
	CPX #SCAN	;Stop if we reach scanner range (MAIN starts at SCAN)
	BNE CLEAR	;Otherwise continue
	LDA #$FF	;Clear loop done, need min to be #FF to work correctly
	STA MIN
MAIN:   LDA ,X		;Move value into A for comparison
	CMP #$FF
	BEQ READ_DONE	;If we get $FF, stop reading immediately
	INC COUNT	;Otherwise, increment count
	CMP MIN
	BLO UPDATE_MIN	;If (unsigned) smaller than current min, update min
CHECK_MAX:
	CMP MAX
	BHI UPDATE_MAX
	BRA UPDATE_SUM 	;If we don't need to update max, go to sum
UPDATE_MIN:
	STA MIN
	BRA CHECK_MAX	;Always check if we need to update max as well
			;Note this only realy applies for first value
UPDATE_MAX:
	STA MAX		;Update max and continue to update sum
UPDATE_SUM:
	ADD SUM+1	;Add to lower byte of SUM
	STA SUM+1
	BCC NEW_VALUE	;If no carry we are done with this value
	INC SUM		;If carry, increment the upper byte of SUM
NEW_VALUE:
	AIX #1		;Move to next byte
	BRA MAIN
READ_DONE:
	LDA COUNT	;Read count into A
	BNE CALC_MEAN	;If count is zero, do nothing
	CLR MIN		;If count is zero, MIN is $FF so we need to clear it
	BRA END
CALC_MEAN:
	LDHX SUM	;If not, compute mean (high byte)
	LDA SUM+1	;Low byte
	LDX COUNT	;Count
	DIV		;Now A is quotient and H is remainder
	STHX TEMP	;Put remainder into TEMP (X in trash after TEMP)
	ASL TEMP	;Double remainder
	BCS ROUND_UP
	LDX TEMP	;If we didn't branch, check if TEMP GEQ COUNT
	CPX COUNT
	BHS ROUND_UP
	STA MEAN	;If we didn't branch by now, round down (do nothing)
	BRA END
ROUND_UP:		;Here we assume quotient to round is in A
	INCA		;Round up
	STA MEAN	;Save final result
END:	STOP		;This exists because I don't like writing STOP more
			;than once