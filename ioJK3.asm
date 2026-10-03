; Ports

PTB     EQU $0001       ;Port B data register
PTD     EQU $0003       ;Port D data register
DDRB    EQU $0005       ;Port B data direction register
DDRD    EQU $0007       ;Port D data direction register
CONFIG1:EQU $001f       ;Configuration register

        org $80         ;Ram

digit0: rmb 1           ;segment data for digit0 (rightmost)
digit1: rmb 1           ;segment data for digit1
digit2: rmb 1           ;segment data for digit2
digit3: rmb 1           ;segment data for digit3
digit4: rmb 1           ;segment data for digit4
digit5: rmb 1           ;segment data for digit5
digit6: rmb 1           ;segment data for digit6
digit7: rmb 1           ;segment data for digit7 (leftmost)
kbd0:   rmb 1           ;keyboard keys C 8 4 0 D 9 5 1
kbd1:   rmb 1           ;keyboard keys E A 6 2 F B 7 3
io_temp_a: rmb 1        ;io routine temporary variable
io_temp_b: rmb 1        ;io routine temporary variable
io_scan_digit: rmb 1    ;scan routine digit select

        org $EC00       ;start of flash memory

; bit   7  6      5             4         3       2          1        0
; PTB  b1 b0 Scan_Source /Input_Strobe Audio Scan_Sink Output_Strobe Com
; PTD  b7 b6     b5            b4        b3      b2          -        -
;
; Output A to b7 b6 b5 b4 b3 b2 b1 b0 (bus)
;
; Destroys A, io_temp_a
;
; Uses 57 cycles
;
output: clr ddrd
        sta ptd                 ;bits 1, 0 don't exist, no trouble
        mov #%00000100,ddrd     ;activating
        mov #%00001100,ddrd     ;all bits
        mov #%00011100,ddrd     ;simultaneously
        mov #%00111100,ddrd     ;causes glitches
        mov #%01111100,ddrd     ;on the 74HCT373's LE inputs
        mov #%11111100,ddrd     ;ptd bits 7-2 now output
        rora
        rora
        rora                    ;now bits 1 0 are in 7 6
        and #%11000000          ;and alone
        sta io_temp_a
        lda ptb                 ;get current values
        and #%00111111
        ora io_temp_a           ;put b1 b0
        sta ptb                 ;in ptb
        mov #%11111110,ddrb     ;ptb all output except for com bit
        rts

; Output A to LEDs
;
; Destroys A, io_temp_a
;
leds:   bsr output              ;A to bus
        bset 1,ptb              ;Output_Strobe to 1
        bclr 1,ptb              ; and now 0
        rts

; Output A to Segment Source Latch
;
; Destroys A, io_temp_a
;
; Uses 73 cycles
;
segment:bsr output              ;A to bus
        bset 5,ptb              ;Scan_Source to 1
        bclr 5,ptb              ; and now 0
        rts

; Output A to Digit Sink Latch
;
; Destroys A, io_temp_a
;
; Uses 73 cycles
;
digit:  bsr output              ;A to bus
        bset 2,ptb              ;Scan_Sink to 1
        bclr 2,ptb              ; and now 0
        rts

; Delay about 1200 microseconds
; so as to meet the 150 ma. spec for the display digit led's
;
; Uses 7202 cycles
; 6 MHz bus clock
;
; Destroys A
;
digit_delay:
        lda #200                ;36 cycles each loop
digit_delay_loop:
        bsr dd_rts              ;waste 8 cycles in two bytes
        bsr dd_rts              ;waste 8 cycles in two bytes
        bsr dd_rts              ;waste 8 cycles in two bytes
        bsr dd_rts              ;waste 8 cycles in two bytes
        deca                    ;1 cycle
        bne digit_delay_loop    ;3 cycles
dd_rts: rts

; Input 8 bits to A
;
; Destroys io_temp_a
;
; Uses 38 cycles
;
input:  clr ddrd                ;ptd now all input
        mov #%00111110,ddrb     ;ptb b1 b0 com input
        bclr 4,ptb              ;lower /Input_Strobe
                                ;assuming that ptb has already been inititalized
        lda ptd                 ;read b7 b6 b5 b4 b3 b2 x x
        and #%11111100
        sta io_temp_a
        lda ptb                 ;read bits b1 b0 in upper bits
        rola
        rola
        rola                    ;get them to b1 b0 positions
        and #%00000011          ;and alone
        ora io_temp_a           ;now have all bus bits in A
        bset 4,ptb              ;release (raise) /Input_Strobe
        rts

; Scan the 7-segment displays and the keyboard.
; Output Digit0-Digit7 to rightmost-leftmost display digits one time.
; Overwrite kbd0-kbd1 with 1=key depressed, 0=key released for:
;  kbd0: C 8 4 0 D 9 5 1
;  kbd1: E A 6 2 F B 7 3
; just gets one sample
;
; takes 2346 cycles + (8 * Digit Delay)
; or 10 ms.
;
; each time 25 + 146 + DD + 87
; 1: 67, 2: 75, 3: 67, 4: 73, 5-8: 0
;
; Destroys A, io_temp_a
;
scan:   pshx                            ;save X
        pshh                            ;save H
        mov #%00000001,io_scan_digit    ;start with digit 0
        ldhx #digit0
scan_loop:
        lda ,x                          ;retrieve segment code for this digit
        bsr segment                     ;send it to the segment latch
        lda io_scan_digit
        bsr digit                       ;activate one digit
        bsr digit_delay
        lda #%00001111                  ;now check if at digits 3-0
        bit io_scan_digit
        beq scan_next                   ;no, don't need to read keyboard
        bsr input                       ;read keyboard row
        and #%11110000                  ;just the row bits, please
        brclr 0,io_scan_digit,scan_loop1
                                        ;here for digit0, read column 1
        sta kbd0                        ;kbd column 1 now in upper kbd0
scan_loop1:
        brclr 1,io_scan_digit,scan_loop2
                                        ;here for digit1, read column 2
        nsa                             ;swap upper/lower nibbles
        ora kbd0                        ;or it in with column 1 data
        sta kbd0                        ;and update lower kbd0
scan_loop2:
        brclr 2,io_scan_digit,scan_loop3
                                        ;here for digit2, read column 3
        sta kbd1                        ;kbd column 3 now in upper kbd1
scan_loop3:
        brclr 3,io_scan_digit,scan_next
                                        ;here for digit3, read column 4
        nsa                             ;swap upper/lower nibbles
        ora kbd1                        ;or it in with column 3 data
        sta kbd1                        ;and update lower kbd1
scan_next:
        clra                            ;turn off
        bsr digit                       ;current sink
        incx                            ;advance digit pointer
        clc                             ;carry is about to go into bit 0
        rol io_scan_digit               ;advance to the next digit
        bcc scan_loop                   ;not done digit7 yet
        pulh                            ;restore h
        pulx                            ;restore X
        rts

; Convert ASCII input in A to best 7-seg display code in A
; if input not found in table, exit with 00 for blank display
;
; preserves all
;
; The search table consists of two-byte entries:
;  the first byte is the ASCII value to display or hex 00-0F;
;  the second byte is the 7-segment code which best displays it.
;  bit code = .GFEDCBA, where A-G are the segments
;  and .= right-hand decimal point.
;  Table ends with $80.
;
ascii_table:
        fcb 0,%00111111
        fcb 1,%00000110
        fcb 2,%01011011
        fcb 3,%01001111
        fcb 4,%01100110
        fcb 5,%01101101
        fcb 6,%01111101
        fcb 7,%00000111
        fcb 8,%01111111
        fcb 9,%01101111
        fcb $A,%01110111
        fcb $B,%01111100
        fcb $C,%00111001
        fcb $D,%01011110
        fcb $E,%01111001
        fcb $F,%01110001
        fcb '0',%00111111
        fcb '1',%00000110
        fcb '2',%01011011
        fcb '3',%01001111
        fcb '4',%01100110
        fcb '5',%01101101
        fcb '6',%01111101
        fcb '7',%00000111
        fcb '8',%01111111
        fcb '9',%01101111
        fcb 'A',%01110111
        fcb 'B',%01111100
        fcb 'C',%00111001
        fcb 'D',%01011110
        fcb 'E',%01111001
        fcb 'F',%01110001
        fcb 'G',%00111101
        fcb 'H',%01110110
        fcb 'I',%00000100
        fcb 'J',%00011110
        fcb 'K',%01110000
        fcb 'L',%00111000
        fcb 'M',%00100011
        fcb 'N',%01010100
        fcb 'O',%01011100
        fcb 'P',%01110011
        fcb 'Q',%01100011
        fcb 'R',%01010000
        fcb 'S',%01101101
        fcb 'T',%00110001
        fcb 'U',%00111110
        fcb 'V',%00011100
        fcb 'W',%01001100
        fcb 'X',%00110110
        fcb 'Y',%01101110
        fcb 'Z',%01011111
        fcb $80                   ;end marker

ascii_7seg:
        pshx                    ;save X
        pshh                    ;save H
        ldhx #ascii_table
ascii_7seg_search:
        tst ,x                  ;at table end?
        bpl ascii_7seg1         ;no
        clra                    ;0 to a for blank
        bra ascii_7seg_exit
ascii_7seg1:
        cmp ,x                  ;A matches table entry?
        beq ascii_7seg_found    ;yes!
        aix #2                  ;no match, advance to
                                ;next table entry
        bra ascii_7seg_search
ascii_7seg_found:
        lda 1,x                 ;read value from next address
ascii_7seg_exit:
        pulh                    ;restore H
        pulx                    ;restore X
        rts

; Get current keyboard depression to A
;  exit with FF for no depressions
;  otherwise with 00-0F
;
; If more than one key depressed returns with the rightmost of:
;  E A 6 2 F B 7 3 C 8 4 0 D 9 5 1
;
; Destroys io_temp_a, io_temp_b
;
keyboard:
        lda kbd0
        ora kbd1
        bne keyboard_depression
keyboard_nothing:
        lda #$ff                ;no depressions
        rts
keyboard_depression:
        mov #16,io_temp_b       ;bit counter
        mov #1,io_temp_a        ;1 key code
        lda kbd0                ;keys C 8 4 0 D 9 5 1
keyboard2:
        rora
        bcc keyboard3           ;not this key
        lda io_temp_a           ;have key, exit with its key code
        rts
keyboard3:
        psha
        lda io_temp_a           ;current key code
        add #4                  ;next row value
        sta io_temp_a           ;new key code
        dec io_temp_b           ;decrement bit counter
        lda io_temp_b
        cmp #12                 ;finished column1?
        bne keyboard5           ;no
        clr io_temp_a           ;0 key code
keyboard4:
        pula                    ;restore kbd bits being rotated
        bra keyboard2           ;handle next nibble
keyboard5:
        cmp #8                  ;finished column2?
        bne keyboard6           ;no
        pula                    ;get rid of old rotating bits
        lda kbd1                ;keys E A 6 2 F B 7 3
        psha                    ;replace it with the new set
        mov #3,io_temp_a        ;3 key code
        bra keyboard4
keyboard6:
        cmp #4                  ;finished column3?
        bne keyboard4           ;no
        mov #2,io_temp_a        ;2 key code
        bra keyboard4

; Scan display until key release and new depression
; return with new key in A (00-0F)
;
; Destroys io_temp_a, io_temp_b
;
new_key:jsr scan
        bsr keyboard
        bpl new_key             ;key still depressed, wait
new_key1:
        jsr scan
        bsr keyboard
        bmi new_key1            ;no key depressed, wait
        rts                     ;okay, return with new key in A

; Generate an 800 Hz. tone for A cycles
;
; Destroys A
;
tone:   bset 3,ptb              ;audio bit high
        bsr tone_delay
        bclr 3,ptb              ;audio bit low
        bsr tone_delay
        deca
        bne tone
        rts

tone_delay:
        psha                    ;6 MHz bus clock
        lda #187                ;20 cycles each loop, 1/1600 second
tone_delay_loop:
        bsr td_rts              ;waste 8 cycles in two bytes
        bsr td_rts              ;waste 8 cycles in two bytes
        deca                    ;1 cycle
        bne tone_delay_loop     ;3 cycles
        pula
td_rts: rts
