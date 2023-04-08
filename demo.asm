; ################################################################################################           
; #                              REAL-TIME DIGITAL AUDIO FILTER - 2021                           #
; ################################################################################################

; ################################################################################################
; #                         Source file:      demo.asm                                           #
; #                         Code version:     v2.1                                               #
; #                         Revision date:    01/02/21 - 06.37pm                                 #
; #                         Programmer:       Janitha Gamage                                     #
; #                         Contact:          djg.gamage@gmail.com                               #
; ################################################################################################

; ################################################################################################
; # This project contains a real-time audio project solution based on the PIC16F877A to perform  #
; #                          all-pass, high-pass and low-pass filtering                          #
; ################################################################################################

; #############			    3 filters:           	                                 ###########
; #############             Straight through (All-pass) : mode 2		             ###########
; #############             Differencer (High-pass)     : mode 1                     ###########
; #############             Averager (Low-pass)         : mode 0                     ###########
; #############                                                                      ###########
; #############             Number of analogue inputs assumed = 1                    ###########
; #############             Electronic push-button switch assumed                    ###########
; #############             Fosc=8MHz, Fsample=8KHz, Ts=125uS                        ###########
; #############                                                                      ###########
; #############   ADC set-up:                                                        ###########
; #############   Fconv=8MHz/32 ADCS1,ADCS0= 1,0                                     ###########
; #############   Analogue input channel = RA2 CHS1, CHS0 = 1,0                      ###########
; #############   External 4.1 Volt Vref used for ADC; ADCON1= 00000001              ###########
; #############   Sampling rate determined via TOIF interrupt flag                   ###########
; #############   Input signal range 1V-->4.1V or 85-->255 in binary                 ###########
; #############                                                                      ###########
; #############   Internal arithmetic: 8-bit saturation logic                        ###########
; #############                                                                      ###########
; #############   DAC driver routine:                                                ###########
; #############   Outputs filterd result serially                                    ###########
; #############   Assumes that the 12-bit DAC-8512 will be used                      ###########
; #############                                                                      ###########
; #############   Mode: on/off indication:                                           ###########
; #############   Assumes that LEDs which operate effectively at 15mA used           ###########
; #############                                                                      ###########
; #############   RB1 = sw_l (mode switch),                                          ###########
; #############   RB2 = DAC clock, RB3 = SDI DAC pin; RB4 = DAC load signal          ###########
; #############   RB5 = Straight through mode indicator, RB6 = Differencer indicator ###########
; #############   RB7 = Averager indicator                                           ###########

; Assembly source line config statements
#include "p16f877a.inc"

; CONFIG
; PIC16F877A configuration bit settings
 __CONFIG _FOSC_HS & _WDTE_OFF & _PWRTE_OFF & _BOREN_OFF & _LVP_OFF & _CPD_OFF & _WRT_OFF & _CP_OFF
 
; ################################################################################################
	    ; Tell assembler to assemble for PIC16f877A 
		LIST p=16f877A				

	    ; Order of information:
	    ; Labels , opcodes, operands, comments
	    TIMER	    equ		0x01 	; Declaration of 8-bit TIMER location
	    STATUS	    equ		0x03 	; Declare name for STATUS register
	    PORTA	    equ		0x05 	; Declaration of PORTA location
	    PORTB	    equ		0x06 	; Digital input port
	    ADCON0	    equ		0x1F
	    ADRESH	    equ		0x1E
	    INTCON	    equ		0x0B 	; Interrupt control
	    OPTION_REG	equ		0x81 	; Timer control register
	    TRISA	    equ		0x85 	; Direction control register for PORT A
	    TRISB	    equ		0x86 	; Direction register for PORT B 
	    ADCON1	    equ		0x1F 	; Digital /analogue control for port A
	    FSR		    equ		0x04 	; Set address for indirect address register
	    INDF	    equ		0x00
	    RP0		    equ		5 		; Page bit
	    RB0		    equ		0		; RB0; bit 0 of port b
	    RB1		    equ		1 		; RB1; bit 1 of port b
	    RB2		    equ		2		; RB2; bit 2 of port b
	    RB3		    equ		3		; RB3; bit 3 of port b
	    RB4		    equ		4		; RB4; bit 4 of port b
	    RB5		    equ		5 		; RB5; bit 5 of port b
	    RB6		    equ		6		; RB6; bit 6 of port b
	    RB7		    equ		7		; RB7; bit 7 of port b	
	    Z		    equ		2 		; Zero bit
	    C		    equ		0 		; Carry bit
	    GIE		    equ		7 		; Global enable bit
	    TMR0IE	    equ		5 		; TIMER overflow enable bit
	    TMR0IF	    equ		2 		; TIMER overflow flag
	    GO		    equ		2 		; Start sampling bit
	    ADON	    equ		0 		; Turn on ADC bit
	    ADIE	    equ	    6 		; A-D interrupt enable bit
	    sign	    equ		7 		; Sign bit; MSB of input data 

; ################################################################################################
; External functions:
extern define_ports					; Define ports
extern init_lcd	    				; Initialise LCD
extern set_addr		    			; Set DDRAM Adress for 2nd row on LCD1
extern send_char					; Send character to LCD
extern clr_disp						; Clear LCD
global de_bounce					; Debounce
 
; ################################################################################################
; RAM location:
	    x_k		    equ		0x55 	; 0x0c
	    y_k		    equ		0x56 	; 0x0e
	    y2_k	    equ		0x57 	; 0x0d
	    x_k_1	    equ		0x35 	; 0x15
 
; ################################################################################################
; Counters for debouncing routine:
	    count	    equ		0x22 	; Counter variable - ORIGINAL VALUE = 0x0f
	    nest_count	equ		0x23 	; Counter variable for nested loop - ORIGINAL VALUE = 0x10
	    temp	    equ		0x25

; ################################################################################################
; Spurious variables:
 
	bit_count	    equ		0x20 	; Bit count for DAC driver routine - ORIGINAL VALUE = 0x17
	filter		    equ		0x21 	; Filter choice storage - ORIGINAL VALUE = 0x17

	; Set-up reset vector
	org 			0
	goto 			main_start

	; Set-up interrupt vector
	org 			4
	goto 			isr_start
	
; ################################################################################################
; #                                      Sub-routine definitions                                 #
; ################################################################################################


; ###################################################################
; #                      Display HIGH-PASS on LCD                   #
; ###################################################################

hp_disp
	call		clr_disp			; Clear display
	movlw   	b'01001000' 		; Print H
	call    	send_char
	movlw   	b'01001001' 		; Print I
	call    	send_char
	movlw   	b'01000111' 		; Print G
	call    	send_char
	movlw   	b'01001000' 		; Print H
	call    	send_char
	movlw   	b'10110000' 		; Print -
	call    	send_char
	movlw   	b'01010000' 		; Print P
	call    	send_char
	movlw   	b'01000001' 		; Print A
	call    	send_char
	movlw   	b'01010011' 		; Print S
	call    	send_char
	call    	set_addr
	movlw   	b'01010011' 		; Print S
	call    	send_char
	movlw   	b'00010000' 		; Print SPACE
	call    	send_char
	movlw   	b'01001101' 		; Print M
	call    	send_char
	movlw   	b'01000100' 		; Print D
	call    	send_char
	movlw   	b'01000101' 		; Print E
	call    	send_char
	movlw   	b'00010000' 		; Print SPACE
	call    	send_char
	return

; ###################################################################
; #                      Display LOW-PASS on LCD                    #
; ###################################################################
	
lp_disp	  
	call		clr_disp			; Clear display
	movlw   	b'01001100' 		; Print L
	call    	send_char
	movlw   	b'01001111' 		; Print O
	call    	send_char
	movlw   	b'01010111' 		; Print W
	call    	send_char
	movlw   	b'10110000'		 	; Print -
	call    	send_char
	movlw   	b'01010000' 		; Print P
	call    	send_char
	movlw   	b'01000001' 		; Print A
	call    	send_char
	movlw   	b'01010011' 		; Print S
	call    	send_char
	movlw   	b'01010011' 		; Print S
	call    	send_char
	call    	set_addr
	movlw   	b'00010000' 		; Print SPACE
	call    	send_char
	movlw   	b'01001101' 		; Print M
	call    	send_char
	movlw   	b'01001111' 		; Print O
	call    	send_char
	movlw   	b'01000100' 		; Print D
	call    	send_char
	movlw   	b'01000101' 		; Print E
	call    	send_char
	movlw   	b'00010000' 		; Print SPACE
	call    	send_char
	movlw   	b'00010000' 		; Print SPACE
	call    	send_char
	movlw   	b'00010000' 		; Print SPACE
	call    	send_char
	return

; ###################################################################
; #                      Display ALL-PASS on LCD                    #
; ###################################################################
	
ap_disp	
	call		clr_disp			; Clear display
	movlw   	b'01000001' 		; Print A
	call    	send_char
	movlw   	b'01001100'			; Print L
	call    	send_char
	movlw   	b'01001100' 		; Print L
	call    	send_char
	movlw   	b'10110000' 		; Print -
	call    	send_char
	movlw   	b'01010000' 		; Print P
	call    	send_char
	movlw   	b'01000001' 		; Print A
	call    	send_char
	movlw   	b'01010011' 		; Print S
	call    	send_char
	movlw   	b'01010011' 		; Print S
	call    	send_char
	call    	set_addr
	movlw   	b'00010000' 		; Print SPACE
	call    	send_char
	movlw   	b'01001101'			; Print M
	call    	send_char
	movlw   	b'01001111' 		; Print O
	call    	send_char
	movlw   	b'01000100'		 	; Print D
	call    	send_char
	movlw   	b'01000101' 		; Print E
	call    	send_char
	movlw   	b'00010000' 		; Print SPACE
	call    	send_char
	movlw   	b'00010000' 		; Print SPACE
	call    	send_char
	movlw   	b'00010000' 		; Print SPACE
	call    	send_char
	gotp		chg_filt

; ###################################################################
; #        Debounce delay = 5e-6*255*24 +4e-6*24 = 30.6mS           #
; ###################################################################
	 
de_bounce
	movlw		d'24'
	movwf		nest_count 			; Init embedded counter

delay_ext
	clrf 		count 				; Init counter to zero
	delay		nop
	incf		count 
	btfss		STATUS,Z
	goto		delay
	decfsz		nest_count
	goto		delay_ext
	return
 
 
; ###################################################################
; #                        ADC setup sub-routine                   ##
; ###################################################################
	 	 
adc_setup
	bsf			STATUS,RP0 			; Change page to 1
	movlw		b'01000001'			; Bit 6 defines fosc ; MSB --> 1 R-justified, MSB --> 0 L-justified
	movwf 		ADCON1 				; RA0 --> RA2 analogue inputs, RA3 = Vref
	movlw 		0xff
	movwf 		TRISA 				; All PORTA pins inputs
	bcf 		STATUS,RP0 			; Set page to 0
	movlw 		b'00010000' 		; ORIGINAL VALUE - 10010000 ; 2 MSBs defines fosc
	movwf 		ADCON0 				; Set ADC clock to fosc/32
	 								; Select channel 2 and turn on ADC
	bcf 		INTCON,ADIE 		; Disable a-d conversion complete interupt
	bsf 		ADCON0,ADON 		; Turn on ADC
	return
	 
; #####################################################################
; #                   Set-up TIMER sub-routine                        #
; #####################################################################
	  
TIMER_setup
	bcf 		INTCON,TMR0IF 		; Make sure that the overflow flag is reset
	bsf 		STATUS,RP0 			; Change to page 1
	movlw 		b'11010000' 		; Select fosc/4 clock
			   						; and prescales clock by a factor of 2:1
	movwf 		OPTION_REG 			;
	bcf 		STATUS,RP0 			; Change to page 0
	movlw 		d'110'				; Approx. 1uS *(255-130) = 125uS
	movwf 		TIMER 				; Move number into TIMER
	return
 
 
; ######################################################################
; #                       Straight-through filter                      #
; ######################################################################
	  
all_pass
	bcf 		PORTB,RB6 			; Turn off differencer indicator
	bcf 		PORTB,RB7			; Turn off averager indicator
	bsf 		PORTB,RB5 			; Turn on straigh-through indicator
	movf 		x_k,0
	movwf 		y_k 				; y(k)=x(k)
	goto 		next
	return
 
 
; ######################################################################
; #                           High-pass filter                         #
; ######################################################################
	 
differencer
	bcf 		PORTB,RB5 			; Turn off straight-through indicator
	bcf 		PORTB,RB7			; Turn off averager indicator
	bsf 		PORTB,RB6 			; Turn on differencer indicator
	movf 		x_k_1,0 			; w=x(k-1)
	subwf 		x_k,0 				; w=x(k)-x(k-1)
	movwf 		y_k 				; y(k)=x(k)-x(k-1)
	movf 		x_k,0
	movwf 		x_k_1
	return
 

; ######################################################################
; #                             Low-pass filter                        #
; ######################################################################
	 
averager
	bcf 		PORTB,RB5 			; Turn off straight-through indicator
	bcf 		PORTB,RB6 			; Turn off differencer indicator
	bsf 		PORTB,RB7			; Turn on averager indicator
	movf 		x_k_1,0 			; w=x(k-1)
	addwf 		x_k,0 				; w=x(k)+x(k-1)
	movwf 		y_k 				; y(k)=x(k)+x(k-1)
	call 		overflow
	movf 		x_k,0
	movwf 		x_k_1
	return
 
; ###################################################################### 
; #                   Overflow test sub-routine                        #
; ######################################################################
;
; NOTE: This routine is not required for all filters, but in some instances
; it will be needed. i.e. 85+85= overflow.
	 
	overflow 
	btfss 		x_k,sign 			; Test for sign of input data.
	goto 		positive
	btfss 		x_k_1,sign 			; Test for possibility of negative overflow
	goto 		test_end
	btfsc 		y_k,sign 			; Test for negative overflow
	goto 		test_end
	movlw 		0x80
	movwf 		y_k 				; Saturate result register to -128
	goto 		test_end
	positive 	btfsc x_k_1,sign 	; Test for possibility of +ve sat
	goto 		test_end
	btfss 		y_k,sign 			; Test for positive overflow
	goto 		test_end
	movlw 		d'127'
	movwf 		y_k 				; Saturate result to +ve = 127
	test_end 	return

; ######################################################################
; #                  DAC driver routine for DAC-8512                   #
; ######################################################################
; Note: The MSB of the 12-bit word is loaded first
; Note: DAC requires a minimum clock pulse width of 40nS
; Load pulse width of 20nS
; Approx 30nS per data value to setup
; FFF = 4.095 Volts; 000 = 0 Volts; input to DAC = unsigned
; Sequence: load bit, clock DAC for 12 times, once all 12-bits in DACS
; Serial register uses the load signal LD to convert value
	
	driver 
	bcf 		STATUS,C 			; Make sure that carry bit is clear
	msb_test 	rlf INDF,1 			; Move bit under test into carry
	btfss 		STATUS,C
	goto 		zero 				; Bit ='0'
	bsf 		PORTB,RB3 			; Drive the SDI pin of DAC high
	bsf 		PORTB,RB2 				
	bcf 		PORTB,RB2 			; Toggle clock to get falling edge
	decfsz 		bit_count,1
	goto 		msb_test 			; More bits to test
	return

	zero
	bcf 		PORTB,RB3 			; Drive the DAC SDI pin Low
	bsf 		PORTB,RB2 				
	bcf 		PORTB,RB2 			; Toggle clock to get falling edge
	decfsz 		bit_count,1
	goto 		msb_test 			; All MSB bits tested
	return
 
 
; ######################################################################
; #                       Main DAC driver routine                      #
; ######################################################################
dac_driver 
	movlw 		y2_k
	movwf 		FSR 				; Set-up pointer for dual word result
	bsf 		PORTB,RB4 			; Make sure that the load signal is false
	bcf 		PORTB,RB2 			; Make sure that the clock is low
	movlw 		d'4' 
	movwf 		bit_count 			; Init bit count for dac driver routine
	swapf 		y2_k,1 				; Move lower nibble to upper nibble
	call 		driver
	movlw 		d'8'
	movwf 		bit_count 			; Init bit count for lsb driver
	incf 		FSR,1
	call 		driver
	nop 							; Allow for DAC to settle down
	bcf 		PORTB,RB4 			; Set load signal true
	nop 							; Allow DAC to settle down
	bsf 		PORTB,RB4 			; Make load signal false
	return
 
 
; ######################################################################
; #                      Change mode sub-routine                       #
; ######################################################################
	
change_mode	
	decfsz 		filter,1
	return
	movlw 		d'3'
	movwf 		filter 				; Keep filter value in range 3 --> 1
	return
		
change_disp	
	btfss		filter,1
	goto		hp_disp 			; Display HIGH-PASS MODE on LCD
	btfss		filter,0
	goto		ap_disp				; Display ALL-PASS MODE on LCD
	btfsc		filter,0
	call		lp_disp				; Display LOW-PASS MODE on LCD
	goto		chg_filt		 		 

high_disp
	call		hp_disp

chg_filt    	return
	

; ######################################################################
; #                      Sign extend y(k) to 12-bits                   #
; ######################################################################
	
sign_extend
	btfsc 		y_k,sign
	goto 		pos
	clrf 		y2_k 				; Set extended sign bits to 0
	goto 		bigger

	pos
	movwf 		y2_k 				; Move for 1's into sign-extension word
									; Multiply result by 16 so that more DAC levels will be excited
	bigger 		bcf STATUS,C
	rlf 		y_k,1
	rlf 		y2_k,1
	bcf 		STATUS,C
	rlf 		y_k,1
	rlf 		y2_k,1
	bcf 		STATUS,C
	rlf 		y_k,
	rlf 		y2_k,1
	bcf 		STATUS,C
	rlf 		y_k,1
	rlf 		y2_k,1
	movlw 		b'00001000' 		; 12th bit of result set
	addwf 		y2_k,1 				; Add 2048 to y(k)
	return

	
; ################################################################################################
; #                                           Main program                                       #
; ################################################################################################

main_start							; Initialise variables and hardware
	bcf			STATUS,RP1 			; Start from page 0
	clrf 		INTCON 				; Make sure ge_mode interrupts disabled for now
	clrf 		y_k 				; y(k) = 0 to start
	clrf 		y2_k 				; Yupper=0 to start
	clrf 		x_k_1 				; x(K-1)= 0 to start
	clrf 		x_k 				; x(k)=0 to start
	movlw 		d'2'
	movwf 		filter 				; Mode value default = straight-through
	bsf 		STATUS,RP0 			; Page 1
	movlw 		b'00000010'
	movwf 		TRISB 				; RB1 = input; all other pins output
	bcf 		STATUS,RP0 			; Page 0
	call 		define_ports		; Define ports
	call 		init_lcd			; Initialise LCD
	bcf 		PORTB,RB7 			; Turn off averager inidicator
	bcf 		PORTB,RB6 			; Turn off differencer indicator
	bsf 		PORTB,RB5 			; Turn on straight-through indicator
	call 		adc_setup 			; Set-up ADC as per specification
	bsf 		INTCON,TMR0IE 		; Enable TIMER overflow locally
	bsf 		INTCON,GIE 			; Enable interrupts globally
	bsf 		ADCON0,GO 			; Start conversion; get samples!
	call 		TIMER_setup 		; Set-up TIMER so that fosc/4 clock is scaled 2:1
	call 		AP_disp				; Display ALL-PASS MODE on LCD

		
mode_key
	btfsc 		PORTB,RB1 			; Test asserted low mode key sw_L
	goto 		mode_key
	call 		de_bounce 			; Debounce key for 30mS
	btfsc 		PORTB,RB1
	goto 		mode_key 			; Debounce failed
	call 		change_mode 		; Key has been pressed change mode value
	call 		change_disp
		 
no_depress	
	btfss 		PORTB,RB1
	goto 		no_depress
	call 		de_bounce 			; Debounce for 30mS
	btfss 		PORTB,RB1
	goto 		no_depress
	goto 		mode_key
		 
		 
isr_start
	movwf 		temp
	movf 		ADRESH,0 			; Get latest sample of data
	movwf 		x_k 				; New sample = x(k), input range 85-->255
	bsf 		ADCON0,GO 			; Start conversion for next time
	call 		TIMER_setup 		; Set-up TIMER for another 125uS interval
	movlw 		d'159'
	subwf 		x_k,1 				; Remove the input DC bias
		 
	; Now input is in range -85 --> 85
	; Overflow still possible
	; Which filter mode?
	btfss		filter,1
	goto		high_pass 
	btfss		filter,0
	goto		all_pass
	btfsc		filter,0
	call		averager
	goto		next

high_pass
	call		differencer

next
	call		sign_extend
	call		dac_driver
	movf 		temp,0
	retfie
	
 end 






