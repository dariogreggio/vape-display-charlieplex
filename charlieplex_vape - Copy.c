/*
	Charlieplex display demo da sigarette vape (sozzoni inglesi!)
	PIC18F2620 su piastrino luppino PdM 2005-6
		J1: 1: RA5
				2: RE3 (MCLR)
				3: RB2
				4: RB6
				5: RB7
				6: RC2
				7: RC6
				8: RC7
				9: Vcc
				10: GND
		P3: 1: RB0
				2: RB3
				3: RC1
				4: RC0
				5: RA4
				6: RA3
				7: RA2
				8: RA1
				9: RA0

	G.Dar 26/4/2025  #govesuviogo #goWW3go
	portions from tiny_charlieplex/matseng (instructables, merduino ;)
	https://www.instructables.com/Charlieplexing-7-segment-displays/
*/


#include <p18cxxx.h>
#include <delays.h>
// #include <stdint.h> // non c'� pd @#�$% metto in generictypedefs
#include <string.h>
#include <timers.h>
#include <portb.h>
#include <pwm.h>
#include <reset.h>
#include <generictypedefs.h>
#include "charlieplex_vape.h"

 
#ifdef __18F26K20
#pragma config WDTEN = ON, WDTPS = 256 /*8192*/, MCLRE=ON, STVREN=ON, LVP=OFF					// no input MCLR
#pragma config FOSC = INTIO7, FCMEN=OFF, IESO=ON, PWRT=ON, BOREN=ON, BORV=22			// BOR/POR off per Sleep!
#pragma config PBADEN=OFF, LPT1OSC = OFF, CCP2MX = PORTC, XINST=ON			?? verificare ext qua
#else
#pragma config WDT = ON, WDTPS = 256 /*8192*/, MCLRE=ON, STVREN=ON, LVP=OFF					// no input MCLR
#pragma config OSC = INTIO7, FCMEN=OFF, IESO=ON, PWRT=ON, BOREN=ON, BORV=2			// BOR/POR off per Sleep!
#pragma config PBADEN=OFF, LPT1OSC = OFF, CCP2MX = PORTC, XINST=ON
#endif


#pragma romdata myidlocs=0x200000
const rom char data0=0x34u;
const rom char data1=0x37u;
const rom char data2=0x34u;
const rom char data3=0x34u;
#pragma romdata


rom const char CopyrightString[]= {'C','h','a','r','l','i','e','p','l','e','x',' ','d','i','s','p','l','a','y',' ','-',' ','v',
	VERNUMH+'0','.',VERNUML/10+'0',(VERNUML % 10)+'0', ' ','-',' ', '2','6','/','0','4','/','2','5', 0 };

rom const char CopyrDate[]={ 'S','/','N',' ','0','0','0','1', CR,LF,0 };

rom const char * rom Copyr1="(C) Dario's Automation 2025 - G.Dar\xd\xa\x0";


#pragma udata

volatile BYTE Timer1;					// usato con Timer3 per gestire "eventi/RTC"
WORD CounterL=0xc00;
BYTE CounterH;	

// The array containing the digits to be Shown on the display
BYTE digits[6];


#ifdef __18F26K20		// no ACCESS qua in Extended! azz
#pragma udata 
#else
#pragma udata access ACCESSBANK
#endif


#pragma udata



#pragma code high_vector=0x08
void interrupt_at_high_vector(void) {
  _asm goto high_isr _endasm
	}

#pragma code low_vector=0x18
void interrupt_at_low_vector(void) {
  _asm goto low_isr _endasm
	}

#pragma code

#pragma interrupt high_isr  nosave=FSR2 //non gli piace!! e perch� si ostina a salvarlo?
void high_isr(void) {
  static BYTE currentdigit = 0;  // Keeps track of the digit to be Shown
  BYTE segments;                               // The bitpattern of the current digit

	if(INTCONbits.TMR0IF) {				// Timer 0
		INTCONbits.TMR0IF=0;					// clear bit IRQ
		TMR0L += TMR0BASE;		// uso +, � meglio!
				//WRITETIMER0() dovrebbe essere la macro! su C30...


  // Get the bit pattern of the current digit
  segments=digits[currentdigit];

  // Set all LED portpins as inputs with the Corresponding
  // segments of the displays will be off
  TRISA |= (
    CHARLIE_1_A_VAL | CHARLIE_2_A_VAL | CHARLIE_3_A_VAL | CHARLIE_4_A_VAL | CHARLIE_5_A_VAL
     );

  TRISB |= (
    CHARLIE_6_B_VAL | CHARLIE_7_B_VAL | CHARLIE_8_B_VAL | CHARLIE_9_B_VAL
     );


  // Set low level LEDs on all port pins. This prepares the segments
  // to be lit if the pin is changed to output at a later stage. The
  // displays of common ar-type anode (positive common) with the segments
  // needs to be sunk to ground to be turned on.
  LATA &= ~(
    CHARLIE_1_A_VAL | CHARLIE_2_A_VAL | CHARLIE_3_A_VAL | CHARLIE_4_A_VAL | CHARLIE_5_A_VAL
     );

  LATB &= ~(
    CHARLIE_6_B_VAL | CHARLIE_7_B_VAL | CHARLIE_8_B_VAL | CHARLIE_9_B_VAL
     );


  // Set as output portpins for segments to be lit
  // We just assume That Each segment has it's standard
  // place in the connections. The special case is handled in the switch-statement below.
  if(segments & 0b00000001) 
		TRISA &= ~CHARLIE_1_A_VAL;
  if(segments & 0b00000010) 
		TRISA &= ~CHARLIE_2_A_VAL;
  if(segments & 0b00000100) 
		TRISA &= ~CHARLIE_3_A_VAL;
  if(segments & 0b00001000) 
		TRISA &= ~CHARLIE_4_A_VAL;
  if(segments & 0b00010000) 
		TRISA &= ~CHARLIE_5_A_VAL;
  if(segments & 0b00100000) 
		TRISB &= ~CHARLIE_6_B_VAL;
  if(segments & 0b01000000)
		TRISB &= ~CHARLIE_7_B_VAL;
  if(segments & 0b10000000) 
		TRISB &= ~CHARLIE_8_B_VAL;

  // Here we do the bit-fiddling thats neccesary to charlieplex.
  // Since one of the segments (each different) of each loop display
  // is moved to the 9'th connection we need to take care of that.
  //
  // Depending on That Which digit are active now we need to handle the situation in its own unique way.
  //
  // The A segment on the first digit is moved from the line 1'th
  // to the line 9'th basically so be it The Same thing as in the bunch
  // of tests above, but only test for the A segment and if it's lit
  // we turn on the 9'th line instead of the first line.  
  // We then need to activate the transistor That handles the common
  // anode for the first digit. The transistor for the first display
  // is connected to the 1'th line (where the A-segment usualy go).
  // So we turn on the output for That pin and set it high.
  //
  // The next time this routine is Called we do with The Same Thing
  // the second digit. But we then check for the B-segment and so on ...
  switch(currentdigit) {
    case 0:
      if(segments & 0b00000001) 
				TRISB &= ~CHARLIE_9_B_VAL;
      TRISA &= ~CHARLIE_1_A_VAL;
      LATA |= CHARLIE_1_A_VAL;
      break;
    case 1:
      if(segments & 0b00000010) 
				TRISB &= ~CHARLIE_9_B_VAL;
      TRISA &= ~CHARLIE_2_A_VAL;
      LATA |= CHARLIE_2_A_VAL;
      break;
    case 2:
      if(segments & 0b00000100) 
				TRISB &= ~CHARLIE_9_B_VAL;
      TRISA &= ~CHARLIE_3_A_VAL;
      LATA |= CHARLIE_3_A_VAL;
      break;
    case 3:		// qua ci sono gli extra/simboli
      if(segments & 0b00001000) 
				TRISB &= ~CHARLIE_9_B_VAL;
      TRISA &= ~CHARLIE_4_A_VAL;
      LATA |= CHARLIE_4_A_VAL;
      break;
/*    case 4:
      if(segments & 0b00010000) 
				TRISB &= ~CHARLIE_9_B_VAL;
      TRISA &= ~CHARLIE_5_A_VAL;
      LATA |= CHARLIE_5_A_VAL;
      break;
    case 5:
      if(segments & 0b00100000) 
				TRISB &= ~CHARLIE_9_B_VAL;
      TRISB &= ~CHARLIE_6_B_VAL;
      LATB |= CHARLIE_6_B_VAL;
      break;*/
		}

  // Show next digit at the next interrupt. This can not be done
  // at the top of the ISR since the digit variable is needed
  // in the switch-statement later on.
  currentdigit++;
  if(currentdigit>3) 
		currentdigit=0;

		}
	}

#pragma interruptlow low_isr
void low_isr(void) {
	BYTE temp,byteRec;

	if(PIR2bits.TMR3IF) {				// Timer 3
		TMR3H+=TMR3BASE >> 8;
		TMR3L+=TMR3BASE & 255;
		PIR2bits.TMR3IF=0;					// clear bit IRQ

		Timer1++;
		}
	}

#pragma code

void main(void) {
	BYTE i,j,temp;
	BYTE digit, seg;


cold_reset:

//	RCONbits.NOT_BOR=1;
	STKPTR=0;		// risparmia una posizione di Stack, xch� main() � stata CALLed!
	StatusReset();

	ClrWdt();

	// imposto oscillatore a 32MHz
	//con 26K20 dovrebbe essere 64!
	OSCCONbits.IRCF2=1;
	OSCCONbits.IRCF1=1;
	OSCCONbits.IRCF0=1;
	OSCTUNEbits.PLLEN=1;

	OSCCONbits.SCS0 = 0;
	OSCCONbits.SCS1 = 0;

	OSCCONbits.IDLEN=0;

	TRISA=0b00000000;						// resetto subito uscite! e il vecchio picfruit18? boh vabbe'
	LATA =0b00000000;						// 
	Delay_ms(100);		// sembra sia meglio aspettare un pizzico prima di leggere la EEPROM.. (v. forum 2006)


	LATA = 0b00011111;

	LATB=0b00000000;			// input & altro
	LATC = 0b00000011;

	
	TRISA=0b00111111;						// 5 input, 1 I/O input D opp. A (non uso MCLR, ma reset interno)
	TRISB=0b10011011;							// 1 input, 3 I/O output
	TRISC=0b11011011;							// 2 input, 3 I/O input o 232! 0=out x buzzer (RB3)


	WriteTimer0(0);						//clr tmr0 & prescaler
			//WRITETIMER0() dovrebbe essere la macro!


//	OPTION_REG=0x7;		// pull-ups B, tmr0 enable, falling-edge portB (x dimmer), prescaler Timer with 1:256 prescaler

	INTCON=0;					// disabilito tutti interrupt


	OpenRB0INT(PORTB_CHANGE_INT_OFF /*& PORTB_PULLUPS_ON*/);
	TRISB=0b10011011;							// 1 input, 3 I/O output


	OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_8);			// NON si pu� usare x per Sleep / power saving
					// Timer-0 sempre high Pty 
	OpenTimer3(TIMER_INT_ON & T3_8BIT_RW & T3_SOURCE_INT & T3_PS_1_8);			// ??NON si pu� usare x per Sleep / power saving
	IPR2bits.TMR3IP = 0;				// Timer-3 Low Pty 

	EnablePullups();		//
// quello in openrb0int non funzia... 2012...
	RCONbits.IPEN=1;				// interrupt in Modalit� avanzata


	ClrWdt();


	
/*	CCP1CON=0b00001100;			//			 TMR2: PWM mode
	CCPR1L=BEEP_STD_FREQ/2;				// duty cycle 50%
//	movlw		BEEP_STD_FREQ/2
//	movwf	 CCPR1H				; questo � read-only in PWM-mode
*/

	//T2CON=0b00000011;								// set prescaler T2
	OpenTimer2(TIMER_INT_OFF & T2_PS_1_16 & T2_POST_1_1);		//0.125*16 = 2uS
	OpenPWM1(BEEP_STD_FREQ); SetDCPWM1((BEEP_STD_FREQ*4L)/2 /* MSB di 200 << 6 va in CCPR1L..*/);		// SetOutputPWM1(SINGLE_OUT,PWM_MODE_1); // FINIRE! ma SetOutputPWM1 c'� solo sui moduli per motori ossia 4 PWM
	OpenPWM1ConfigIO();			// fa il TRIS...
//	IPR1bits.TMR2IP = 0;				// Timer-2 Low Pty NON USATO
	ClosePWM1();
//T2CONbits.TMR2ON = 0;




	ClrWdt();

#ifdef __18F26K20
	CM1CON0=0b00000111;			// okkio ai comparatori, non c'era ma dovrebbe servire... 11/2008
	CM2CON0=0b00000111;			// 
//	OpenADC(ADC_FOSC_32 & ADC_RIGHT_JUST & ADC_12_TAD, ADC_CH10 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,
//		ADC_1ANA);
	ANSEL= 0b00000000;
	ANSELH=0b00000000;			// v. comando 105!
	
#else
	CMCON=0b00000111;			// okkio ai comparatori, non c'era ma dovrebbe servire... 11/2008
//	OpenADC(ADC_FOSC_32 & ADC_RIGHT_JUST & ADC_12_TAD, ADC_CH4 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,
//		ADC_4ANA /* & 0xf */ /*patch di g.dar*/);
	// VERIFICA su questo PIC! si potrebbe usare RB4, che pero' e' EUSART!
	ADCON1=0b00001111;	//		; Tutto disattivato! FARE 1 Analogici ! e left-justified
	ADCON0=0b00000000;	//		; TUTTO DISATTIVATO RA0 (obbligatorio...)
#endif



warm_reset:


	Delay_ms(100);


	INTCONbits.PEIE = 1;			// attiva interrupt perif (SERVE Timer1


	INTCON=0b11100000;				// solo timer, 
	
	do {


		for(seg=0; seg<8; seg++) {
			for(digit=0; digit<6; digit++) {
				digits[digit]=1<<seg;
				Delay_ms(50);
				digits[digit]=0;
				}
			}

		if(!--CounterL) {					// quando il contatore e' zero...
#ifdef __18F26K20
			CounterL=0xc00;		// 64MHz!
#else
			CounterL=0x600;
#endif

			CounterH--;
			}

		if(!(CounterH & FLASH_TIME))
			goto ResCnt;

NoResCnt:

		continue;

ResCnt:
		LATC ^= LedOVal;
		// Display 012345 for one second
		digits[0]=0x3f;
		digits[1]=0x06;
		digits[2]=0x5b;
		digits[3]=0x4F;
		digits[4]=0x6E;
		digits[5]=0x6d;
		Delay_ms(1000);
		digits[0]=0x00;
		digits[1]=0x00;
		digits[2]=0x00;
		digits[3]=0x00;
		digits[4]=0x00;
		digits[5]=0x00;

		} while(1);

	}


void Delay_uS(BYTE uSec) {

	// 3/4 di prologo...
	do {
		ClrWdt();			// 1
		Delay1TCY();			//  @32MHz
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		// 1
//		Delay1TCY();		mah
		} while(--uSec);		// 2

	}

void Delay_ms(BYTE temp) {

	do {
		Delay_uS(250);
		} while(--temp);	    // 2 
	}												// 1


// ---------------------------------------------------------------------

                              // ...o solo in EPROM!
void EEscrivi_(SHORTPTR addr,BYTE n) {	// non uso puntatore, xch� va a finire a 16bit! SPRECONI!

	EEADR = (BYTE)addr;
	EEDATA=n;

	EECON1bits.EEPGD=0;		// Point to Data Memory
	EECON1bits.CFGS=0;		// Access EEPROM
	EECON1bits.WREN=1;

	INTCONbits.GIE = 0;			// disattiva interrupt globali
	EECON2=0x55;		 // Write 55h
	EECON2=0xAA;		 // Write AAh
	EECON1bits.WR=1;									// abilita write.
	INTCONbits.GIE = 1;			// attiva interrupt globali
	do {
		ClrWdt();
		} while(EECON1bits.WR);							// occupato ? 


	EECON1bits.WREN=0;								// disabilita write.
  }

BYTE EEleggi(SHORTPTR addr) {

	EEADR=(BYTE)addr;			// Address to read
	EECON1bits.EEPGD=0;		// Point to Data Memory
	EECON1bits.CFGS=0;		// Access EEPROM
	EECON1bits.RD=1;		// EE Read
	return EEDATA;				// W = EEDATA
	}

