/*
	Charlieplex display demo da sigarette vape (sozzoni inglesi!)
	PIC18F2620 su piastrino luppino PdM 2005-6
		J1: 1: RA5
				2: RE3 (MCLR)
				3: RB2
				4: RB6
				5: RB7
				6: RC2	(CCP1=buzzer)
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
Led=RC5

	G.Dar 26/4/2025  #govesuviogo #goWW3go
	portions from tiny_charlieplex/matseng (instructables, merduino ;)
	https://www.instructables.com/Charlieplexing-7-segment-displays/
*/


#include <p18cxxx.h>
#include <delays.h>
// #include <stdint.h> // non c'è pd @#£$% metto in generictypedefs
#include <stdio.h>
#include <stdlib.h>
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

const rom BYTE table_7seg[]={0, // .GFEDCBA
															0b00111111,0b00000110,0b01011011,0b01001111,0b01100110,
												 			0b01101101,0b01111101,0b00000111,0b01111111,0b01101111,
															0b00000100,			// il segno %
															0b00000010,			// goccia
															0b00000001			// batteria
															};

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

#pragma interrupt high_isr  //nosave=FSR2 //non gli piace!! e perché si ostina a salvarlo?
void high_isr(void) {
  static BYTE currentdigit = 0;  // Keeps track of the multiplex

	if(INTCONbits.TMR0IF) {				// Timer 0
		INTCONbits.TMR0IF=0;					// clear bit IRQ
		TMR0L += TMR0BASE;		// uso +, è meglio!
				//WRITETIMER0() dovrebbe essere la macro! su C30...


	  // Set all LED portpins as inputs with the Corresponding
	  // 	segments of the displays will be off
	  TRISA |= (
	    CHARLIE_1_A_VAL | CHARLIE_2_A_VAL | CHARLIE_3_A_VAL | CHARLIE_4_A_VAL | CHARLIE_5_A_VAL
	     );
	
		LATA = 1 << currentdigit;
		TRISA &= ~(1 << currentdigit);
	
	  // Here we do the bit-fiddling thats neccesary to charlieplex.
	  // Depending on That Which digit are active now we need to handle the situation in its own unique way.
	  //
	  switch(currentdigit) {
	    case 0:
				LATA = CHARLIE_1_A_VAL;
				TRISA &= ~CHARLIE_1_A_VAL;
			  if(digits[3] & 0b00000001) 		// batteria
					TRISA &= ~CHARLIE_5_A_VAL;
			  if(digits[3] & 0b00000010) 		// goccia
					TRISA &= ~CHARLIE_4_A_VAL;
			  if(digits[3] & 0b00000100) 		// %
					TRISA &= ~CHARLIE_3_A_VAL;
	      break;
	    case 1:
				LATA = CHARLIE_2_A_VAL;
				TRISA &= ~CHARLIE_2_A_VAL;
			  if(digits[0] & 0b00000010) 			// 1B
					TRISA &= ~CHARLIE_3_A_VAL;
			  if(digits[0] & 0b00000100) 			// 1C
					TRISA &= ~CHARLIE_4_A_VAL;
			  if(digits[2] & 0b00010000) 			// 3E
					TRISA &= ~CHARLIE_5_A_VAL;
			  if(digits[1] & 0b01000000) 			// 2G
					TRISA &= ~CHARLIE_1_A_VAL;
	      break;
	    case 2:
				LATA = CHARLIE_3_A_VAL;
				TRISA &= ~CHARLIE_3_A_VAL;
			  if(digits[2] & 0b00000100) 			// 3C
					TRISA &= ~CHARLIE_5_A_VAL;
			  if(digits[1] & 0b00000001) 			// 2A
					TRISA &= ~CHARLIE_4_A_VAL;
			  if(digits[1] & 0b00000100) 			// 2C
					TRISA &= ~CHARLIE_2_A_VAL;
			  if(digits[1] & 0b00100000) 			// 2F
					TRISA &= ~CHARLIE_1_A_VAL;
	      break;
	    case 3:
				LATA = CHARLIE_4_A_VAL;
				TRISA &= ~CHARLIE_4_A_VAL;
			  if(digits[2] & 0b00000001) 			// 3A
					TRISA &= ~CHARLIE_5_A_VAL;
			  if(digits[1] & 0b00000010) 			// 2B
					TRISA &= ~CHARLIE_3_A_VAL;
			  if(digits[1] & 0b00001000) 			// 2D
					TRISA &= ~CHARLIE_2_A_VAL;
			  if(digits[1] & 0b00010000) 			// 2E
					TRISA &= ~CHARLIE_1_A_VAL;
	      break;
	    case 4:
				LATA = CHARLIE_5_A_VAL;
				TRISA &= ~CHARLIE_5_A_VAL;
			  if(digits[2] & 0b00000010) 			// 3B
					TRISA &= ~CHARLIE_4_A_VAL;
			  if(digits[2] & 0b00001000) 			// 3D
					TRISA &= ~CHARLIE_3_A_VAL;
			  if(digits[2] & 0b00100000) 			// 3F
					TRISA &= ~CHARLIE_2_A_VAL;
			  if(digits[2] & 0b01000000) 			// 3G
					TRISA &= ~CHARLIE_1_A_VAL;
	      break;
			}

  // Show next digit at the next interrupt. This cannot be done
  //  at the top of the ISR since the digit variable is needed
  //  in the switch-statement later on.
  currentdigit++;
  if(currentdigit>4) 
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
	BYTE digit,seg;
	char buf[10];


cold_reset:

//	RCONbits.NOT_BOR=1;
	STKPTR=0;		// risparmia una posizione di Stack, xché main() è stata CALLed!
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

	LATB = 0b00000000;			// 
	LATC = 0b00000000;

	
	TRISA=0b00000000;						// 
	TRISB=0b00000000;						// 
	TRISC=0b00000000;						// 0=out x buzzer 


	WriteTimer0(0);						//clr tmr0 & prescaler
			//WRITETIMER0() dovrebbe essere la macro!


//	OPTION_REG=0x7;		// pull-ups B, tmr0 enable, falling-edge portB (x dimmer), prescaler Timer with 1:256 prescaler

	INTCON=0;					// disabilito tutti interrupt


	OpenRB0INT(PORTB_CHANGE_INT_OFF /*& PORTB_PULLUPS_ON*/);
	TRISB=0b00000000;							//


	OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_8);			// NON si può usare x per Sleep / power saving
					// Timer-0 sempre high Pty 
	OpenTimer3(TIMER_INT_ON & T3_8BIT_RW & T3_SOURCE_INT & T3_PS_1_8);			// ??NON si può usare x per Sleep / power saving
	IPR2bits.TMR3IP = 0;				// Timer-3 Low Pty 

	EnablePullups();		//
// quello in openrb0int non funzia... 2012...
	RCONbits.IPEN=1;				// interrupt in Modalità avanzata


	ClrWdt();


	
/*	CCP1CON=0b00001100;			//			 TMR2: PWM mode
	CCPR1L=BEEP_STD_FREQ/2;				// duty cycle 50%
//	movlw		BEEP_STD_FREQ/2
//	movwf	 CCPR1H				; questo è read-only in PWM-mode
*/

	//T2CON=0b00000011;								// set prescaler T2
	OpenTimer2(TIMER_INT_OFF & T2_PS_1_16 & T2_POST_1_1);		//0.125*16 = 2uS
	OpenPWM1(BEEP_STD_FREQ); SetDCPWM1((BEEP_STD_FREQ*4L)/2 /* MSB di 200 << 6 va in CCPR1L..*/);		// SetOutputPWM1(SINGLE_OUT,PWM_MODE_1); // FINIRE! ma SetOutputPWM1 c'è solo sui moduli per motori ossia 4 PWM
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
	i=0;

	/*while(1) {
		m_LedOBit ^= 1;
	Delay_ms(100);
}
*/

	INTCONbits.PEIE = 1;			// attiva interrupt perif (SERVE Timer1

	INTCON=0b11100000;				// solo timer, 
	
	CounterL=0xc00;
	CounterH=0;

	do {


/*		for(seg=0; seg<8; seg++) {
			for(digit=0; digit<3; digit++) {
				digits[digit]=1 << seg;
				Delay_ms(250);
				digits[digit]=0;
				}
			}*/


		if(!--CounterL) {					// quando il contatore e' zero...
#ifdef __18F26K20
			CounterL=0xc00;		// 64MHz!
#else
			CounterL=0x600;
#endif

			CounterH--;

			if(!(CounterH & FLASH_TIME))
				goto ResCnt;
			}

NoResCnt:

		continue;

ResCnt:
		m_LedOBit ^= 1;

		i++;
//		itoa(i % 100,&buf);
		sprintf(buf,"%03u",i % 100);
		digits[1]=table_7seg[buf[1]-'0'+1];
		digits[2]=table_7seg[buf[2]-'0'+1];

		if(i>=200) {
			// add rain :)
			digits[0]=0;
			digits[3]=0b00000010;
			}
		else if(i>=100) {
			digits[0]=table_7seg[buf[0]-'0'+1];
			// add rain :)
			digits[3]=0;
			}
		else {
			digits[0]=0;
			digits[3]=0;
			}
		if(i & 4)
			digits[3] |= 0b00000001;		// add battery

		if(i>=250)
			i=0;

		ClrWdt();
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

void Delay_ms(WORD temp) {

	do {
		Delay_uS(250);
		Delay_uS(250);
		Delay_uS(250);
		Delay_uS(250);
		} while(--temp);	    // 2 
	}												// 1


// ---------------------------------------------------------------------

void EEscrivi_(SHORTPTR addr,BYTE n) {	// non uso puntatore, xché va a finire a 16bit! SPRECONI!

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

