/*
	Charlieplex display con interfaccia RS232 (da sigarette vape (sozzoni inglesi!)

	G.Dar 27/4/2025  #govesuviogo #goWW3go vancouver in the heart, palermo in der cul!
	( https://www.instructables.com/Charlieplexing-7-segment-displays/
*/

// If you want to use pin 4 (GP3) as an input, be sure to disable MCRLE in your config. Here is my 12F683 configuration:
// CONFIG
// INTRCIO  			//4MHz su 12F675, su 683 fino a 8
//#pragma config FOSC = INTOSCCLK // Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSC oscillator: I/O function on RA4/RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON      // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF       // Brown Out Detect (BOR disabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode is enabled)
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)


#define _XTAL_FREQ 8000000UL			// PIC12F683  


#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pic12f683.h>
#include "display_212_232.h"

 
#pragma config IDLOC0 = 0x44
#pragma config IDLOC1 = 0x47


const char CopyrightString[]= {'D','i','s','p','l','a','y',' ','2','3','2',' ','-',' ','v',
	VERNUMH+'0','.',VERNUML/10+'0',(VERNUML % 10)+'0', ' ','-',' ', '2','7','/','0','4','/','2','5', 0 };

//const char CopyrDate[]={ 'S','/','N',' ','0','0','0','1', CR,LF,0 };

const char * Copyr1="(C) Dario's Automation 2025 - G.Dar\xd\xa\x0";

const uint8_t table_7seg[]={0, // .GFEDCBA
															0b00111111,0b00000110,0b01011011,0b01001111,0b01100110,
												 			0b01101101,0b01111101,0b00000111,0b01111111,0b01101111,
															0b00000100,			// il segno %
															0b00000010,			// goccia
															0b00000001			// batteria
															};


volatile uint8_t Timer1;					// usato con Timer3 per gestire "eventi/RTC"
uint16_t CounterL=0xc00;
uint8_t CounterH;	
volatile uint8_t ByteRec;

// The array containing the digits to be Shown on the display
uint8_t digits[4],digitPos;



void interrupt isr() {		// 157uS prescaler=3, 8/12/24 8MHz 12F683
  static uint8_t currentdigit = 0b00000001;  // Keeps track of the multiplex
  static uint8_t bitpos=0,byteToRec=0;

	if(INTCONbits.GPIF) {
    GPIO;
    
    if(!GPIObits.GP3) {   // se scende, start!
      ByteRec=Do232I();
//      VEDERE se fare tutto da qua, che causa flicker, o andare bit per bit... serve timer per bit-time
//      if(bitpos>=10)
//        ByteRec=byteToRec;
      }
    
    
    INTCONbits.GPIF=0;
		}
  
	if(INTCONbits.TMR0IF) {				// Timer 0		~???Hz
		INTCONbits.TMR0IF=0;					// clear bit IRQ
		TMR0 += TMR0BASE;		// uso +, è meglio!


	  // Set all LED portpins as inputs with the corresponding
	  // 	segments of the displays will be off
	  TRISIO |= (
	    CHARLIE_1_A_VAL | CHARLIE_2_A_VAL | CHARLIE_3_A_VAL | CHARLIE_4_A_VAL | CHARLIE_5_A_VAL
	     );
	
		GPIO = currentdigit;
		TRISIO &= ~currentdigit;
	
	  // Here we do the bit-fiddling thats neccesary to charlieplex.
	  // Depending on That Which digit are active now we need to handle the situation in its own unique way.
	  //
	  switch(currentdigit) {
	    case 0b00000001:
				GPIO = CHARLIE_1_A_VAL;
				TRISIO &= ~CHARLIE_1_A_VAL;
			  if(digits[3] & 0b00000001) 		// batteria
					TRISIO &= ~CHARLIE_5_A_VAL;
			  if(digits[3] & 0b00000010) 		// goccia
					TRISIO &= ~CHARLIE_4_A_VAL;
			  if(digits[3] & 0b00000100) 		// %
					TRISIO &= ~CHARLIE_3_A_VAL;
	      break;
	    case 0b00000010:
				GPIO = CHARLIE_2_A_VAL;
				TRISIO &= ~CHARLIE_2_A_VAL;
			  if(digits[0] & 0b00000010) 			// 1B
					TRISIO &= ~CHARLIE_3_A_VAL;
			  if(digits[0] & 0b00000100) 			// 1C
					TRISIO &= ~CHARLIE_4_A_VAL;
			  if(digits[2] & 0b00010000) 			// 3E
					TRISIO &= ~CHARLIE_5_A_VAL;
			  if(digits[1] & 0b01000000) 			// 2G
					TRISIO &= ~CHARLIE_1_A_VAL;
	      break;
	    case 0b00000100:
				GPIO = CHARLIE_3_A_VAL;
				TRISIO &= ~CHARLIE_3_A_VAL;
			  if(digits[2] & 0b00000100) 			// 3C
					TRISIO &= ~CHARLIE_5_A_VAL;
			  if(digits[1] & 0b00000001) 			// 2A
					TRISIO &= ~CHARLIE_4_A_VAL;
			  if(digits[1] & 0b00000100) 			// 2C
					TRISIO &= ~CHARLIE_2_A_VAL;
			  if(digits[1] & 0b00100000) 			// 2F
					TRISIO &= ~CHARLIE_1_A_VAL;
	      break;
	    case 0b00010000:
				GPIO = CHARLIE_4_A_VAL;
				TRISIO &= ~CHARLIE_4_A_VAL;
			  if(digits[2] & 0b00000001) 			// 3A
					TRISIO &= ~CHARLIE_5_A_VAL;
			  if(digits[1] & 0b00000010) 			// 2B
					TRISIO &= ~CHARLIE_3_A_VAL;
			  if(digits[1] & 0b00001000) 			// 2D
					TRISIO &= ~CHARLIE_2_A_VAL;
			  if(digits[1] & 0b00010000) 			// 2E
					TRISIO &= ~CHARLIE_1_A_VAL;
	      break;
	    case 0b00100000:
				GPIO = CHARLIE_5_A_VAL;
				TRISIO &= ~CHARLIE_5_A_VAL;
			  if(digits[2] & 0b00000010) 			// 3B
					TRISIO &= ~CHARLIE_4_A_VAL;
			  if(digits[2] & 0b00001000) 			// 3D
					TRISIO &= ~CHARLIE_3_A_VAL;
			  if(digits[2] & 0b00100000) 			// 3F
					TRISIO &= ~CHARLIE_2_A_VAL;
			  if(digits[2] & 0b01000000) 			// 3G
					TRISIO &= ~CHARLIE_1_A_VAL;
	      break;
			}

	  // Show next digit at the next interrupt. This cannot be done
	  //  at the top of the ISR since the digit variable is needed
	  //  in the switch-statement later on.
	  currentdigit <<= 1;
	  if(currentdigit==0b00001000) 
			currentdigit <<= 1;
	  if(currentdigit & 0b01000000) 
			currentdigit=0b00000001;

		}
  
	if(PIR1bits.TMR2IF) {
    PIR1bits.TMR2IF=0;

		Timer1++;
		}
  
	}


void main(void) {
	uint8_t i,j,temp;
	char buf[10];


cold_reset:

	CLRWDT();

  OSCCON=0b01100001;    // a 8Mhz non va, è instabile @#£$%
  OSCTUNE=0b00000000;
//  OSCCONbits.SCS=1;
//	while(!OSCCONbits.HFIOFR);		// inutili...
	while(!OSCCONbits.HTS);
  OSCCONbits.IRCF=0b111;    // boh così pare forse...

  
	TRISIO=0b00111111;						// tutto off
	GPIO =0b00000000;						// 
	__delay_ms(100);		// sembra sia meglio aspettare un pizzico prima di leggere la EEPROM.. (v. forum 2006)


	GPIO = 0b00111111;

	TRISIO=0b00001000;						// GP3

	TMR0=0;						//clr tmr0 & prescaler

	INTCON=0;					// disabilito tutti interrupt

	T2CON =0b00010100;	//1:3 postscaler, 1:1 prescaler; 
	// WGM0=0b0101, TCCR0B<4:3> + TCCR0A<1:0>
  PR2 = TMR2BASE;
	TMR2=0;
  PIE1bits.TMR2IE=1; 


	CLRWDT();


 	OPTION_REG=0b00000100;		// prescaler Timer0 1:32 NON USATO QUA; pullup; INT pin su falling edge (no usiamo IOC)
	CMCON0 =0x07;			// Set GP<2:0> to digital I/O

	ANSEL=0b00000000;
	WPU = 0b00001000;	//pull up ma su GP3 non c'è



warm_reset:

	__delay_ms(100);
	i=0;
  digitPos=0;

	/*while(1) {
		m_LedOBit ^= 1;
	__delay_ms(100);
}
*/


	INTCON=0b11101000;				// GIE, PEIE, timer + GPIO
	IOC = 0b00001000;     // GP3/MCLR per 232
	
	CounterL=0xc00;
	CounterH=0;

	do {


    if(ByteRec) {
      if(ByteRec=='\f') {
        digits[0]=digits[1]=digits[2]=digits[3]=0;
        digitPos=0;
        }
      else if(ByteRec=='\n' || ByteRec=='\r') {
        digitPos=0;
        }
      else if(ByteRec>='0' && ByteRec<='9') {     // isdigit
        if(digitPos<4)
          digits[digitPos++]=ByteRec;
        }
      else if(ByteRec==' ') {
        if(digitPos<4)
          digits[digitPos++]=0;
        }
      else if(ByteRec=='%') {
        digits[3] |= 0b00000100;
        }
      else if(ByteRec=='$') {   // be' batteria :)
        digits[3] |= 0b00000001;
        }
      else if(ByteRec=='*') {   // goccia
        digits[3] |= 0b00000010;
        }
      ByteRec=0;
      }

		if(!--CounterL) {					// quando il contatore e' zero...
			CounterL=0x600;

			CounterH--;

			if(!(CounterH & FLASH_TIME))
				goto ResCnt;
			}

NoResCnt:

		continue;

ResCnt:
//		m_LedOBit ^= 1;


		CLRWDT();
		} while(1);

	}


uint8_t Do232I(void) {								// RS232, input (8,n,1): esce in A il BYTE se C=0 o C=1=errore
	uint8_t i,temp;
//	uint8_t delay_232;


//	INTCONbits.TMR0IE=0;					// disable se no disturba! anche 220V??
	// usare if(DataRdyUSART() temp=ReadUSART();

	i=8;
// COME faccio ad aspettare lo start?? no, messa in irq...

// poi, aspetto mezzo bit dopo l'inizio di start..
	__delay_us(BIT_TIME/2);

	temp=0;
	do {
		__delay_us(BIT_TIME);
	
		temp >>= 1;						// bit al livello TTL...; LSB first
		if(GPIObits.GP3)
			temp |= 0x80;

		}	while(--i);

#ifdef PARITA_232
	Do232dl();
	i=GetParita(temp);
	if(!SPI_232_I) {
// parita' ricevuta = 0, quindi se la mia e' 1 equivale ad errore
		}
	else {
//	parita' ricevuta = 1, quindi do C=0 se la mia e' anch'essa 1, opp. 1
		}
#endif

// ignoro stop...

//	INTCONbits.TMR0IE=1;					// 
	return temp;			//FINIRE PARITA'!!!

	}


// ---------------------------------------------------------------------

