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
	VERNUMH+'0','.',VERNUML/10+'0',(VERNUML % 10)+'0', ' ','-',' ', '0','1','/','0','5','/','2','5', 0 };

//const char CopyrDate[]={ 'S','/','N',' ','0','0','0','1', CR,LF,0 };

const char * Copyr1="(C) Dario's Automation 2025 - G.Dar\xd\xa\x0";

const uint8_t table_7seg[]={0, // .GFEDCBA
															0b00111111,0b00000110,0b01011011,0b01001111,0b01100110,
												 			0b01101101,0b01111101,0b00000111,0b01111111,0b01101111,
													0b01000000,
													0b01110111,0b01111100,0b00111001,0b01011110,0b01111001,0b01110001,	//A..F
													0b00111101,0b01110110,0b00110000,0b00001110,0b01110110,0b00111000,	//G..L
													0b00110111,0b01010100,0b01011100,0b01110011,0b01100111,0b01010000,	//M..R
													0b01101101,0b01111000,0b00111110,0b00011100,0b00011100,0b00110110,	//S..X
													0b01100010,0b01011011,		//Y..Z
															0b00000100,			// il segno %
															0b00000010,			// goccia
															0b00000001			// batteria
															};


volatile uint8_t Timer1;					// usato con Timer3 per gestire "eventi/RTC"
uint16_t CounterL=0x300;
uint8_t CounterH;	
volatile uint8_t ByteRec;

// The array containing the digits to be Shown on the display
uint8_t digits[4],digitPos;



void interrupt isr() {		// 3.2mS prescaler=1:64, 1/5/25 8MHz 12F683
  static uint8_t currentdigit = 0b00000001;  // Keeps track of the multiplex
//  static uint8_t bitpos=0,byteToRec=0;

	if(INTCONbits.GPIF) {
//    GPIO;
    
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
	
//		GPIO = currentdigit;
//		TRISIO &= ~currentdigit;
	
	  // Here we do the bit-fiddling thats necessary to charlieplex.
	  // Depending on That Which digit are active now we need to handle the situation in its own unique way.
	  //
	  switch(currentdigit) {
	    case 0b00000001:
				GPIO = CHARLIE_1_A_VAL;
				TRISIO &= ~CHARLIE_1_A_VAL;
			  if(digits[3] & 0b00000001) 		// batteria/fulmine
					TRISIO &= ~CHARLIE_5_A_VAL;
			  if(digits[3] & 0b00000010) 		// goccia/lucchetto
					TRISIO &= ~CHARLIE_2_A_VAL;
			  if(digits[3] & 0b00000100) 		// %
					TRISIO &= ~CHARLIE_3_A_VAL;
			  if(digits[3] & 0b00001000) 		//
					TRISIO &= ~CHARLIE_4_A_VAL;
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
#ifndef __DEBUG
//  OSCCONbits.SCS=1;
//	while(!OSCCONbits.HFIOFR);		// inutili...
	while(!OSCCONbits.HTS);
  OSCCONbits.IRCF=0b111;    // boh così pare forse...
#endif

  
	TRISIO=0b00111111;						// tutto off
	GPIO =0b00000000;						// 
	__delay_ms(100);		// sembra sia meglio aspettare un pizzico prima di leggere la EEPROM.. (v. forum 2006)


	GPIO = 0b00111111;

	TRISIO=0b00001000;						// GP3

	TMR0=0;						//clr tmr0 & prescaler

	INTCON=0;					// disabilito tutti interrupt

	T2CON =0b01111111;	//1:16 postscaler, 1:16 prescaler; 
	// WGM0=0b0101, TCCR0B<4:3> + TCCR0A<1:0>
  PR2 = TMR2BASE;
	TMR2=0;
  PIE1bits.TMR2IE=1; 


	CLRWDT();


 	OPTION_REG=0b00000101;		// prescaler Timer0 1:64; pullup; INT pin su falling edge (no usiamo IOC)
	CMCON0 =0x07;			// Set GP<2:0> to digital I/O

	ANSEL=0b00000000;
	WPU = 0b00001000;	//pull up ma su GP3 non c'è



warm_reset:

	__delay_ms(100);
	i=0;

	/*while(1) {
		m_LedOBit ^= 1;
	__delay_ms(100);
}
*/


	INTCON=0b11101000;				// GIE, PEIE, timer + GPIO
	IOC = 0b00001000;     // GP3/MCLR per 232
	
	CounterL=0x300;
	CounterH=0;
  digits[0]=digits[3]=0;
  digits[2]=digits[1]=0b01000000;   // boot :)  OCCHIO seg. 2F è rotto...
  digitPos=0;

	do {
    uint8_t seg,digit;
/*		for(seg=0; seg<7; seg++) {
			for(digit=0; digit<4; digit++) {
				digits[digit]=1<<seg;
				__delay_ms(100);
				digits[digit]=0;
				}
			}*/
//#define TEST_SEGMENTI
#ifdef TEST_SEGMENTI
#define DELAY1 2000
#define DELAY2 1000
		INTCON=0;
		TRISIO=255;
		GPIO=  0b00000001;
		__delay_ms(DELAY1);
		TRISIO=0b00011110;		// batteria/fulmine
		__delay_ms(DELAY2);
		TRISIO=0b00101110;		// (nulla)/corto con fulmine..
		__delay_ms(DELAY2);
		TRISIO=0b00111010;		// %
		__delay_ms(DELAY2);
		TRISIO=0b00111100;		// goccia/lucchetto
		__delay_ms(DELAY2);

		TRISIO=255;
		GPIO=  0b00000010;		//
		__delay_ms(DELAY1);
		TRISIO=0b00011101;		// 3E
		__delay_ms(DELAY2);
		TRISIO=0b00101101;		// 1C
		__delay_ms(DELAY2);
		TRISIO=0b00111001;		// 1B
		__delay_ms(DELAY2);
		TRISIO=0b00111100;		// 2G
		__delay_ms(DELAY2);

		TRISIO=255;
		GPIO=  0b00000100;
		__delay_ms(DELAY1);
		TRISIO=0b00011011;		// 3C
		__delay_ms(DELAY2);
		TRISIO=0b00101011;		// 2A
		__delay_ms(DELAY2);
		TRISIO=0b00111001;		// 2C
		__delay_ms(DELAY2);
		TRISIO=0b00111010;		// 2F ???
		__delay_ms(DELAY2);

		TRISIO=255;
		GPIO=  0b00010000;
		__delay_ms(DELAY1);
		TRISIO=0b00001111;		// 3A
		__delay_ms(DELAY2);
		TRISIO=0b00101011;		// 2B
		__delay_ms(DELAY2);
		TRISIO=0b00101101;		// 2D
		__delay_ms(DELAY2);
		TRISIO=0b00101110;		// 2E
		__delay_ms(DELAY2);

		TRISIO=255;
		GPIO=  0b00100000;
		__delay_ms(DELAY1);
		TRISIO=0b00001111;		// 3B
		__delay_ms(DELAY2);
		TRISIO=0b00011011;		// 3D
		__delay_ms(DELAY2);
		TRISIO=0b00011101;		// 3F
		__delay_ms(DELAY2);
		TRISIO=0b00011110;		// 3G
		__delay_ms(DELAY2);

		TRISIO=255;
#endif

    if(ByteRec) {
/*		sprintf(buf,"%03u",ByteRec);
		digits[0]=table_7seg[buf[0]-'0'+1];
		digits[1]=table_7seg[buf[1]-'0'+1];
		digits[2]=table_7seg[buf[2]-'0'+1];*/
      switch(ByteRec) {
        case '\x1b':
          // bah il mux va ma non riceve + e non pulisce... RESET();      // vabbe' :)
          goto warm_reset;
          break;
        case '\f':
          digits[0]=digits[1]=digits[2]=digits[3]=0;
          digitPos=0;
          break;
        case '\n':
        case '\r':
          digitPos=0;
          break;
        case ' ':
          if(digitPos<4)
            digits[digitPos++]=0;
          break;
        case '.':     // 
          digits[digitPos>0 ? digitPos-1 : digitPos] |= 0b10000000;   // diciamo (qua non c'è
          break;
        case '-':     // 
          digits[0] |= 0b01000000;   // qua non c'è
          break;
        case '%':
          digits[3] |= 0b00000100;
          break;
        case '$':   // be' batteria :)
          digits[3] |= 0b00000001;
          break;
        case '*':   // goccia/lucchetto
          digits[3] |= 0b00000010;
          break;
        default:
          if(ByteRec>='0' && ByteRec<='9') {     // isdigit
            if(digitPos<3)
              digits[digitPos++]=table_7seg[ByteRec-'0'+1];
            }
          else if(ByteRec>='A' && ByteRec<='Z') {     //
            if(digitPos<3)
              digits[digitPos++]=table_7seg[ByteRec-'A'+1+10+1];
            }
          else if(ByteRec>='a' && ByteRec<='z') {     //
            if(digitPos<3)
              digits[digitPos++]=table_7seg[ByteRec-'a'+1+10+1];
            }
          break;
         }
      ByteRec=0;
      }

		if(!--CounterL) {					// quando il contatore e' zero...
			CounterL=0x100;

			CounterH--;

			if(!(CounterH & FLASH_TIME))
				goto ResCnt;
			}

NoResCnt:

		continue;

ResCnt:
//		m_LedOBit ^= 1;
          
/*		i++;
//		itoa(i % 100,&buf);
		sprintf(buf,"%03u",i % 100);
//		sprintf(buf,"%03u",Timer1 % 100);	// test ok 25mS 30/4/25
		digits[1]=table_7seg[buf[1]-'0'+1];
		digits[2]=table_7seg[buf[2]-'0'+1];

		if(i>=200) {
			// add rain :) goccia/lucchetto
			digits[0]=0;
			digits[3]=0b00000010;
			}
		else if(i>=100) {
			digits[0]=table_7seg[buf[0]-'0'+1];
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
  */


		CLRWDT();
		} while(1);

	}


uint8_t Do232I(void) {								// RS232, input (8,n,1): esce in A il BYTE se C=0 o C=1=errore
	uint8_t i,temp;


//	INTCONbits.TMR0IE=0;					// disable se no disturba! 
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

	__delay_us(BIT_TIME);
//	__delay_us((BIT_TIME*3)/2);   // stop, faccio così per non perdere troppo tempo prima del succ. char
	if(!GPIObits.GP3)     // stop bit
    temp=0;     // framing error...

//	INTCONbits.TMR0IE=1;					// 
	return temp;			//FINIRE PARITA'!!!

	}


// ---------------------------------------------------------------------

