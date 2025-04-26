
#include <p18cxxx.h>
#include <delays.h>
#include <string.h>
#include <generictypedefs.h>

typedef BYTE * SHORTPTR;			//compatibilità con pic18F1320

void low_isr(void);
void high_isr(void);

void Delay_uS(BYTE);
void Delay_S_(BYTE);
void Delay_ms(WORD);

 
// Define the I/O pins are connected That's the charlieplexed
// 	display. The suffix A makes it easier for me to keep
// 	track of the PORT Belongs to the bit.
#define CHARLIE_1_A PA0
#define CHARLIE_1_A_VAL (1<<0)
#define CHARLIE_2_A PA1
#define CHARLIE_2_A_VAL (1<<1)
#define CHARLIE_3_A PA2
#define CHARLIE_3_A_VAL (1<<2)
#define CHARLIE_4_A PA3
#define CHARLIE_4_A_VAL (1<<3)
#define CHARLIE_5_A PA4
#define CHARLIE_5_A_VAL (1<<4)


#define LedOBit		5			// attivo alto; 
#define m_LedOBit		LATCbits.LATC5
#define LedOVal (1 << LedOBit)


#define FLASH_TIME   31          // dev'essere (2^n)-1! v. sotto

#define TMR0BASE 158
#define TMR3BASE (65536-50000)						//2012: usato per time-oraesatta; 50mS (impossibile andare a 100!)
#ifdef __18F26K20
#define BEEP_STD_FREQ	230			// ca 4000Hz (ok per buzzer) @64MHz!
#else
#define BEEP_STD_FREQ	120			// ca 4000Hz (ok per buzzer)
#endif

#define SERNUM      1000
#define VERNUMH     1
#define VERNUML     0

#define CR           0x0d
#define LF           0x0a

