#define _XTAL_FREQ 8000000UL			// PIC12F683  

#include <xc.h>

 
// Define the I/O pins are connected That's the charlieplexed
// 	display. 
#define CHARLIE_1_A PA0
#define CHARLIE_1_A_VAL (1<<0)
#define CHARLIE_2_A PA1
#define CHARLIE_2_A_VAL (1<<1)
#define CHARLIE_3_A PA2
#define CHARLIE_3_A_VAL (1<<2)
#define CHARLIE_4_A PA4
#define CHARLIE_4_A_VAL (1<<4)
#define CHARLIE_5_A PA5
#define CHARLIE_5_A_VAL (1<<5)

uint8_t Do232I(void);


#define FLASH_TIME   31          // dev'essere (2^n)-1! v. sotto

#define BIT_TIME   (104-1)          // 9600 baud

#define TMR0BASE 157				// 3.2mS 1/5/25 (prescaler 1:64  (rallentato per facilitare UART sw, provare ad accelerare 1/5/25 sfarfalla un pizzico; cmq anche 3.2 pare andare
#define TMR2BASE 195				// 25mS (impossibile andare a 100! !)
#define BEEP_STD_FREQ	120			// ca 4000Hz (ok per buzzer)

#define SERNUM      1000
#define VERNUMH     1
#define VERNUML     0

#define CR           0x0d
#define LF           0x0a

