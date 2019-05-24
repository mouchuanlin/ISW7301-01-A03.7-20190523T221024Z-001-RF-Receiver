/* 
 * File:   config.h
 * Author: Scott
 *
 * Created on November 14, 2017, 2:49 PM
 */

#ifndef CONFIG_H
#define	CONFIG_H

// CONFIG1
#pragma config FOSC = XT       // Oscillator Selection (ECH, External Clock, High Power Mode (4-32 MHz): device clock supplied to CLKIN pin)
#pragma config WDTE = SWDTEN        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = ON          // Flash Program Memory Code Protection (Program memory code protection is enabled)
#pragma config CPD = ON         // Data Memory Code Protection (Data memory code protection is enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ  4000000
#define BAUD        9600
#define SDI_PIN     TRISB4
#define SDO_PIN     TRISC7
//#define JUMPER    RC4

#define WATCHDOG_SLEEP_256ms    0b01000
#define WATCHDOG_SLEEP_512ms    0b01001
#define WATCHDOG_SLEEP_1S       0b01010
#define WATCHDOG_SLEEP_2S       0b01011
#define WATCHDOG_SLEEP_4S       0b01100
#define WATCHDOG_SLEEP_8S       0b01101
#define WATCHDOG_SLEEP_16S      0b01110
#define WATCHDOG_SLEEP_32S      0b01111
#define WATCHDOG_SLEEP_64S      0b10000
#define WATCHDOG_SLEEP_128S     0b10001
#define WATCHDOG_SLEEP_256S     0b10010


#define G_LED           LATCbits.LATC2
#define R_LED           LATBbits.LATB5
#define SW_1            RA0                 // Active-low
#define GPIO0           RC3
#define GPIO2           RB7
#define CSense_ASSERTED     RC3                 // GPIO0

#define SMOKE_TYPE      0b1000
#define FLOOD_TYPE      0b0110
#define CO_TYPE         0b0010
#define GLASS_TYPE      0b1100
#define MOTION_TYPE     0b1001
#define DOOR_TYPE       0b0011
#define PANIC_TYPE      0b0001
#define HVAC_TYPE       0b1011
#define APPLIANCE_TYPE  0b0101
//#define TBD10_TYPE      0b0100
//#define TBD11_TYPE      0b0111
//#define TBD12_TYPE      0b1010
//#define TBD13_TYPE      0b1101
//#define TBD14_TYPE      0b1110
//#define TBD15_TYPE      0b0000
//#define TBD16_TYPE      0b1111
    
#define MAX_CMDS_STORED 20
#define _7MIN           801
#define _4MIN           938
#define _30S_TICK       7           // ticks required for test signal timeout of ~30s.

#define _T2_7S          54

#include <stdint.h>
#include <stdbool.h>

enum TimerState {
    OFF,
    ON,
    TIMER_DONE,
    TEST            // shortened timeout to 30s
};


void check7minTimer();
void start_rssi_timer();

extern unsigned short packetCounter;
bool _7minTimerOn = false;
bool alreadyAsserted = false;
bool rssiTimerDone = false;
bool alreadyReset = true;
uint8_t delayCount = 0;
uint8_t t2cnt = 0;

uint16_t msgTmrCnt[MAX_CMDS_STORED] = 0x00;
enum TimerState msgTmrState[MAX_CMDS_STORED] = 0x00;
uint32_t msgReceived[MAX_CMDS_STORED];           // Stores all received messages
uint8_t endMsgPtr = 0;              // tracks the end of the received message buffer


const uint8_t UNIT_NUM @ 0x0080 = 0x01;

#endif	/* CONFIG_H */