/* 
 * File:   uart.h
 * Author: Scott
 *
 * Created on February 12, 2018, 1:58 PM
 */

#ifndef UART_H
#define	UART_H


#define RXD_IN      TRISC5
#define TXD_IN      TRISC4
#define GSM_INT     RA2

void init_uart();
void start_uart();
void construct_uart_packet(uint8_t rxBuffer[], uint8_t numBytes, uint8_t txBuffer[]);
void terminate_uart();
void tell_mother(uint8_t rxBuffer[], uint8_t numBytes);
void wakeup_GSM_and_send_header(uint8_t headerByte);
void enable_RX_uart_interrupt();
void write_uart(unsigned char data);


unsigned char RS232Buf[6];
volatile bool haveHeader = false;
volatile bool uartRxIsFull = false;
volatile bool receivedCR = false;
volatile bool receivedLF = false;
uint8_t repeatUART = 5;         /* Repeat UART transmission this many times */
uint8_t pos = 0;
uint8_t test1;
bool testMatch;

#endif	/* UART_H */

