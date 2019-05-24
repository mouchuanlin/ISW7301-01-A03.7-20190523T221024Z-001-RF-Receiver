
#include <xc.h>
#include <stdbool.h>
#include "config.h"
#include "uart.h"

/*************************************************
 * LOCAL VARIABLES
 ************************************************/
static uint8_t wdtSaveBits, intconSaveBits, pirSaveBits;
static uint16_t _3s = 3000;


/* Initialize uart interface. Assumes 9600 baud. */
void init_uart()
{
    RXD_IN = 1;         /* Set RX pin as input */
    TXD_IN = 0;         /* Set TX pin as output */
    
    BRGH = 1;           /* Baud = 9600 */
    SPBRG = 25;         /* */
}

void start_uart()
{
    SYNC = 0;           /* Config uart as asynchronous */
    SPEN = 1;           /* Enable UART module */
    
    /* Enable both Rx and Tx ports */
    CREN = 1;
    TXEN = 1;
    
    WDTCONbits.SWDTEN = 0;
    wdtSaveBits = WDTCON;
//    WDTCONbits.WDTPS = 0b10001;
    WDTCONbits.SWDTEN = 1;
}

void enable_RX_uart_interrupt()
{
    uartRxIsFull = false;
    haveHeader = false;
    receivedCR = false;
    receivedLF = false;
    intconSaveBits = INTCON;
//    PEIE = 1;
    pirSaveBits = PIR1;
    PIR1 = 0;
    RCIE = 1;
    RCIF = 0;
    INTCONbits.PEIE = 1;
    GIE = 1;
}
void disable_RX_uart_interrupt()
{
    RCIE = 0;
    INTCON = intconSaveBits;
    PIR1 = pirSaveBits;
}

void terminate_uart()
{
    SPEN = 0;
    CREN = 0;
    TXEN = 0;
    WDTCON = wdtSaveBits;
    WDTCONbits.SWDTEN = 1;
}

void construct_uart_packet(uint8_t rxBuffer[], uint8_t numBytes, unsigned char txBuffer[])
{
    /* For adapting to current hub software with two chips */
    txBuffer[0] = '$';
    /* Write serial ID as-is to output buffer */
    for(uint8_t i = 0; i < 4; i ++)
        txBuffer[i + 1] = rxBuffer[i];
    
//    txBuffer[9] = rxBuffer[4];              // e.g. high byte of CRC
//    txBuffer[10] = rxBuffer[5];             // e.g. low byte of CRC
    
    txBuffer[5] = '\r';                     // to accommodate current hub
    txBuffer[6] = '\n';                     // to accommodate current hub
}

void wakeup_GSM_and_send_header(uint8_t headerByte)
{
    GSM_INT = 1;
    for (uint8_t i = 0; i < 10; i++)        // 1s
    {
        CLRWDT();
        __delay_ms(100);
    }
    CLRWDT();
    write_uart(headerByte);
    GSM_INT = 0;
}


/* Tell the motherboard what we received */
void tell_mother(uint8_t rxBuffer[], uint8_t numBytes)
{
    uint8_t txBuffer[7];
    uint16_t counter = 0;
    uint8_t i = 0;
    /* */
    init_uart();
    construct_uart_packet(rxBuffer, numBytes, txBuffer);
    start_uart();
    //    for (uint8_t i = 0; i < repeatUART; i ++)

    while (i++ < repeatUART && !(receivedCR && receivedLF))
    {
        wakeup_GSM_and_send_header(txBuffer[0]);
        for (uint8_t j = 1; j < sizeof(txBuffer); j ++)
        {    write_uart(txBuffer[j]);   }
        
        enable_RX_uart_interrupt();
        uartRxIsFull = false;
        counter = 0;                        // Clear timer
        
        while(!uartRxIsFull && (counter <= _3s))
        {
            /* Split up because interrupt cannot occur from within delay() */
            CLRWDT();
            __delay_ms(1);
            counter++;
            if (receivedCR && receivedLF)
                uartRxIsFull = true;
        }
        
        disable_RX_uart_interrupt();
        
        if (receivedCR && receivedLF)
        {
            haveHeader = false;
            pos = 0;
            if(RS232Buf[1] == rxBuffer[0] && RS232Buf[2] == rxBuffer[1] \
                    && RS232Buf[3] == rxBuffer[2] && RS232Buf[4] == rxBuffer[3])
            {
                R_LED = 1;
                CLRWDT();
                __delay_ms(150);
                CLRWDT();
                __delay_ms(150);
                CLRWDT();
                R_LED = 0;
            }
        }
        else
        {
            /* Toggle USART receiver */
            pos = 0;
            CREN = 0;
            __delay_us(20);
            CREN = 1;
        }
    }
    if (receivedCR && receivedLF)         // if received UART, start timer and increment to next valid pkt spot in receivedMsg array; if not, repeat on reception of next pkt.
    {
        receivedCR = false;
        receivedLF = false;
        if (testMatch)
            msgTmrState[endMsgPtr] = TEST;
        else
            msgTmrState[endMsgPtr] = ON;
        if (!_7minTimerOn)
        {
            // No longer using timer 1 to track time elapsed
//            T1CONbits.nT1SYNC = 1;
//            T1CONbits.TMR1CS = 0b10;
//            T1CONbits.T1CKPS = 0b11;
//            T1CONbits.T1OSCEN = 1;
//            INTCONbits.GIE = 0;
//            T1CONbits.TMR1ON = 1;
//            PIE1bits.TMR1IE = 1;
//            INTCONbits.PEIE = 1;
//            PIR1bits.TMR1IF = 0;
//            INTCONbits.GIE = 1;
            _7minTimerOn = true;
        }
        endMsgPtr++;
    }
    else
    {
        msgReceived[endMsgPtr] = 0x00;          // renew msgReceived
//        if (endMsgPtr > 0)
//            endMsgPtr--;
    }
    
    terminate_uart();
}


void write_uart(unsigned char data)
{
    while (!TRMT);              // Wait for UART TX buffer to empty completely
    TXREG = data;
}