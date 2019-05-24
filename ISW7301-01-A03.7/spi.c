/*
 * File: spi.c
 * Author: km6dpv
 * 
 * Created on Nov. 14, 2017, 15:26
 */

//#include <xc.h>
//#include <pic16lf1829.h>
#include <htc.h>
#include <stdint.h>
#include "config.h"
#include "cc1120_reg_config.h"
#include "spi.h"


/****************************************************************************
 * LOCAL FUNCTIONS
 */


void initializeSPI()
{
//    APFCON0bits.SDOSEL = 0;         // SDO function on pin RC7
//    APFCON0bits.SSSEL = 0;          // ~SS is always on pin RC6
    
    TRISC6 = 0;                     // ~SS is output
    TRISC7 = 0;                     // SDO is output
    TRISB4 = 1;                     // SDI is input
    TRISB6 = 0;                     // SCL is output
    
    //---------- configs for SSPCON register------------
    //Synchronous Serial Port Mode Select bits
    //configs as SPI Master mode, clock = FOSC / (4 * (SSPADD+1))
    SSP1STAT = 0x00;
    SSP1CON1 = 0x00;
    SSP1CON1bits.SSPM = 0b0000;     // SCK = F_osc/16
    
    
    SSP1CON1bits.CKP = 0;
    SSP1STATbits.CKE = 1;
    SSP1STATbits.SMP = 0;
    
    SSP1CON2 = 0;
    
    SSP1ADD = 9;        // 100kHz clock speed with 4MHz xtal
    
    //---------PIE1----------
    PIE1bits.SSP1IE = 0;//1;
    
    //---------PIR1----------
    PIR1bits.SSP1IF = 0;        // Clr MSSP IF
    
    //---------PIR2----------
    //  PIR2bits.BCL1IF = 0;    // Clr bus Collision IF bit
    
    SSP1CON1bits.SSPEN = 1;
}


uint8_t SPISend(uint8_t data)
{
    uint8_t temp;

    SSP1IF = 0;
    SSP1BUF = data;
    while(!SSP1IF);
    temp = SSPBUF;//read the buffer
    return temp;//return the buffered byte

    //while(BF) continue;// wait until complete this bit transmission
}


rfStatus_t cc1120SpiWriteReg(uint16_t addr, uint8_t *data, uint8_t len)
{
    uint8_t tempExtended = (uint8_t)(addr>>8);
    uint8_t tempAddr = (uint8_t)(addr & 0x00FF);
    uint8_t returnData;
    /* Is this a FIFO access? -> Returns chip not ready */
    if((CC1120_SINGLE_TXFIFO <= tempAddr) && (tempExtended == 0))
        return STATUS_CHIP_RDYn_BM;
    
    /* Determine register space you want to access */
    if(!tempExtended)
    {
        returnData = trx8BitRegAccess((RADIO_BURST_ACCESS | RADIO_WRITE_ACCESS),\
                tempAddr, data, len);
    }
    else if (tempExtended == 0x2F)      // Access extended space
    {
        returnData = trx16BitRegAccess((RADIO_BURST_ACCESS | RADIO_WRITE_ACCESS),\
                tempExtended, tempAddr, data, len);
    }
    
    return returnData;
}

rfStatus_t cc1120SpiReadReg(uint16_t addr, uint8_t *data, uint8_t len)
{
    uint8_t tempExtended = (uint8_t)(addr>>8);
    uint8_t tempAddr = (uint8_t)(addr & 0x00FF);
    uint8_t returnData;
    
    
    /* Is this a FIFO access? -> Returns chip not ready */
    if((CC1120_SINGLE_TXFIFO <= tempAddr) && (tempExtended == 0))
        return STATUS_CHIP_RDYn_BM;
    
    /* Determine register space you want to access */
    if(!tempExtended)
    {
        returnData = trx8BitRegAccess((RADIO_BURST_ACCESS | RADIO_READ_ACCESS),\
                tempAddr, data, len);
    }
    else if (tempExtended == 0x2F)      // Access extended space
    {
        returnData = trx16BitRegAccess((RADIO_BURST_ACCESS | RADIO_READ_ACCESS),\
                tempExtended, tempAddr, data, len);
    }
    
    return returnData;
}

rfStatus_t trxCmdStrobe(uint8_t cmd)
{
    uint8_t rc;
    TRXEM_SPI_BEGIN();
    while(MISO);
    
    rc = SPISend(cmd);
    TRXEM_SPI_END();
    return rc;
}


rfStatus_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData, uint8_t len)
{
  uint8_t readValue;

  /* Pull CS_N low and wait for SO to go low before communication starts */
  TRXEM_SPI_BEGIN();
//  while(MISO);
  __delay_us(10);
  
  /* send register address byte & store chip status */
  readValue = SPISend((uint8_t)(accessType|addrByte));

  trxReadWriteBurstSingle((uint8_t)(accessType|addrByte),pData,len);
  TRXEM_SPI_END();
  /* return the status byte value */
  return(readValue);
}

rfStatus_t trx16BitRegAccess(uint8_t accessType, uint8_t extAddr, uint8_t regAddr, uint8_t *data, uint8_t len)
{
  uint8_t readValue;

  TRXEM_SPI_BEGIN();
  while(MISO);
  
  /* send extended address byte with access type bits set */
  readValue = SPISend((uint8_t)(accessType|extAddr));
  SPISend(regAddr);
  
  /* Send <len> # of bytes */
  trxReadWriteBurstSingle((uint8_t)(accessType|extAddr),data,len);
  TRXEM_SPI_END();
  /* return the status byte value */
  return(readValue);
}

void trxReadWriteBurstSingle(uint8_t addr, uint8_t *data, uint8_t len)
{
    uint16_t i;
    
    /* Send <len> # of bytes. Send 0's for Rx. */
    if(addr & RADIO_READ_ACCESS)
    {
        if(addr & RADIO_BURST_ACCESS)
        {
            for (i = 0; i < len; i++)
            {
                *data = SPISend(0);
                data++;
            }
        }
        else
        {
            *data = SPISend(0);
        }
    }
    else
    {
        if (addr & RADIO_BURST_ACCESS)
        {
            /* Send <len> # of bytes. Doesn't overwrite data. */
            for (i = 0; i < len; i++)
            {
                SPISend(*data);
                data++;
            }
        }
        else
        {
            SPISend(*data);
        }
    }
    return;
}

/*******************************************************************************
 * @fn          cc115lSpiWriteTxFifo
 *
 * @brief       Write pData to radio transmit FIFO.
 *
 * input parameters
 *
 * @param       *pData - pointer to data array that is written to TX FIFO
 * @param       len    - Length of data array to be written
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
rfStatus_t cc1120SpiWriteTxFifo(unsigned char *pData, unsigned char len)
{
  unsigned char rc;
  rc = trx8BitRegAccess(0x00, CC1120_BURST_TXFIFO, pData, len);
//  cc1120SpiWriteReg(CC1120_BURST_TXFIFO, pData, len);
  return (rc);
}

/******************************************************************************
 * @fn      cc115lGetTxStatus(void)
 *          
 * @brief   This function transmits a No Operation Strobe (SNOP) to get the 
 *          status of the radio and the number of free bytes in the TX FIFO.
 *          
 *          Status byte:
 *          
 *          ---------------------------------------------------------------------------
 *          |          |            |                                                 |
 *          | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO |
 *          |          |            |                                                 |
 *          ---------------------------------------------------------------------------
 *
 *
 * input parameters
 *
 * @param   none
 *
 * output parameters
 *         
 * @return  rfStatus_t 
 *
 */
rfStatus_t cc1120GetTxStatus(void)
{
    return(trxCmdStrobe(CC1120_SNOP));
}
