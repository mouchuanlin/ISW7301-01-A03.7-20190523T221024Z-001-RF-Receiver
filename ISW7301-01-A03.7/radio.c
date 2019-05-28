
#include <xc.h>
#include <htc.h>
#include <stdint.h>
#include <pic16lf1829.h>
#include "config.h"
#include "cc1120_reg_config.h"
#include "handle_data.h"
#include "spi.h"

#define CRC_ENABLE          TRUE
#define CRC_INIT            0xFFFF
#define CRC16_POLY          0x8005

#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2


extern bool receivedData(uint8_t rxBuffer[], uint8_t* rxBytes, uint8_t* marcStatus)
{
    if(receivedSync)
    {
        CLRWDT();
        receivedSync = false;
        R_LED = 1;
        // Read number of bytes in RX FIFO
        cc1120SpiReadReg(CC1120_NUM_RXBYTES, rxBytes, 1);
        if(rxBytes != 0)
        {   
            // Read MARCState bits to check for RX FIFO err
            cc1120SpiReadReg(CC1120_MARCSTATE, marcStatus, 1);
            // if marcStatus returns RX FIFO err
            if ((*marcStatus & 0x1F) == CC1120_RX_FIFO_ERROR)
                trxCmdStrobe(CC1120_SFRX);      // Flush RX FIFO
            else
            {
                *rxBytes = 6;
                // Read all bytes from RX FIFO
                cc1120SpiReadRxFifo(rxBuffer, *rxBytes);
                // Flush RX FIFO
                trxCmdStrobe(CC1120_SFRX);
            }
            __delay_ms(20);
            trxCmdStrobe(CC1120_SWORRST);
            __delay_ms(10);
        }
        trxCmdStrobe(CC1120_SWOR);
        R_LED = 0;
        return true;
    }
    else
        return false;
}



/******************************************************************************
* @fn          calcCRC
*
* @brief       Calculates a checksum over the payload, included the seq. number
*              and length byte.
*
* @param       uint8 crcData: The data to perform the CRC-16 operation on
*              uint16 crcReg: The current value of the CRC register. For the 
*                             first byte the value CRC16_INIT should be 
*                             supplied. For each additional byte the value 
*                             returned for the last invocation should be 
*                             supplied
*
* @return      crcReg
*/
uint16_t calcCRC(uint8_t crcData, uint16_t crcReg)
{
    unsigned int i;
    for (i = 0; i < 8; i++)
    {
        if ((uint8_t)((crcReg & 0x8000) >> 8) ^ ((uint8_t)(crcData & 0x80)))
            crcReg = (crcReg << 1) ^ CRC16_POLY;
        else
            crcReg = (crcReg << 1);
        crcData <<= 1;
    }

  return crcReg;
}


/*******************************************************************************
*   @fn         createPacket
*
*   @brief      This function is called before a packet is transmitted. It fills
*               the txBuffer with a packet consisting of a length byte, two
*               bytes packet counter and n random bytes.
*
*               The packet format is as follows:
*               |--------------------------------------------------------------|
*               |           |           |           |         |       |        |
*               | Dev  type | pktCount1 | pktCount0 | rndData |.......| rndData|
*               |           |           |           |         |       |        |
*               |--------------------------------------------------------------|
*                txBuffer[0] txBuffer[1] txBuffer[2]            txBuffer[PKTLEN]
*
*   @param       Pointer to start of txBuffer
*
*   @return      none
*/
extern void createAckPacket(unsigned char txBuffer[]) 
{
    txBuffer[0] = 0x06;
}


//void writePreamble(uint16_t preambleTime)
//{   
//    trxCmdStrobe(CC1120_STX);
//    __delay_ms(100);
//}


/*******************************************************************************
 * @fn          cc112xSpiReadRxFifo
 *
 * @brief       Reads RX FIFO values to pData array
 *
 * input parameters
 *
 * @param       *pData - pointer to data array where RX FIFO bytes are saved
 * @param       len    - number of bytes to read from the RX FIFO
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
rfStatus_t cc1120SpiReadRxFifo(uint8_t *pData, uint8_t len)
{
  rfStatus_t rc;
  rc = trx8BitRegAccess(0x00,CC1120_BURST_RXFIFO, pData, len);
  return (rc);
}


/*******************************************************************************
*   @fn         calibrateRcOsc
*
*   @brief      Calibrates the RC oscillator used for the eWOR timer. When this
*               function is called, WOR_CFG0.RC_PD must be 0
*
*   @param      none
*
*   @return     none
*/
void calibrateRCOsc(void) 
{
    uint8_t temp;

    // Read current register value
    cc1120SpiReadReg(CC1120_WOR_CFG0, &temp,1);

    // Mask register bit fields and write new values
    temp = (uint8_t)((temp & 0xF9) | (0x02 << 1));

    // Write new register value
    cc1120SpiWriteReg(CC1120_WOR_CFG0, &temp,1);

    // Strobe IDLE to calibrate the RCOSC
    trxCmdStrobe(CC1120_SIDLE);

    while((cc1120GetTxStatus() & 0x70) != 0);
    // Disable RC calibration
    temp = (uint8_t)((temp & 0xF9) | (0x00 << 1));
    cc1120SpiWriteReg(CC1120_WOR_CFG0, &temp, 1);
}

/*******************************************************************************
*   @fn         manualCalibration
*
*   @brief      Calibrates radio according to CC112x errata
*
*   @param      none
*
*   @return     none
*/
void manualCalibration(void) {

    unsigned char original_fs_cal2;
    unsigned char calResults_for_vcdac_start_high[3];
    unsigned char calResults_for_vcdac_start_mid[3];
    unsigned char marcstate;
    unsigned char writeByte;

    // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    cc1120SpiWriteReg(CC1120_FS_VCO2, &writeByte, 1);

    // 2) Start with high VCDAC (original VCDAC_START + 2):
    cc1120SpiReadReg(CC1120_FS_CAL2, &original_fs_cal2, 1);
    writeByte = (unsigned char)(original_fs_cal2 + VCDAC_START_OFFSET);
    cc1120SpiWriteReg(CC1120_FS_CAL2, &writeByte, 1);

    // 3) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    trxCmdStrobe(CC1120_SCAL);

    do {
        cc1120SpiReadReg(CC1120_MARCSTATE, &marcstate, 1);
    } while (marcstate != 0x41);

    // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with 
    //    high VCDAC_START value
    cc1120SpiReadReg(CC1120_FS_VCO2,
                     &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
    cc1120SpiReadReg(CC1120_FS_VCO4,
                     &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
    cc1120SpiReadReg(CC1120_FS_CHP,
                     &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);

    // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    cc1120SpiWriteReg(CC1120_FS_VCO2, &writeByte, 1);

    // 6) Continue with mid VCDAC (original VCDAC_START):
    writeByte = original_fs_cal2;
    cc1120SpiWriteReg(CC1120_FS_CAL2, &writeByte, 1);

    // 7) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    trxCmdStrobe(CC1120_SCAL);

    do {
        cc1120SpiReadReg(CC1120_MARCSTATE, &marcstate, 1);
    } while (marcstate != 0x41);

    // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained 
    //    with mid VCDAC_START value
    cc1120SpiReadReg(CC1120_FS_VCO2, 
                     &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
    cc1120SpiReadReg(CC1120_FS_VCO4,
                     &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
    cc1120SpiReadReg(CC1120_FS_CHP,
                     &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);

    // 9) Write back highest FS_VCO2 and corresponding FS_VCO
    //    and FS_CHP result
    if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
        calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
        writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
        cc1120SpiWriteReg(CC1120_FS_VCO2, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
        cc1120SpiWriteReg(CC1120_FS_VCO4, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
        cc1120SpiWriteReg(CC1120_FS_CHP, &writeByte, 1);
    } else {
        writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
        cc1120SpiWriteReg(CC1120_FS_VCO2, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
        cc1120SpiWriteReg(CC1120_FS_VCO4, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
        cc1120SpiWriteReg(CC1120_FS_CHP, &writeByte, 1);
    }
}


/* 7s timer to reset WOR as needed*/
void start_rssi_timer()
{
    t2cnt = 0;
    T2CONbits.T2CKPS = 0b00;//11;    // 64 prescaler
    T2CONbits.T2OUTPS = 0b0000;//1111; // 1:16 postscaler
    PIE1bits.TMR2IE = 1;
    PIR1bits.TMR2IF = 0;
    T2CONbits.TMR2ON = 1;
    
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
}

void cc1120_sleep()
{
    trxCmdStrobe(CC1120_SPWD);
}

void cc1120_idle()
{
    trxCmdStrobe(CC1120_SIDLE);
}

uint8_t cc1120_state(void)
{
  uint8_t state;

  cc1120SpiReadReg(CC1120_MARCSTATE, &state, 1);
  
  return state & 0x1f;
}