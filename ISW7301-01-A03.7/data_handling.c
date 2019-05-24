
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "cc1120_reg_config.h"
#include "uart.h"
#include "spi.h"


bool crcOK(uint8_t *rxBuffer, uint8_t payloadLength)
{
    /* Start with entire packet including CRC, then compute CRC >>> if the
     * resulting computation is 0, then the packet has not been corrupted */
    /* rxBuffer[payloadLength] is MSB of CRC; payloadLength + 1 is LSB */
    uint16_t init_val = (uint16_t)(rxBuffer[payloadLength] << 8) | (uint16_t)(rxBuffer[payloadLength + 1] << 0);
    uint16_t crc_val = 0x0000;
    for (uint8_t i = 0; i < payloadLength; i ++)
    {
        crc_val = calcCRC(rxBuffer[i], crc_val);
    }
    if (crc_val == 0x0000)       // if CRC matches what it should have been
        return true;        // return true
    else
        return false;
}


bool isUniqueTransmission(uint8_t *receiveBuf)
{
    uint8_t j = 0, i = 0;
    uint32_t temp = *receiveBuf++;
    temp <<= 8;
    temp |= *receiveBuf++;
    temp <<= 8;
    temp |= *receiveBuf++;
    temp <<= 8;
    temp |= *receiveBuf++;
    bool msgMatches = false;
    
    // check if it's a test
    test1 = temp & 0x04;
    testMatch = (bool)(test1 == 0x04);
    
    while (!msgMatches && i < endMsgPtr)     // parse all messages already received in buffer, exit if no match
    {
        msgMatches |= (bool)(temp == msgReceived[i++]);
    }
    if (!msgMatches)
    {
        msgReceived[endMsgPtr] = temp;
        
        msgTmrCnt[endMsgPtr] = 0x00;            // Setting at 0x00 renews timer count
    }
    return (bool)(!msgMatches);     // if it matches, "not unique"; if it doesn't match, "is unique"
}