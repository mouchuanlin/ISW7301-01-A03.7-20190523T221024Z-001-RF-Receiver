/* 
 * File:   handle_data.h
 * Author: Scott
 *
 * Created on February 8, 2018, 12:27 PM
 */

#ifndef HANDLE_DATA_H
#define	HANDLE_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

bool crcOK(uint8_t rxBuffer[], uint8_t payloadLength);
bool isUniqueTransmission(uint8_t *receiveBuf);
void sendAck();

bool receivedSync = false;


#endif	/* HANDLE_DATA_H */

