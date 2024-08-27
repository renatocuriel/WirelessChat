#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "main.h"
#include "usart.h"
#include "spsgrf.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

extern volatile SpiritFlagStatus xTxDoneFlag;
extern volatile SpiritFlagStatus xRxDoneFlag;

void getUsername(void);
void printUsage(void);
void processPayload(uint8_t srcAddress, uint8_t destAddress, char* payload, uint16_t bytesRecv);
void printPayload(uint8_t srcAddress, uint8_t destAddress, char* payload, uint16_t  bytesRecv);
void processInput(message_t *msg_t);
uint16_t createPDU(uint8_t flag, uint8_t *buf, uint8_t *message, uint16_t len);
void parseCommand(const char *input, uint8_t *number, char *message);

#endif /* INC_UTILS_H_ */
