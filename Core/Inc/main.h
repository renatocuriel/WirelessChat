#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

typedef struct {
	char message[300];
	uint16_t len;
} message_t;

#define LOG_ON 0x1
#define RESPONSE 0x2
#define HEARTBEAT 0x3
#define MESSAGE 0x4
#define MY_ADDRESS 0x7F
#define BROADCAST 0xFF
#define MAXBUF 300

void Error_Handler(void);

#define SPIRIT1_GPIO3_Pin GPIO_PIN_7
#define SPIRIT1_GPIO3_GPIO_Port GPIOC
#define SPIRIT1_GPIO3_EXTI_IRQn EXTI9_5_IRQn
#define SPIRIT1_SDN_Pin GPIO_PIN_10
#define SPIRIT1_SDN_GPIO_Port GPIOA
#define SPIRIT1_SPI_CSn_Pin GPIO_PIN_6
#define SPIRIT1_SPI_CSn_GPIO_Port GPIOB


extern char message[MAXBUF]; // for getting message from term
extern char username[21]; // to keep my username
extern uint8_t loggedOn; // flag to know whether or not I've logged on in USART ISR
extern uint16_t msgIndex; // for keeping track of message index in USART ISR


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
