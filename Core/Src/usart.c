#include "main.h"
#include "usart.h"
#include <stdlib.h>

const char * clear_screen = "\033[2J";
const char * move_cursor_home = "\033[H";
const char * green_text = "\033[32m";
const char * red_text = "\033[31m";
const char * reset_text = "\033[0m";

void USART_Init(void) {
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);    // mask AF selection
	GPIOA->AFR[0] |= ((7 << GPIO_AFRL_AFSEL2_Pos) |             // select USART2 (AF7)
					 (7 << GPIO_AFRL_AFSEL3_Pos));             // for PA2 and PA3

	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);     // enable alternate function
	GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);  // for PA2 and PA3

	// Configure USART2 connected to the debugger virtual COM port
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;            // enable USART by turning on system clock
	USART2->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);     // set data to 8 bits
	USART2->BRR = CLK_FREQUENCY / BAUDRATE;                      // baudrate for 861600
	USART2->CR1 |= USART_CR1_UE;                       // enable USART
	USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);      // enable transmit and receive for USART

	// enable interrupts for USART2 receive
	USART2->CR1 |= USART_CR1_RXNEIE;                   // enable RXNE interrupt on USART2
	USART2->ISR &= ~(USART_ISR_RXNE);                  // clear interrupt flag while (message[i] != 0)

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); // set NVIC Priority Grouping
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)); // set interrupt priorities
    NVIC_EnableIRQ(USART2_IRQn);
}

void UART_SendString(const char* str)
{
    uint16_t i;
    for (i = 0; str[i] != 0; i++) {                // check for terminating NULL character
        while (!(USART2->ISR & USART_ISR_TXE));       // wait for transmit buffer to be empty
        USART2->TDR = str[i];                      // transmit character to USART
    }
}
