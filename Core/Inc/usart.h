#ifndef INC_USART_H_
#define INC_USART_H_

#define CLK_FREQUENCY 32000000
#define BAUDRATE 861600

extern const char* clear_screen;
extern const char* move_cursor_home;
extern const char* green_text;
extern const char* red_text;
extern const char* reset_text;

void USART_Init(void);
void UART_SendString(const char* str);

#endif /* INC_USART_H_ */
