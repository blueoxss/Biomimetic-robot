#include "uart.h"



void uart_Init(void){
	USART_Format_Set(&USARTD0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	USART_Baudrate_Set(&USARTD0, 8 , 0);

	USART_Rx_Enable(&USARTD0);
	USART_Tx_Enable(&USARTD0);
}

void uartSendTXbit(unsigned char data){

	do{}while(!USART_IsTXDataRegisterEmpty(&USARTD0));
		USART_PutChar(&USARTD0, data);

}

void uartSendTX(unsigned char *string){
	int i=-1;
	do{
		uartSendTXbit(string[++i]);
	}
	while(string[i] != '\r');
	
}
