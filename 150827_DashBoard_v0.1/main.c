/* This file has been prepared for Doxygen automatic documentation generation.*/
#include "usart_driver.h"
#include "avr_compiler.h"

#define NUM_BYTES  3

#define USART USARTD0

USART_data_t USART_data;
uint8_t sendArray[NUM_BYTES] = {0x00, 0xff, 0xff};
//, 0xaa, 0xf0};
uint8_t receiveArray[NUM_BYTES];
bool success;

//good luck

ISR(USARTD0_RXC_vect)
{
     USART_RXComplete(&USART_data);
}


ISR(USARTD0_DRE_vect)
{
     USART_DataRegEmpty(&USART_data);
}



int main(void)
{
	
     /* counter variable. */
     uint8_t i;

     /* This PORT setting is only valid to USARTC0 if other USARTs is used a
      * different PORT and/or pins are used. */
     /* PC3 (TXD0) as output. */
     PORTD.DIRSET   = PIN3_bm;
     /* PC2 (RXD0) as input. */
     PORTD.DIRCLR   = PIN2_bm;

     /* Use USARTC0 and initialize buffers. */
     USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);

     /* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
     USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc,
                  USART_PMODE_DISABLED_gc, false);

     /* Enable RXC interrupt. */
     USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

     /* Set Baudrate to 9600 bps:
      * Use the default I/O clock frequency that is 2 MHz.
      * Do not use the baudrate scale factor
      *
      * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
      *                 = 12
      */
     USART_Baudrate_Set(&USART, 8,1); //8,0

     /* Enable both RX and TX. */
     USART_Rx_Enable(USART_data.usart);
     USART_Tx_Enable(USART_data.usart);

     /* Enable PMIC interrupt level low. */
     PMIC.CTRL |= PMIC_LOLVLEX_bm;

     /* Enable global interrupts. */
     sei();
	 // importing client's data
	 while(1){
		 /* Fetch received data as it is received. */
	     i = 0;
	     while (i < NUM_BYTES) {
		 		 //receiveArray[i] = USART_GetChar(USART_data.usart);
					/**/
	             if (USART_RXBufferData_Available(&USART_data)) {
	                     receiveArray[i] = USART_RXBuffer_GetByte(&USART_data);
	                     i++;
	             }/**/
				 i++;
	     }


	     /* Send sendArray. */
	     i = 0;
	     while (i < NUM_BYTES) {
	             bool byteToBuffer;
	             byteToBuffer = USART_TXBuffer_PutByte(&USART_data, receiveArray[i]);
	             if(byteToBuffer){
	                     i++;
	             }
	     }
	 }
     
     
	 
}


