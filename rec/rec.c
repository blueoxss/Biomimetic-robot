#define F_CPU 16000000UL  // 16 MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

ISR(USART1_RX_vect){ 
	PORTA=UDR1;
}

int main(void){
	PORTF_DIR = 1;

	while(1){
		PORTF_OUTSET = 0x0F;
	}
	return 0;

}

int main(void){
    UCSR1B=0x90; UBRR1H=0; UBRR1L=103;
    sei();
    while (1){}
}
