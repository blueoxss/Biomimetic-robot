//* �۽ź�*///////////////

/*****************************************************
Chip type           : ATmega128     : 16.000000 MHz
*****************************************************/


#define F_CPU 16000000UL  // 16 MHz
#include <avr/io.h>
#include <util/delay.h>

int main(void){
    PORTA=0xFF;        // SW PORT full-up
    UCSR1B=8; UBRR1H=0; UBRR1L=103;
    while(1){
        _delay_ms(1000);
        UDR1=~PINA;
    }
}
[��ó] XBee ���׺� ���_Wire(XB24-AWI-001) - ����ġ -> LED (AVRStudio) (! ���ڰ���) |�ۼ��� ŰƮ

