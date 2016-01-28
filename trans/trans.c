//* 송신부*///////////////

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
[출처] XBee 지그비 모듈_Wire(XB24-AWI-001) - 스위치 -> LED (AVRStudio) (! 전자공작) |작성자 키트

