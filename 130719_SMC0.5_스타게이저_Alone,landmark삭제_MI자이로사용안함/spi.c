#include "spi.h"

void spi_init(void)
{
	// setup SPI I/O pins
	// clk/
    // 16000000/2 Mhz
    // CLK2X ENABLE DORD MASTEFR MODE[1:0] PRESCALER[1:0]
    // 1	  1		0		1		11		00 
    SPIC.CTRL = 0xdc;
    // clear status
    SPIC.STATUS = 0x00;
}

void spiSendByte(char data)
{
    // send a byte over SPI and ignore reply
	SPIC.DATA = data;
	while( !(SPIC.STATUS & (1<<0x80)));//0x80 is SPIF
}

char spiTransferByte(char data)
{
    // send the given data
    SPIC.DATA = data;

    // wait for transfer to complete
    while( (SPIC.STATUS & (1<<0x80))==0);//0x80 is SPIF
 
    return SPIC.DATA;
}
