
// global AVRLIB defines

#include "avr_compiler.h"

void spi_init(void);

void spiSendByte(char data);
char spiTransferByte(char data);
