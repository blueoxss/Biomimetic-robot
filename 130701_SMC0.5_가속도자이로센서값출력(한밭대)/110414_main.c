
#include "spi_driver.h"
#include "usart_driver.h"
#include "avr_compiler.h"

#define USART USARTD0

#define CLKSYS_IsReady( _oscSel ) ( OSC.STATUS & (_oscSel) )

volatile unsigned char Acc_read=0;
static unsigned int XXL_u,YXL_u,ZXL_u,XGL_u,YGL_u,ZGL_u;
static unsigned int timer=0;

//function
int read_sensor_data(unsigned int *xxl,unsigned int *yxl,unsigned int *zxl);
void clk_init(void);
void port_init(void);
void interrupt_init(void);
void Init_L3G4200DH(void);
void Init_LIS331DLH(void);

/*! \brief The number of test data bytes. */
#define NUM_BYTES     7

/* Global variables */

/*! \brief SPI master module on PORT C. */
SPI_Master_t spiMasterC;

/*! \brief SPI slave module on PORT D. */
SPI_Slave_t spiSlaveD;

/*! \brief SPI Data packet */
SPI_DataPacket_t dataPacket;

/*! \brief Test data to send from master. */
uint8_t masterSendData[NUM_BYTES] = {0xe8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/*! \brief Data received from Accelerometer */
uint8_t ACC_DATA[NUM_BYTES];
/*! \brief Data received from Gyroscope */
uint8_t GYRO_DATA[NUM_BYTES];

/*! \brief Result of the test. */
bool success = true;

/* Instantiate pointer to ssPort. */
PORT_t *ssPort = &PORTC;


void clk_init(void){
	OSC.XOSCCTRL=0b11001011;//datasheet p89
	OSC.CTRL=0x08;
	do {} while ( CLKSYS_IsReady( OSC_XOSCRDY_bm ) == 0 );
	//CCP=0x9D;
	CCP=0xD8;
	CLK.CTRL=0x03;

}


void port_init(void){
	
	PORTA.OUT=0x00;
	PORTA.DIR=0x00;

	PORTB.OUT=
	PORTB.DIR=

	PORTC.OUT=0xa3;
	PORTC.DIR= PORTC.DIR | 0xa3;
	
	PORTD.DIRSET = PIN3_bm;
	PORTD.DIRCLR = PIN2_bm;

	PORTE.OUT=
	PORTE.DIR=

	PORTF.OUT=
	PORTF.DIR=
	
	PORTH.OUT=
	PORTH.DIR=
	
	PORTJ.OUT=0x00;
	PORTJ.DIR=0x00;

	
	PORTK.OUT=0x00;
	PORTK.DIR=0x00;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////


void Init_L3G4200DH(void)
{
  unsigned char write_register=0;
  //CTRL_REG1
  //DR1 DR0 BW1 BW0 PD Zen Xen Yen
  //0    0   1   1   1   1   1   1
  //ODR=100Hz
  write_register=0x3f;

	SPI_MasterSSLow(ssPort, PIN1_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x20);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
//    spiSendByte(0x20);
 //   spiSendByte(write_register);
	SPI_MasterSSHigh(ssPort, PIN1_bm);

  //CTRL_REG2
  //0 0 HPM1 HPM1 HPCF3 HPCF2 HPCF1 HPCF0
  //0 0   0   0    0      0     0     0
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN1_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x21);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	SPI_MasterSSHigh(ssPort, PIN1_bm);

  //CTRL_REG3
  //I1_Int1 I1_Boot H_Lactive PP_OD I2_DRDY I2_WTM I2_ORun I2_Empty
  //  0        0        0       0     0       0      0        0 
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN1_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x22);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	SPI_MasterSSHigh(ssPort, PIN1_bm);

  //CTRL_REG4
  //BDU BLE FS1 FS0 - ST1 ST0 SIM
  // 1   0   0   0  0  0   0   0
  write_register=0x80;
	SPI_MasterSSLow(ssPort, PIN1_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x23);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	SPI_MasterSSHigh(ssPort, PIN1_bm);

  //CTRL_REG5
  //BOOT FIFO_EN - HPen INT1_Sel1 INT1_Sel0 Out_Sel1 Out_Sel0
  // 0      0    0  0      0          0        0        0
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN1_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x24);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	SPI_MasterSSHigh(ssPort, PIN1_bm);


}
void Init_LIS331DLH(void)
{
  unsigned char write_register=0;
  //CTRL_REG1
  //PM2 PM1 PM0 DR1 DR0 Zen Yen Xen
  //0    0   1   0   1   1   1   1
  //ODR=100Hz
  write_register=0x2f;
//  write_register=DR
	SPI_MasterSSLow(ssPort, PIN0_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x20);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	SPI_MasterSSHigh(ssPort, PIN0_bm);
  //CTRL_REG2
  //BOOT HPM1 HPM0 FDS HPen2 HPen1 HPCF1 HPCF0
  //0    0   0  0   0          0          00
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x21);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	SPI_MasterSSHigh(ssPort, PIN0_bm);
  //CTRL_REG3
  //IHL PP_OD LIR2 I2_CFG1 I2_CFG0 LIR1 I1_CFG1 I1_CFG0
  //0    0      0    0       0      0      1       0
  write_register=0x02;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x22);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	SPI_MasterSSHigh(ssPort, PIN0_bm);
  //CTRL_REG4
  //BDU BLE FS1 FS0 STsign 0 ST SIM
  //1    0   0   0    0    0  0  0
  write_register=0x80;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x23);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	SPI_MasterSSHigh(ssPort, PIN0_bm);
  //CTRL_REG5
  //0 0 0 0 0 0 TurnOn1 TurnOn0
  //0 0 0 0 0 0    0       0 
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x24);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	SPI_MasterSSHigh(ssPort, PIN0_bm);

}

#if 0
int read_gyro_data(unsigned int *xgl,unsigned int *ygl, unsigned int *zgl){
//int read_gyro_data(int *xgl,int *ygl,int *zgl){
	static unsigned char status=0;
	static unsigned char acc_data[6];

//new data available
/*
  while((status&=0x0f)==0)
  { 

PORTB&=~(_BV(PB1));
	spiSendByte(0xa7);
	status=spiTransferByte(0xFF);
PORTB |= _BV(PB1);
  }
*/

//PORTB&=~(_BV(PB1));//PB1을 0으로 만드는거
//PORTB |= _BV(PB1);//PB1을 1로 만드는거
//sensor data multiple read
PORTC.OUT = PORTC.OUT & 0xfd;//Pc1을 0으로 만드는거
	spiSendByte(0xe8);
	acc_data[0]=spiTransferByte(0xFF);
	acc_data[1]=spiTransferByte(0xFF);
	acc_data[2]=spiTransferByte(0xFF);
	acc_data[3]=spiTransferByte(0xFF);
	acc_data[4]=spiTransferByte(0xFF);
	acc_data[5]=spiTransferByte(0xFF);
	PORTC.OUT = PORTC.OUT | 0x02;//pc1을 1로 만드는거
//check timer
//    timer=TCNT1H*256+TCNT1L;



  acc_data[1]=acc_data[1]+0x80;
  acc_data[3]=acc_data[3]+0x80;
  acc_data[5]=acc_data[5]+0x80;
  *xgl= (unsigned int)acc_data[1]*256+acc_data[0];
  *ygl= (unsigned int)acc_data[3]*256+acc_data[2];
  *zgl= (unsigned int)acc_data[5]*256+acc_data[4];


//  *xgl= (int)acc_data[1]*256+acc_data[0];
//  *ygl= (int)acc_data[3]*256+acc_data[2];
//  *zgl= (int)acc_data[5]*256+acc_data[4];

	return 0;
}

int read_sensor_data(unsigned int *xxl,unsigned int *yxl,unsigned int *zxl){
//int read_sensor_data(int *xxl,int *yxl,int *zxl){

//	static unsigned char status=0;
  static unsigned char acc_data[6];


//check new data available
/*  while((status&=0x0f)==0)
  { 

PORTB&=~(_BV(PB0));
	spiSendByte(0xa7);
	status=spiTransferByte(0xFF);
PORTB |= _BV(PB0);
  }
*/

//sensor data multiple read


	PORTC.OUT = PORTC.OUT & 0xfe;//Pc0을 0으로 만드는거
	spiSendByte(0xe8);
	SPI_MasterTransceiveByte(&spiMasterC, 0xe8);

	SPI_MasterTransceiveByte(&spiMasterC, 0xff);

	acc_data[0]=spiTransferByte(0xFF);
	acc_data[1]=spiTransferByte(0xFF);
	acc_data[2]=spiTransferByte(0xFF);
	acc_data[3]=spiTransferByte(0xFF);
	acc_data[4]=spiTransferByte(0xFF);
	acc_data[5]=spiTransferByte(0xFF);
	PORTC.OUT = PORTC.OUT | 0x01;//pc1을 1로 만드는거




  acc_data[1]=acc_data[1]+0x80;
  acc_data[3]=acc_data[3]+0x80;
  acc_data[5]=acc_data[5]+0x80;
  *xxl= (unsigned int)acc_data[1]*256+acc_data[0];
  *yxl= (unsigned int)acc_data[3]*256+acc_data[2];
  *zxl= (unsigned int)acc_data[5]*256+acc_data[4];
//  *xxl= (int)acc_data[1]*256+acc_data[0];
//  *yxl= (int)acc_data[3]*256+acc_data[2];
//  *zxl= (int)acc_data[5]*256+acc_data[4];

	return 0;
}

#endif

/*int main(void)
{
	

	clk_init();
	port_init();
	uart_Init();
	spi_init();
	unsigned char XYZ_buffer[100];
	
	Init_LIS331DLH();
  	Init_L3G4200DH();
	
	while(1)
	{
		
		read_gyro_data(&XGL_u, &YGL_u, &ZGL_u);
		read_sensor_data(&XXL_u, &YXL_u, &ZXL_u);

		sprintf((char*)XYZ_buffer,"[XXL%6u][YXL%6u][ZXL%6u][XGY%6u][YGY%6u][ZGY%6u]\r\n",XXL_u,YXL_u,ZXL_u,XGL_u,YGL_u,ZGL_u);
		//uartSendTXbit('3');
		//sprintf((char*)XYZ_buffer,"abcdefgefgh12341\n\r");	
		uartSendTX(XYZ_buffer);
	}


}*/

void USART_INIT(void)
{

	/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins is used. */
	/* PIN3 (TXD0) as output. */
	PORTD.DIRSET = PIN3_bm;

	/* PC2 (RXD0) as input. */
	PORTD.DIRCLR = PIN2_bm;

	/* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USARTD0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);


	/* Set Baudrate to 115200 bps:
	 * Use the default I/O clock fequency that is 16 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USARTD0, 8 , 0);



	/* Enable both RX and TX. */
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


int main(void)
{
	unsigned int XXL,YXL,ZXL,XGL,YGL,ZGL;
	unsigned char XYZ_buffer[100];
	clk_init();
	port_init();
 	USART_INIT();
//	spi_init();


	/* Init SS pin as output with wired AND and pull-up. */
	PORTC.DIRSET = PIN0_bm;
	PORTC.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
	PORTC.DIRSET = PIN1_bm;
	PORTC.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;

	/* Set SS output to high. (No slave addressed). */
	PORTC.OUTSET = PIN0_bm;
	PORTC.OUTSET = PIN1_bm;

	/* Instantiate pointer to ssPort. */
	PORT_t *ssPort = &PORTC;

	/* Initialize SPI master on port C. */
	SPI_MasterInit(&spiMasterC,
	               &SPIC,
	               &PORTC,
	               false,
	               SPI_MODE_3_gc,
	               SPI_INTLVL_OFF_gc,
	               false,
	               SPI_PRESCALER_DIV16_gc);

	/* Initialize ACC & Gyro */
	Init_L3G4200DH();
	Init_LIS331DLH();

	/* Read Sensor data */

	while(true) {
	/* Create data packet (SS to slave by PC0). */
	SPI_MasterCreateDataPacket(&dataPacket,
	                           masterSendData,
	                           ACC_DATA,
	                           NUM_BYTES,
	                           &PORTC,
	                           PIN0_bm);


	/* MASTER: Pull SS line low. This has to be done since
	 *         SPI_MasterTransceiveByte() does not control the SS line(s). */
	SPI_MasterSSLow(ssPort, PIN0_bm);
	/* Transceive packet. */
	SPI_MasterTransceivePacket(&spiMasterC, &dataPacket);
	/* MASTER: Release SS to slave. */
	SPI_MasterSSHigh(ssPort, PIN0_bm);


	/* Create data packet (SS to slave by PC1). */
	SPI_MasterCreateDataPacket(&dataPacket,
	                           masterSendData,
	                           GYRO_DATA,
	                           NUM_BYTES,
	                           &PORTC,
	                           PIN1_bm);

	/* MASTER: Pull SS line low. This has to be done since
	 *         SPI_MasterTransceiveByte() does not control the SS line(s). */
	SPI_MasterSSLow(ssPort, PIN1_bm);
	/* Transceive packet. */
	SPI_MasterTransceivePacket(&spiMasterC, &dataPacket);
	/* MASTER: Release SS to slave. */
	SPI_MasterSSHigh(ssPort, PIN1_bm);

	  ACC_DATA[2]=ACC_DATA[2]+0x80;
	  ACC_DATA[4]=ACC_DATA[4]+0x80;
	  ACC_DATA[6]=ACC_DATA[6]+0x80;
	  XXL= (unsigned int)ACC_DATA[2]*256+ACC_DATA[1];
	  YXL= (unsigned int)ACC_DATA[4]*256+ACC_DATA[3];
	  ZXL= (unsigned int)ACC_DATA[6]*256+ACC_DATA[5];

	  GYRO_DATA[2]=GYRO_DATA[2]+0x80;
	  GYRO_DATA[4]=GYRO_DATA[4]+0x80;
	  GYRO_DATA[6]=GYRO_DATA[6]+0x80;
	  XGL= (unsigned int)GYRO_DATA[2]*256+GYRO_DATA[1];
	  YGL= (unsigned int)GYRO_DATA[4]*256+GYRO_DATA[3];
	  ZGL= (unsigned int)GYRO_DATA[6]*256+GYRO_DATA[5];

		sprintf((char*)XYZ_buffer,"[XXL%6u][YXL%6u][ZXL%6u][XGY%6u][YGY%6u][ZGY%6u]\r\n",XXL,YXL,ZXL,XGL,YGL,ZGL);
		//uartSendTXbit('3');
		//sprintf((char*)XYZ_buffer,"abcdefgefgh12341\n\r");	
		uartSendTX(XYZ_buffer);
	}
	while(true) {
		nop();
	}
}
