#include "spi_driver.h"
#include "usart_driver.h"
#include "avr_compiler.h"
#include <math.h>
#include <string.h>
#include <stdio.h>


//#define USART USARTD0

#define CLKSYS_IsReady( _oscSel ) ( OSC.STATUS & (_oscSel) )

#define OKR_ONLY 0

double enc_check=0;

double TJX=0;
double TJY=0;
double TJA=0;
double TJZ=0;
double TJW=0;

double Z_x=0;
double Z_y=0;
double Z_a=0;

unsigned int vision_X=0;
unsigned int vision_Y=0;
unsigned int vision_angle=0;

double vision_init_flag=0;

double Vacc[2]={0,0};

double Vacc_x=0;
double Vacc_y=0;
double Venc_x=0;
double Venc_x_left=0;		
double Venc_x_right=0;


#define DELAY_COMP_G 10




#define distance_th 3.5

#define SCALE_XXL	0.0006125//0.000598//-0.0098//-0.0006125			// FS=00      1/16 mg/digit,0.0098/16(m/s^2)/digit
#define SCALE_YXL	0.0006125//0.000598//-0.0098//-0.0006125
#define SCALE_ZXL	0.0006125//0.000598//0.0098//0.0006125
//1axis analog gyro
/*
#define SCALE_XGY	0.000146764//0.00015271631//8.75mDegree/sec*digit//-0.001214142//1/14.375*pi/180
#define SCALE_YGY	0.000146764//0.00015271631//0.001214142
#define SCALE_ZGY	0.004697979//0.26977144에서 라디안변환//0.2691744//0.26868588//0.12988//0.065104166666
*/
#define SCALE_XGY	0.000146764//0.00015271631//8.75mDegree/sec*digit//-0.001214142//1/14.375*pi/180
#define SCALE_YGY	0.000146764//0.00015271631//0.001214142
#define SCALE_ZGY	0.00015271631//0.000146764*1.013//*1.025//0.00015271631//0.001214142  1.04
#define SCALE_ZGY_NEG	0.059437323//0.065104166666//0.03255208//0.12988//0.000146764*1.013//*1.025//0.00015271631//0.001214142  1.04
#define SCALE_ZGY_POS	0.067379659//0.065104166666//0.03255208//0.12988//0.000146764*1.013//*1.025//0.00015271631//0.001214142  1.04
#define GRAVITY_COUNT	16000.0

#define SENSOR_COUNT	0.000008

#define CAM_CAL_SLP	0.774	
#define CAM_CAL_INT 0.127


#define PI	3.141592654
#define SIXTEENBITS 65536//104448.0
#define ENCODER_CNT_REV 80865
#define REDUCTION_ENCODER            128000  // ticks/m
#define WHEEL_DIAMETER 0.195
#define WHEEL_BASE	0.331527913


// 영상 관련 MACRO
 
#define MOTOR_ENCODER 7168.0

#define Tsample 100
#define Bsample 1000



#define  TV_NORM 3.0000


volatile unsigned char Acc_read=0;
//static unsigned int XXL_u,YXL_u,ZXL_u,XGY_u,YGY_u,ZGY_u;
//static unsigned int timer=0;
//

	double m_biasXXL;
	double m_biasYXL;
	double m_biasZXL;
	double m_biasXGY;
	double m_biasYGY;
	double m_biasZGY;
	double m_biasXXL0g=0;
	double m_biasYXL0g=0;
	double m_biasZXL0g=0;

	double T;


char host_string[300];
double gravityVect[3];

	int int_temp=0;

double m_dLeftInc=0;
unsigned int m_LeftEncoder=0;
unsigned int m_prevDLeft=0;
double  m_dRightInc=0;
unsigned int  m_RightEncoder=0;
unsigned int  m_prevDRight=0;
double m_distance=2;
int m_visionUpdateFlag=0;

//function
int read_sensor_data(unsigned int *xxl,unsigned int *yxl,unsigned int *zxl);
void clk_init(void);
void port_init(void);
void interrupt_init(void);
void Init_L3G4200DH(void);
void Init_LIS3DH(void);
void timer_Init(void);
void int_init(void);

int timeDelay=10;

/*! \brief The number of test data bytes. */
#define NUM_BYTES     7


int g_encoder_flag=1;

double m_XGY_buf[DELAY_COMP_G];
double m_YGY_buf[DELAY_COMP_G];
double m_ZGY_buf[DELAY_COMP_G];

double m_XXL_buf[DELAY_COMP_G];
double m_YXL_buf[DELAY_COMP_G];
double m_ZXL_buf[DELAY_COMP_G];


double m_enc_buf[DELAY_COMP_G];
double m_enc_buf_left[DELAY_COMP_G];
double m_enc_buf_right[DELAY_COMP_G];

double m_enc_timebuf[DELAY_COMP_G];
double m_T_buf[DELAY_COMP_G];

int robotState_buf[DELAY_COMP_G];




double m_enc_sum=0;

int indexD=0;
int robotState=0;

int indexG=0;	
int indexO=0;

///gyro filter
#define Hys_TH	0.0174532925199432*1


double 	m_ZGY_F[10]={0,0,0,0,0,0,0,0,0,0};
double  m_ZGY_filter;
double	m_ZGY_F_sum;
double	gyro_test;
double	z_deg_filter;
int	g_cnt;
int	f_cnt;


/* Global variables */

/*! \brief SPI master module on PORT D. */
SPI_Master_t spiMasterD;

/*! \brief SPI slave module on PORT D. */
SPI_Slave_t spiSlaveD;

/*! \brief SPI Data packet */
SPI_DataPacket_t dataPacket;

/*! \brief Test data to send from master. */
uint8_t masterSendData[NUM_BYTES] = {0xe8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

uint8_t masterSendData_adc[NUM_BYTES] = {0xc8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};//adc

uint8_t masterSendData_gyro[NUM_BYTES] = {0xe8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t masterSendData_gyro_init[NUM_BYTES] ={0x60, 0x3f, 0x00, 0x00, 0x80, 0x00, 0x00}; 
//uint8_t masterSendData_vision[4] = {0x00, 0x01, 0x02, 0x03};
uint8_t masterSendData_vision[4] = {0x00, 0x01, 0x02, 0x03};

/*! \brief Data received from Accelerometer */
uint8_t ACC_DATA[NUM_BYTES];
/*! \brief Data received from Gyroscope */
uint8_t GYRO_DATA[NUM_BYTES];
/*! \brief Data received from Vision Board */
uint8_t VISION_DATA[4];

uint8_t ACC_ADC[NUM_BYTES];

/*! \brief Result of the test. */
bool success = true;

/* Instantiate pointer to ssPort. */
PORT_t *ssPort = &PORTD;

/*! USART data struct used in example. */
USART_data_t USARTC0_data;

USART_data_t USARTD0_data;

USART_data_t USARTE0_data;


/*! Array to put received data in. */
uint8_t receiveArray[NUM_BYTES];

int RobotStateEstimation();

void clk_init(void){
	OSC.XOSCCTRL=0b11001011;//datasheet p89
	OSC.CTRL=0x08;
	do {} while ( CLKSYS_IsReady( OSC_XOSCRDY_bm ) == 0 );
	//CCP=0x9D;
	CCP=0xD8;
	CLK.CTRL=0x03;

}
/*
void clk_init(void){
	CLK.CTRL=0x01;
	CLK.PSCTRL=0x01;

	OSC.CTRL=0x02;



}
*/
volatile int samplingFlag=0;

int sfcnt=0;
ISR(TCC1_OVF_vect)
{
	
	samplingFlag=1;
//	sei();
}
ISR(USARTC0_RXC_vect)
{
//	if(!USART_RXBufferData_Available(&USARTD0_data))
//	{
//		USART_InterruptDriver_Initialize(&USARTD0_data, &USARTD0, USART_DREINTLVL_LO_gc);
//	}
//
	USART_RXComplete(&USARTC0_data);
}

ISR(USARTD0_RXC_vect)
{
//	if(!USART_RXBufferData_Available(&USARTD0_data))
//	{
//		USART_InterruptDriver_Initialize(&USARTD0_data, &USARTD0, USART_DREINTLVL_LO_gc);
//	}
//
	USART_RXComplete(&USARTD0_data);
}

ISR(USARTE0_RXC_vect)
{
//	if(!USART_RXBufferData_Available(&USARTD0_data))
//	{
//		USART_InterruptDriver_Initialize(&USARTD0_data, &USARTD0, USART_DREINTLVL_LO_gc);
//	}
//
	USART_RXComplete(&USARTE0_data);
}


void timer_Init(void){
	
	//TCC0.CTRLA=0x04;//prescaler : clk/8
	TCC0.CTRLA=0x06;//prescaler : clk/256
	TCC0.CTRLB=0x00;
	TCC0.CTRLC=0x00;
	TCC0.CTRLD=0x00;
	TCC0.CTRLE=0x00;
	//TCC0.INTCTRLA=0x00;
	//TCC0.INTCTRLB=0xFF;//interrupt 관련 부분, 다시 체크할것
	//TCC0.INTFLAGS=0x01;//마지막 bit가 OVFIF: Overflow/Underflow Interrupt Flag
	TCC0.PERH=0xFF;//PER 가 period를 조절할 수 있는 부분임
	TCC0.PERL=0xFF;

	TCC1.CTRLA=0x03;//prescaler : clk/4
	//TCC1.CTRLA=0x04;//prescaler : clk/4
	TCC1.CTRLB=0x00;
	TCC1.CTRLC=0x00;
	TCC1.CTRLD=0x00;
	TCC1.CTRLE=0x00;
	TCC1.INTCTRLA=0x02;
	TCC1.INTCTRLB=0x00;//interrupt 관련 부분, 다시 체크할것
	TCC1.INTFLAGS=0x01;//마지막 bit가 OVFIF: Overflow/Underflow Interrupt Flag

//	TCC1.PERH=0x90;
//	TCC1.PERL=0xAD;	//108Hz
	TCC1.PERH=0b10011100;
	TCC1.PERL=0b01000000; //100Hz
	//TCC1.PERH=0b01101000;//PER 가 period를 조절할 수 있는 부분임
	//TCC1.PERL=0b00101011; //150Hz

	
}
void int_init(void){
	
	
	PMIC.INTPRI=0x00;
	PMIC.CTRL=0x07;//High level interrupt enable	
}


void port_init(void){
	
//	PORTA.OUT=0x04;
//	PORTA.DIR=0xea;

	PORTA.OUT=0x04;
	PORTA.DIR=0x44;
	
	PORTB.OUT=PORTB.OUT|0x04;
	PORTB.DIR=PORTB.DIR|0x04;

//	PORTC.OUT=0xa7;
//	PORTC.DIR= PORTC.DIR | 0xa7;

	PORTC.OUT=PORTC.OUT|0x01;
	PORTC.DIR= 0xb9;
	
	PORTD.DIRSET = PIN3_bm;
	PORTD.DIRCLR = PIN2_bm;

/*	PORTE.OUT=
	PORTE.DIR=

	PORTF.OUT=
	PORTF.DIR=
	
	PORTH.OUT=
	PORTH.DIR=
*/	

}


/////////////////////////////////////////////////////////////////////////////////////////////////////

/*
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
	SPI_MasterTransceiveByte(&spiMasterC, 0x3f);
//    spiSendByte(0x20);
 //   spiSendByte(write_register);
	SPI_MasterSSHigh(ssPort, PIN1_bm);

  //CTRL_REG2
  //0 0 HPM1 HPM1 HPCF3 HPCF2 HPCF1 HPCF0
  //0 0   0   0    0      0     0     0
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN1_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x21);
	SPI_MasterTransceiveByte(&spiMasterC, 0x00);
	SPI_MasterSSHigh(ssPort, PIN1_bm);

  //CTRL_REG3
  //I1_Int1 I1_Boot H_Lactive PP_OD I2_DRDY I2_WTM I2_ORun I2_Empty
  //  0        0        0       0     0       0      0        0 
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN1_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x22);
	SPI_MasterTransceiveByte(&spiMasterC, 0x00);
	SPI_MasterSSHigh(ssPort, PIN1_bm);

  //CTRL_REG4
  //BDU BLE FS1 FS0 - ST1 ST0 SIM
  // 1   0   0   0  0  0   0   0
  write_register=0x80;
	SPI_MasterSSLow(ssPort, PIN1_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x23);
	SPI_MasterTransceiveByte(&spiMasterC, 0x80);
	SPI_MasterSSHigh(ssPort, PIN1_bm);

  //CTRL_REG5
  //BOOT FIFO_EN - HPen INT1_Sel1 INT1_Sel0 Out_Sel1 Out_Sel0
  // 0      0    0  0      0          0        0        0
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN1_bm);
	SPI_MasterTransceiveByte(&spiMasterC, 0x24);
	SPI_MasterTransceiveByte(&spiMasterC, 0x00);
	SPI_MasterSSHigh(ssPort, PIN1_bm);


}
*/
void Init_LIS3DH(void)
{
  unsigned char write_register=0;
  //CTRL_REG1
  //ODR3 ODR2 ODR1 ODR0 LPen Zen Yen Xen
  //  0    1    0    1   0   1   1   1
  //ODR=100Hz
 	 write_register=0x57;
//  write_register=DR
	SPI_MasterSSLow(ssPort, PIN0_bm);
	_delay_us(10);
	SPI_MasterTransceiveByte(&spiMasterD, 0x20);
	SPI_MasterTransceiveByte(&spiMasterD, write_register);
	_delay_us(10);
	SPI_MasterSSHigh(ssPort, PIN0_bm);
	_delay_us(10);
  //CTRL_REG2
  //HPM1 HPM0 HPCF2 HPCF1 FDS HPCLICK HPIS2 HPIS1
  // 0    0     0     0    0     0       0    0
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	_delay_us(10);
	SPI_MasterTransceiveByte(&spiMasterD, 0x21);
	SPI_MasterTransceiveByte(&spiMasterD, write_register);
	_delay_us(10);
	SPI_MasterSSHigh(ssPort, PIN0_bm);
	_delay_us(10);
  //CTRL_REG3
  //I1_CLICK I1_AOI1 I1_AOI2 I1_DRDY1 I1_DRDY2 I1_WTM I1_OVERRUN   --
  //	0       0        0       0       0        0       1         0
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	_delay_us(10);
	SPI_MasterTransceiveByte(&spiMasterD, 0x22);
	SPI_MasterTransceiveByte(&spiMasterD, write_register);
	_delay_us(10);
	SPI_MasterSSHigh(ssPort, PIN0_bm);
	_delay_us(10);
  //CTRL_REG4
  //BDU BLE FS1 FS0   HR  ST1 ST0 SIM
  //1    0   0   0    1    0   0   0
  write_register=0x88;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	_delay_us(10);
	SPI_MasterTransceiveByte(&spiMasterD, 0x23);
	SPI_MasterTransceiveByte(&spiMasterD, write_register);
	_delay_us(10);
	SPI_MasterSSHigh(ssPort, PIN0_bm);
	_delay_us(10);
  //CTRL_REG5
  //BOOT FIFO_EN -- -- LIR_INT1 D4D_INT1 0 0
  //0       0     0  0    0       0      0 0 
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	_delay_us(10);
	SPI_MasterTransceiveByte(&spiMasterD, 0x24);
	SPI_MasterTransceiveByte(&spiMasterD, write_register);
	_delay_us(10);
	SPI_MasterSSHigh(ssPort, PIN0_bm);
	//CTRL_REG6
  //I2_CLICKen I2_INT1 0 BOOT_I1 0 - - H_LACTIVE -
  //	0        0     0    0    0  0      0     0 
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	_delay_us(10);
	SPI_MasterTransceiveByte(&spiMasterD, 0x25);
	SPI_MasterTransceiveByte(&spiMasterD, write_register);
	_delay_us(10);
	SPI_MasterSSHigh(ssPort, PIN0_bm);

 write_register=0xC0;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	_delay_us(10);
	SPI_MasterTransceiveByte(&spiMasterD, 0x1F);
	SPI_MasterTransceiveByte(&spiMasterD, write_register);
	_delay_us(10);
	SPI_MasterSSHigh(ssPort, PIN0_bm);


}




void USART_INIT(void)
{
	/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins is used. */
	/* PIN3 (TXC0) as output. */
	PORTC.DIRSET = PIN3_bm;

	/* PC2 (RXC0) as input. */
	PORTC.DIRCLR = PIN2_bm;

	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USARTC0_data, &USARTC0, USART_DREINTLVL_LO_gc);

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USARTC0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USARTC0_data.usart, USART_RXCINTLVL_HI_gc);

	/* Set Baudrate to 115200 bps:
	 * Use the default I/O clock fequency that is 16 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
//	USART_Baudrate_Set(&USARTC0, 8 , 0);

	USART_Baudrate_Set(&USARTC0, 16 , 1);		// 57600
//	USART_Baudrate_Set(&USARTC0, 16 , 0);		// 115200
	USARTC0.CTRLB|=0x04;                      //CLK2X


	/* Enable both RX and TX. */
	USART_Rx_Enable(&USARTC0);
	USART_Tx_Enable(&USARTC0);


	/* This PORT setting is only valid to USARTD0 if other USARTs is used a
	 * different PORT and/or pins is used. */
	/* PIN3 (TXD0) as output. */
	PORTD.DIRSET = PIN3_bm;

	/* PC2 (RXD0) as input. */
	PORTD.DIRCLR = PIN2_bm;

	/* Use USARTD0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USARTD0_data, &USARTD0, USART_DREINTLVL_LO_gc);

	/* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USARTD0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USARTD0_data.usart, USART_RXCINTLVL_HI_gc);

	/* Set Baudrate to 115200 bps:
	 * Use the default I/O clock fequency that is 16 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
//	USART_Baudrate_Set(&USARTC0, 8 , 0);

//	USART_Baudrate_Set(&USARTD0, 16 , 1);		// 57600
	USART_Baudrate_Set(&USARTD0, 16, 0);		// 115200, 하기소닉 x,y,theta 받을때의 baudrate, 스타게이저 
	USARTD0.CTRLB|=0x04;                      //CLK2X


	/* Enable both RX and TX. */
	USART_Rx_Enable(&USARTD0);
	USART_Tx_Enable(&USARTD0);

	/* Enable PMIC interrupt level low. */
//	PMIC.CTRL |= PMIC_LOLVLEX_bm;



	/* This PORT setting is only valid to USARTE0 if other USARTs is used a
	 * different PORT and/or pins is used. */
	/* PIN3 (TXE0) as output. */
	PORTE.DIRSET = PIN3_bm;

	/* PC2 (RXE0) as input. */
	PORTE.DIRCLR = PIN2_bm;

	/* Use USARTE0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USARTE0_data, &USARTE0, USART_DREINTLVL_LO_gc);

	/* USARTE0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USARTE0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USARTE0_data.usart, USART_RXCINTLVL_LO_gc);

	//USART_Baudrate_Set(&USARTE0, 16 , 1);		// 57600, 하기소닉 보드에 encoder 값 넣을 때의 baudrate
	USART_Baudrate_Set(&USARTE0, 16, 0);		// 115200, MI 보드에서  gyro 값 받을 때의 baudrate
	USARTE0.CTRLB|=0x04;

	/* Enable both RX and TX. */
	USART_Rx_Enable(&USARTE0);
	USART_Tx_Enable(&USARTE0);

	/* Enable PMIC interrupt level low. */
//	PMIC.CTRL |= PMIC_LOLVLEX_bm;



}

void uartC0SendTXbit(unsigned char data){

	do{}while(!USART_IsTXDataRegisterEmpty(&USARTC0));
		USART_PutChar(&USARTC0, data);

}

void uartC0SendTX(unsigned char *string){
	int i=-1;
	do{
		uartC0SendTXbit(string[++i]);
	}
	while(string[i] != '\0');
	
}

void uartD0SendTXbit(unsigned char data){

	do{}while(!USART_IsTXDataRegisterEmpty(&USARTD0));
		USART_PutChar(&USARTD0, data);

}

void uartD0SendTX(unsigned char *string){
	int i=-1;
	do{
		uartD0SendTXbit(string[++i]);
	}
	while(string[i] != '\0');
	
}

void uartE0SendTXbit(unsigned char data){

	do{}while(!USART_IsTXDataRegisterEmpty(&USARTE0));
		USART_PutChar(&USARTE0, data);

}

void uartE0SendTX(unsigned char *string){
	int i=-1;
	do{
		uartE0SendTXbit(string[++i]);
	}
	while(string[i] != '\0');
	
}


unsigned int lt,rt;


////hagisonic_module communication protocol
char hagisonic_string[300];
int H_angle=0, H_x=0, H_y=0, Prev_H_angle=0, Prev_H_x=0, Prev_H_y=0;
int Encoder_angle=0, Encoder_x=0, Encoder_y=0,Prev_Encoder_angle=0, Prev_Encoder_x=0, Prev_Encoder_y=0;
 double H_angle_deg;
 double Encoder_angle_deg;
unsigned int start_bit, end_bit;
unsigned int uart_test;
int test=0;
int data_cnt=0;
int hagisonic_cnt=0;
unsigned int i_cnt=0;
char XYZ_buffer[100];
char XXX_buffer[100];

char Encoder_diff_Left;
char Encoder_diff_Right;


char 	test_enL=0x00, test_enR=0x00;
char	H_encoderL=0, H_encoderR=0;
char	H_encoderL_BM=0, H_encoderR_BM=0;


double	H_encoderL_sum=0, H_encoderR_sum=0;


double H_angle_deg;
double Encoder_angle_deg;



//스타게이저로부터 데이터 받는 함수
typedef struct landmark{

int id;
double x;
double y;
double theta;
int init_flag;
int correct_flag;
int average_cnt;


struct landmark* next;

}landmark;

unsigned int stg_string_length=0,stg_string_length_temp=0;
//char * keyword_pt, *length_pt, *string_pt;
//char * pch;
int stg_cnt=0;

int stg_id =0;
double stg_angle=0, stg_x=0, stg_y=0, stg_z=0;
double stg_angle_prev=0, stg_x_prev=0, stg_y_prev=0;
char stg_string[300]={0,};
int stg_first=1, stg_angle_init=0;
landmark* Reference;
int landmark_known_flag = 0;
double new_landmark_angle=0, new_landmark_X=0, new_landmark_Y=0;
int new_landmark_cnt = 0;
double TJA_prev=0, TJX_prev=0, TJY_prev=0, TJW_prev=0, TJZ_prev=0;

landmark *temp_a;


int test_cnt=0;
#define landmark_num 2
//int landmark_DB[landmark_num]={28486, 28498, 28502,28528,28512,28496};	//407호 6개
//int landmark_DB[landmark_num]={28486, 28498, 28512,28528};	//407호 4개
int landmark_DB[landmark_num]={28514, 28516};		//409호 2개, 대각선
//int landmark_DB[landmark_num]={28514, 28500};		//409호 2개, 직선
//int landmark_DB[landmark_num]={28514};

int landmark_id_check(int stg_id)
{
	for(int i=0 ; i < landmark_num ; i++)
	{
		if(landmark_DB[i]==stg_id)	return 1;
		
		
	}
	return 0;
}
 
void stg_module()
{	
	
	unsigned int stg_string_length=0;
	char * keyword_pt;
	char * pch;
	
	static int stg_i=0;
	static int stg_coord_init=1;
	int i=0;
	double current_coord_theta=0, current_coord_x=0, current_coord_y=0;	
	double x_prime, y_prime;
	
	
	

	while (USART_RXBufferData_Available(&USARTD0_data))
	{
		stg_string[stg_i] = USART_RXBuffer_GetByte(&USARTD0_data);
		stg_i++;
		
		
	}	
	stg_string[stg_i]='\0';
	keyword_pt = stg_string;
	stg_string_length=strlen(keyword_pt);	
	
	if(20 < stg_string_length && stg_string_length < 40 && *keyword_pt == '~' && *(keyword_pt+1) == '^' && *(keyword_pt+2) == 'I')
	{			
	
		pch = keyword_pt;
			
		pch=strchr(keyword_pt,'I');		// 알파벳 'I' 찾기 
		stg_id = atoi(pch+1);
				

		landmark_known_flag = 0;
		if(landmark_id_check(stg_id))
		{
			pch = strchr(pch+1,'|');	// 특수문자 '|'(Shift + \) 찾기
			stg_angle = -1 * atof(pch+1);

			pch = strchr(pch+1,'|');
			stg_x = atof(pch+1)/100.0;

			pch = strchr(pch+1,'|');
			stg_y = atof(pch+1)/100.0;

			pch = strchr(pch+1,'|');
			stg_z = atof(pch+1)/100.0;
		
			
			landmark_known_flag=2;
			
			
			if(stg_coord_init == 1)//Reference 랜드마크 등록 - 한번만 실행됨.
			{
				Reference = (landmark*)malloc(sizeof(landmark));
				Reference->id = stg_id;
				Reference->x = 0;
				Reference->y = 0;
				Reference->theta = 0;
				Reference->correct_flag = 0;
				Reference->average_cnt = 0;			
				Reference->next = NULL;
		
				stg_coord_init++;
			}			

			if(stg_id == Reference->id)		// Reference 랜드마크일 경우, 좌표 변환 없이 그대로 출력
			{		
				landmark_known_flag = 1; 
				if( sqrt(stg_x*stg_x+stg_y*stg_y) < 1.3 )	m_visionUpdateFlag=1;
				//m_visionUpdateFlag=1;
				
			}			
			else
			{
				landmark* a = Reference;
				
		
				for(; a->next != NULL; a = a->next){	//DB에 등록되어 있는 랜드마크인지 아닌지 확인 
					if( a->id == stg_id )
					{	
						
						if(a->init_flag == 0)		//DB에 랜드마크 ID만 등록되어 있으며, 랜드마크 좌표의 초기화가 되지 않음.
						{
							
							landmark_known_flag = 3;	//3의 의미: DB에 랜드마크가 추가는 됐지만 아직 초기화가 안됨
							new_landmark_angle = TJA - stg_angle*PI/180.0;		//radian
			
							x_prime = cos(new_landmark_angle)*stg_x - sin(new_landmark_angle)*stg_y;
							y_prime = sin(new_landmark_angle)*stg_x + cos(new_landmark_angle)*stg_y;
			
							new_landmark_X = TJX - x_prime;
							new_landmark_Y = TJY - y_prime;
							
							
							a->theta = (a->theta*a->average_cnt + new_landmark_angle*180/PI)/(a->average_cnt+1);		//degree
							a->x = (a->x*a->average_cnt + new_landmark_X)/(a->average_cnt+1);
							a->y = (a->y*a->average_cnt + new_landmark_Y)/(a->average_cnt+1);
							a->average_cnt = a->average_cnt + 1;
							if(a->average_cnt == 20) 	
							{	
								a->init_flag = 1;
								a->correct_flag = 1;	
							}

							break;

						}
						else	//현재 랜드마크가 DB에 랜드마크가 등록된 경우, DB에 등록된 랜드마크 좌표 사용
						{
							current_coord_theta = a->theta;
							current_coord_x = a->x;
							current_coord_y = a->y;

							landmark_known_flag = 1;	//1의 의미: DB에 랜드마크 사용
							if( sqrt(stg_x*stg_x+stg_y*stg_y) < 1.3 )	
							{	m_visionUpdateFlag=1;
								//temp_a = a;
							}
							//m_visionUpdateFlag=1;
							break;	
						}
					}				
				}
				
				if(landmark_known_flag == 2)
				{
					if( a->id == stg_id )	//위의 for문이 DB의 마지막 랜드마크를 확인하지 못하기때문에 
					{						//현재 랜드마크가 DB의 마지막 랜드마크인지 체크하는 구문
						
						if(a->init_flag == 0)
						{
							
							landmark_known_flag = 3;	//3의 의미: DB에 랜드마크가 추가는 됐지만 아직 초기화가 안됨
							new_landmark_angle = TJA - stg_angle*PI/180.0;		//radian
			
							x_prime = cos(new_landmark_angle)*stg_x - sin(new_landmark_angle)*stg_y;
							y_prime = sin(new_landmark_angle)*stg_x + cos(new_landmark_angle)*stg_y;
			
							new_landmark_X = TJX - x_prime;
							new_landmark_Y = TJY - y_prime;
							
							
							a->theta = (a->theta*a->average_cnt + new_landmark_angle*180/PI)/(a->average_cnt+1);		//degree
							a->x = (a->x*a->average_cnt + new_landmark_X)/(a->average_cnt+1);
							a->y = (a->y*a->average_cnt + new_landmark_Y)/(a->average_cnt+1);
							a->average_cnt = a->average_cnt + 1;
							if(a->average_cnt == 20)
							{	
								a->init_flag = 1;
								a->correct_flag = 1;	
							} 	

						}
						else
						{
							current_coord_theta = a->theta;
							current_coord_x = a->x;
							current_coord_y = a->y;

							landmark_known_flag = 1;	//1의 의미: DB에 랜드마크 사용
							if( sqrt(stg_x*stg_x+stg_y*stg_y) < 1.3 )	
							{
								m_visionUpdateFlag=1;
								//temp_a = a;
							}
							//m_visionUpdateFlag=1;							
						}									
					}					
				}
				
				if(landmark_known_flag == 2)	//DB에 등록되어 있지 않은 랜드마크인 경우, DB에 랜드마크 추가
				{
					if( sqrt(stg_x*stg_x+stg_y*stg_y) < 1.3 )
					{					
						new_landmark_angle = TJA - stg_angle*PI/180.0;		//radian
			
						x_prime = cos(new_landmark_angle)*stg_x - sin(new_landmark_angle)*stg_y;
						y_prime = sin(new_landmark_angle)*stg_x + cos(new_landmark_angle)*stg_y;
			
						new_landmark_X = TJX - x_prime;
						new_landmark_Y = TJY - y_prime;
						
						landmark* temp = (landmark*)malloc(sizeof(landmark));

						temp->id = stg_id;	
						temp->theta = new_landmark_angle*180/PI;		//degree
						temp->x = new_landmark_X;
						temp->y = new_landmark_Y;
						temp->init_flag = 0;
						temp->correct_flag = 0;		 
						temp->average_cnt = 1;
						temp->next = NULL;

						a->next = temp;							
					}
				}
				//새로운 랜드마크 기준 좌표를 reference 기준으로 좌표 변환 
				stg_angle = stg_angle + current_coord_theta;
				double stg_xx = cos(current_coord_theta*PI/180.0)*stg_x - sin(current_coord_theta*PI/180.0)*stg_y + current_coord_x;
				double stg_yy = sin(current_coord_theta*PI/180.0)*stg_x + cos(current_coord_theta*PI/180.0)*stg_y + current_coord_y;
				stg_x=stg_xx;
				stg_y=stg_yy;				
				

				//랜드마크가 잘못 등록된 경우 DB에서 랜드마크 삭제
				
				double stg_angle_temp = stg_angle, TJA_temp = TJA *180/PI;
				if( TJA_temp < 0.0 || stg_angle_temp < 0.0 )
				{	stg_angle_temp += 360.0;
					TJA_temp += 360.0;
				}
								
				test_cnt = 0;
				if( m_visionUpdateFlag == 1 && a->correct_flag == 1 )
				{
					a->correct_flag = 0;

					if(abs( stg_angle_temp - TJA_temp ) > 5.0 || abs( (stg_x - TJX)*100.0 ) > 10.0 || abs( (stg_y - TJY)*100.0 ) > 10.0 )
					{
						test_cnt = 1;					
						temp_a = Reference;
						//랜드마크 삭제를 위한 DB 검색
						for(; temp_a->next != NULL; temp_a = temp_a->next){
							
							if( temp_a->next->id == a->id )
							{									
								test_cnt = 2;
								temp_a->next = a->next;
								free(temp_a->next);							
								
							
								stg_angle = TJA*180.0/PI;
								stg_x = TJX;
								stg_y = TJY;								

								break;						
							}		
						}		
				 	}
				}
				//DB에서 랜드마크 삭제 종료
				
			
				
			}				
			if( stg_id != Reference->id && current_coord_theta == 0 && current_coord_x == 0 && current_coord_y == 0)
			{
				stg_angle = stg_angle_prev;
				stg_x = stg_x_prev;
				stg_y = stg_y_prev;
			}	
			
				
			stg_angle_prev = stg_angle;
			stg_x_prev = stg_x;
			stg_y_prev = stg_y;

			Z_x=stg_x;
			Z_y=stg_y;
			Z_a=stg_angle*PI/180.0;
		
		}
		
	}
	
	for(i=0; i<stg_i; i++)	stg_string[i] = 0;
	stg_i = 0;

}


/*
unsigned int stg_string_length=0,stg_string_length_temp=0;
char * keyword_pt, *length_pt, *string_pt;
char * pch;
int stg_cnt=0;
int null_cnt=0;
int flag1=0, flag2=0;
int stg_first=1;
*/
// MI 보드로부터 gyro값을 받는 함수
double MI_Angle=0, MI_Rate=0;
int MI_Angle_temp=0, MI_Rate_temp=0, MI_ChkSum=0,MI_ChkSum_temp=0;
int MI_flag = 0;
char MI_string[500]={0,};
void MI_module()
{
	//unsigned int hagisonic_string_length=0;
	char * keyword_pt, * temp_pt;
	//char * pch;
	char temp_char1, temp_char2;
	
	//unsigned int stg_string_length=0;
	//char ;
	
	static int MI_i=0;

	int i=0;
	

	while (USART_RXBufferData_Available(&USARTE0_data))
	{
		MI_string[MI_i] = USART_RXBuffer_GetByte(&USARTE0_data);
		MI_i++;	
		
	}	
	
	keyword_pt = MI_string;

	for(i=0; i<(MI_i/8); i++){
		/*
		while(*keyword_pt != 0xFF)
		{
			keyword_pt++;
		}
		*/
		keyword_pt= strchr(MI_string,0xFF);
		if(*(keyword_pt+1) == 0xFF){

			temp_pt = keyword_pt;
			temp_pt++;
			temp_pt++;
			temp_char1 = *temp_pt;
			temp_pt++;
			temp_char2 = *temp_pt;

			MI_Rate_temp = (int)(temp_char2)<<8;
			MI_Rate_temp +=(int)(temp_char1);

		
			temp_pt++;
			temp_char1 = *temp_pt;
			temp_pt++;
			temp_char2 = *temp_pt;

			MI_Angle_temp = (int)(temp_char2)<<8;
			MI_Angle_temp +=(int)(temp_char1);
		
			temp_pt++;
			temp_char1 = *temp_pt;
			temp_pt++;
			temp_char2 = *temp_pt;

			MI_ChkSum = (int)(temp_char2)<<8;
			MI_ChkSum +=(int)(temp_char1);
			
			MI_ChkSum_temp = 0xFFFF + MI_Rate_temp + MI_Angle_temp;

			if(MI_ChkSum == MI_ChkSum_temp){
				MI_Rate = (double)(MI_Rate_temp)/100.0;				
				MI_Angle = (double)(MI_Angle_temp)/100.0;
				MI_flag = 1;
			}

		
		}
	}
	for(i=0; i<MI_i; i++)	MI_string[i] = 0;
	MI_i = 0;
	/*
	for(i=0; i<MI_i; i++)
	{
		//sprintf(MI_buffer,"%c",stg_string[i]);
		//uartC0SendTXbit(*MI_buffer);
		sprintf(XYZ_buffer,"%c",MI_string[i]);
		uartC0SendTXbit(XYZ_buffer[0]);
	}
	*/	
	//sprintf(XYZ_buffer,"%d %d %d \n",MI_Rate,MI_Angle,MI_flag);
	//uartC0SendTX((unsigned char*)XYZ_buffer);

}


//PC로부터 encoder 신호를 받는 함수
void host_interface()
{


	static int initial_ENC=1;
//	static int encoder_on=0;

	unsigned int host_string_length=0;
	char * keyword_pt;
	unsigned char temp_char1, temp_char2, temp_char3;
	
	char * temp_pt;
	int encoder_on;



//	static int oldTimeDelay=0;


	static int host_i=0;
	
//	unsigned char XYZ_buffer[100];

	while (USART_RXBufferData_Available(&USARTC0_data))
	{
			host_string[host_i] = USART_RXBuffer_GetByte(&USARTC0_data);
			host_i++;
	}

	host_string[host_i]='\0';
	keyword_pt=strstr(host_string, "[");	

	temp_pt=keyword_pt;		
	host_string_length=strlen(keyword_pt);
	
	if(keyword_pt==NULL){
		host_i=0;
		host_string[0]='\0';
	}

	temp_pt++;
	if(*temp_pt=='E')
	{
		if(host_string_length>9 && *(temp_pt+8)==']')		
		{
			temp_pt++;
			temp_char1=*temp_pt;
			temp_pt++;
			temp_char2=*temp_pt;
			temp_pt++;
			temp_char3=*temp_pt;
	//					enc_left=temp_char1<<16+temp_char2<<8+temp_char3;
			if(temp_char1=='N')	encoder_on=0;
			else				encoder_on=1;
			if(encoder_on){
				m_LeftEncoder= (unsigned int)(temp_char1&0b00011111)<<11;
				m_LeftEncoder+=(unsigned int)(temp_char2&0b00011111)<<6;
				m_LeftEncoder+=(unsigned int)(temp_char3&0b00111111);
		//					enc_right=(keyword_pt+6)*256*256+(keyword_pt+7)*256+(keyword_pt+8);
				temp_pt++;
				temp_pt++;
				temp_char1=*temp_pt;
				temp_pt++;
				temp_char2=*temp_pt;
				temp_pt++;
				temp_char3=*temp_pt;
				m_RightEncoder= (unsigned int)(temp_char1&0b00011111)<<11;
				m_RightEncoder+=(unsigned int)(temp_char2&0b00011111)<<6;
				m_RightEncoder+=(unsigned int)(temp_char3&0b00111111);
		//					image_x=(keyword_pt+10)*256+(keyword_pt+11);		
							
				lt=m_LeftEncoder -m_prevDLeft;
				rt=m_RightEncoder-m_prevDRight;
						
			
				if(lt>32768)	m_dLeftInc=-(double)(65536-lt)/REDUCTION_ENCODER;
				else 			m_dLeftInc=(double)lt/REDUCTION_ENCODER;
				if(rt>32768)	m_dRightInc=-(double)(65536-rt)/REDUCTION_ENCODER;
				else 			m_dRightInc=(double)rt/REDUCTION_ENCODER;
							
				m_prevDLeft=m_LeftEncoder;
				m_prevDRight=m_RightEncoder;


				g_encoder_flag=1;
				//g_encoder_flag=encoder_on;

				if(initial_ENC){
					m_dLeftInc=0;
					m_dRightInc=0;
					initial_ENC=0;
				}
			}
			else{				
				m_dLeftInc=0;
				m_dRightInc=0;
				initial_ENC=1;
			}	
			host_i=host_string_length-10;
			strcpy(host_string,keyword_pt+10);
		}		
	}
}


double m_XXL;
double m_YXL;
double m_ZXL;
double m_XGY;
double m_YGY;
double m_ZGY;
double m_XXL0g;
double m_YXL0g;
double m_ZXL0g;



#define XL_BUF		10
#define XL_STOP_TH	0.002
#define SLIP_V_TH	0.45
#define SLIP_CNT_TH 10


#define BIAS_CHECK_CNT	100
#define BIAS_GYRO_TH		7000.0/2400*0.004697979*100

int slip_cnt[2]={0,0};
int RobotStateEstimation(){
	int i;
	int state=1;	// Robot is stopped
	static double  XLsqBuf[2][XL_BUF]={{0,},};
	static double  XLBuf[2][XL_BUF]={{0,},};
	static double XLsqSum[2]={0,0};
	static double XLSum[2]={0,0};
	static int indexX=0;
	double XLdata[2]={m_XXL, m_YXL};
	const double XLth[3]={0.0020 ,0.0020};
	double XLvar[2];
	indexX=(indexX+1)%XL_BUF;
	for(i=0;i<1;i++){
		XLsqSum[i]-= XLsqBuf[i][indexX];
		XLSum[i]-= XLBuf[i][indexX];
		XLsqBuf[i][indexX]= XLdata[i]* XLdata[i];
		XLBuf[i][indexX]= XLdata[i];
		XLsqSum[i]+= XLsqBuf[i][indexX];
		XLSum[i]+= XLBuf[i][indexX];
		XLvar[i]=(XLsqSum[i]-XLSum[i]*XLSum[i]/XL_BUF)/XL_BUF;
		if(XLvar[i]>XLth[i]){	
			state= 0;	// Robot is moving		
			break;
		}
	}

	if(state==1)
	{
		m_XXL_buf[indexG]=0;
		m_YXL_buf[indexG]=0;
	
		if(Vacc_x<0.1)	Vacc_x*=0.3;
		else			Vacc_x*=0.8;
	
		if(Vacc_y<0.1)	Vacc_y*=0.3;
		else			Vacc_y*=0.8;
	}	

//	double AccDifference=(XLSum[0]/10);
	double Vdif[2]={Vacc_x-Venc_x,Vacc_y-0};
//	double Vdif[2]={Vacc_x-Venc_x,Vacc_y-0};
	if(state ==0){	
		for(i=0;i<2;i++)
		{
			if(Vdif[i]>SLIP_V_TH || Vdif[i]<-SLIP_V_TH)		slip_cnt[i]++;//state= -1;//slip
			else	slip_cnt[i]=0;		

			if(slip_cnt[i]>SLIP_CNT_TH)	state=-1;
		}
	}
	/*
	if(Vdif[0]>SLIP_V_TH || Vdif[0]<-SLIP_V_TH)		slip_cnt++;//state= -1;//slip
	else	slip_cnt=0;		

	if(slip_cnt>SLIP_CNT_TH)	state=-1;
	*/
	return state;	
}


void CheckGyroBias(int k){
	static double vX[3]={0,0,0};
	static double sXsum[3]={0,0,0};
	static double Tsum[3]={0,0,0};
	static int bcnt[3]={0,0,0};
	double vXest=0;
	double *bias;
	double *gy;
	const double scale[3]={SCALE_XGY,SCALE_YGY,SCALE_ZGY};
	if(k==0){ 		
		bias= &m_biasXGY;
		gy=   &m_XGY;
	}
	else if(k==1){ 		
		bias= &m_biasYGY;
		gy=   &m_YGY;
	}
	else		{	//(k==2)
		bias= &m_biasZGY;
		gy=   &m_ZGY;
	}

	vX[k]+=*gy *T;
 	if(vX[k]<BIAS_GYRO_TH && vX[k]> -BIAS_GYRO_TH && robotState==1){	
		if(bcnt[k]==BIAS_CHECK_CNT){
			if(Tsum[k] !=0)	vXest=sXsum[k]*2/Tsum[k];
			else			vXest=0;				
			*bias+=vXest/scale[k]*(double)BIAS_CHECK_CNT/108;
			//taeilBuf[0]=vXest/SCALE_YGY*(double)BIAS_CHECK_CNT/108;
			//m_biasXXL+=vX/SCALE_XXL*BIAS_CHECK_CNT/108/2;
			//taeilBuf[0]=vX*BIAS_CHECK_CNT/108/SCALE_XXL/2;
			vX[k]=0;
			sXsum[k]=0;
			Tsum[k]=0;
			bcnt[k]=0;
			return;
		}
		else{			
			sXsum[k]+=vX[k]*T;
			Tsum[k]+=T;
			bcnt[k]++;	
		}
	}
	else{	
		vX[k]=0;
		sXsum[k]=0;
		Tsum[k]=0;
		bcnt[k]=0;
	//	taeilBuf[0]=0;
	}		
}


int main(void)
{
	unsigned int XXL,YXL,ZXL;
	//unsigned int ADC1,ADC2,ADC3;
//	int16_t ZGY;
	unsigned int XGY,YGY,ZGY;
//	unsigned char XYZ_buffer[100];
	unsigned int timer=0;
	unsigned int timer_old=0;
	unsigned int diff_counter=0;
	unsigned int m_IMU_count=0;
    double m_XXL_sum=0;
    double m_YXL_sum=0;
    double m_ZXL_sum=0;
    double m_XGY_sum=0;
    double m_YGY_sum=0;
    double m_ZGY_sum=0;


	clk_init();
	port_init();
 	USART_INIT();
	
	timer_Init();
	int_init();
//	sei();
	

//	spi_init();


	/* Init SS pin as output with wired AND and pull-up. */
	//120813 portC를 D로 바꿈 SPI인거 같아서  
	PORTD.DIRSET = PIN0_bm;
	PORTD.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
	PORTD.DIRSET = PIN1_bm;
	PORTD.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;

	/* Set SS output to high. (No slave addressed). */
	PORTD.OUTSET = PIN0_bm;
	PORTD.OUTSET = PIN1_bm;

	/* Instantiate pointer to ssPort. */
	PORT_t *ssPort = &PORTD;

	/* Initialize SPI master on port D. */
	SPI_MasterInit(&spiMasterD,
	               &SPID,
	               &PORTD,
	               false,
	               SPI_MODE_3_gc,
	               SPI_INTLVL_OFF_gc,
	               false,
	               SPI_PRESCALER_DIV16_gc);

	
	/* Use USARTD0 and initialize buffers. */
	//USART_InterruptDriver_Initialize(&USARTD0_data, &USARTD0, USART_DREINTLVL_HI_gc);

	/* Enable RXC interrupt. */
	//USART_RxdInterruptLevel_Set(USARTD0_data.usart, USART_RXCINTLVL_HI_gc);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_HILVLEX_bm;

	/* Enable global interrupts.*/
	sei();

	/* Initialize ACC & Gyro */
//	Init_L3G4200DH();
	Init_LIS3DH();
	
	SPI_MasterCreateDataPacket(&dataPacket,
		                           masterSendData_gyro_init,
		                           GYRO_DATA,
		                           NUM_BYTES,
		                           &PORTD,
		                           PIN1_bm);

		SPI_MasterSSLow(ssPort, PIN1_bm);

		/* Transceive packet. */
		SPI_MasterTransceivePacket(&spiMasterD, &dataPacket);
		/* MASTER: Release SS to slave. */

		SPI_MasterSSHigh(ssPort, PIN1_bm);



//	USART_Rx_Enable(&USARTD0);
//	USART_Tx_Enable(&USARTD0);


	

	double z_deg=0;


	unsigned int oldT=0;
	unsigned int newT=0;	
	unsigned int dT=0;	
	int i;

	
	double x1m=0;
	double x1p=0;
	
	double P1p=1.0;

	double x2m[4]={0,0,0,0};
	double x2p[4]={0,0,0,0};
	
	double P2p[4][4]={{0.1,0,0,0},
				 	  {0,0.1,0,0},
					  {0,0,0.1,0},
			 		  {0,0,0,0.1}};


	double enc_time_cnt=0;

	for(i=0;i<DELAY_COMP_G;i++)
	{
		m_XGY_buf[i]=0.0;
		m_YGY_buf[i]=0.0;
		m_ZGY_buf[i]=0.0;

		m_XXL_buf[i]=0.0;
		m_YXL_buf[i]=0.0;
		m_ZXL_buf[i]=0.0;

		m_enc_buf[i]=0.0;
		m_enc_timebuf[i]=0.0;
		m_T_buf[i]=0.0;

	}

	//char XYZ_buffer_debug[20];
	//sprintf((char*)XYZ_buffer_debug,"Hello W");
	//uartSendTX((unsigned char*)XYZ_buffer_debug);
	//landmark* Reference = (landmark*)malloc(sizeof(landmark));
	while(1) 
	{
		
		
		//if(1)
		if(samplingFlag)
		{
				
		
			
//				adc_start_conversion(&ADCA, ADC_CH0);			
				samplingFlag=0;
				timer=TCC0.CNT;
				diff_counter=timer-timer_old;
				
				
				/* Create data packet (SS to slave by PC0). */
				SPI_MasterCreateDataPacket(&dataPacket,
				                           masterSendData,
				                           ACC_DATA,
				                           NUM_BYTES,
				                           &PORTD,
				                           PIN0_bm);


				/* MASTER: Pull SS line low. This has to be done since
				 *         SPI_MasterTransceiveByte() does not control the SS line(s). */
				SPI_MasterSSLow(ssPort, PIN0_bm);
				_delay_us(5);
				/* Transceive packet. */
				SPI_MasterTransceivePacket(&spiMasterD, &dataPacket);
				/* MASTER: Release SS to slave. */
				_delay_us(5);
				SPI_MasterSSHigh(ssPort, PIN0_bm);
				
				
				/* Create data packet (SS to slave by PC1). */
				SPI_MasterCreateDataPacket(&dataPacket,
				                           masterSendData_gyro,
				                           GYRO_DATA,
				                           NUM_BYTES,
				                           &PORTD,
				                           PIN1_bm);

				/* MASTER: Pull SS line low. This has to be done since
				 *         SPI_MasterTransceiveByte() does not control the SS line(s). */
				SPI_MasterSSLow(ssPort, PIN1_bm);
				/* Transceive packet. */
				_delay_us(5);
				SPI_MasterTransceivePacket(&spiMasterD, &dataPacket);
				/* MASTER: Release SS to slave. */
				_delay_us(5);
				SPI_MasterSSHigh(ssPort, PIN1_bm);







				timer_old=timer;

				T=(double)diff_counter/2000000.0*32.0;

				ACC_DATA[2]=ACC_DATA[2]+0x80;
				ACC_DATA[4]=ACC_DATA[4]+0x80;
				ACC_DATA[6]=ACC_DATA[6]+0x80;
				YXL= (unsigned int)ACC_DATA[2]*256+ACC_DATA[1];
				XXL= (unsigned int)ACC_DATA[4]*256+ACC_DATA[3];
				ZXL= (unsigned int)ACC_DATA[6]*256+ACC_DATA[5];

				GYRO_DATA[2]=GYRO_DATA[2]+0x80;
				GYRO_DATA[4]=GYRO_DATA[4]+0x80;
				GYRO_DATA[6]=GYRO_DATA[6]+0x80;
				XGY= (unsigned int)GYRO_DATA[4]*256+GYRO_DATA[3];
				YGY= (unsigned int)GYRO_DATA[2]*256+GYRO_DATA[1];
				ZGY= (unsigned int)GYRO_DATA[6]*256+GYRO_DATA[5];	
			
			if(m_IMU_count<Tsample)
			{
				m_IMU_count++;
			}
			else if(m_IMU_count<Tsample+Bsample)
			{
				m_XXL_sum+=XXL;
				m_YXL_sum+=YXL;
				m_ZXL_sum+=ZXL;
				//m_XGY_sum+=XGY;
				//m_YGY_sum+=YGY;
				m_ZGY_sum+=ZGY;
				m_IMU_count++;
			}
			else if(m_IMU_count==Tsample+Bsample)
			{
				//SetTimer(25,1,NULL);
				m_biasXXL=(double)m_XXL_sum/(double)Bsample;
				m_biasYXL=(double)m_YXL_sum/(double)Bsample;
				m_biasZXL=(double)m_ZXL_sum/(double)Bsample-GRAVITY_COUNT;
				//m_biasXGY=(double)m_XGY_sum/(double)Bsample;
				//m_biasYGY=(double)m_YGY_sum/(double)Bsample;
				m_biasZGY=(double)m_ZGY_sum/(double)Bsample;

				
				gravityVect[0]=0;
				gravityVect[1]=0;
				gravityVect[2]=SCALE_ZXL*GRAVITY_COUNT;

				m_IMU_count++;

		    }
		    else
			{
				
			
				//encoder_interface(0);
				//encoder_interface(1);
				
				//host_interface();
				


				//unsigned int a=TCC0.CNT;

				//position_estimator(XXL,YXL,ZXL,XGY,YGY,ZGY);
				
				//unsigned int b=TCC0.CNT;

				//TJX=(double)(b-a)/2000000.0*32.0;

				//FF_controller();

				newT=TCC0.CNT;
				dT=newT-oldT;

				
				////////////////////////////////////////////////
				///////////////////////////////////////////////////////
				///////////////////////////////////////////////////////
				///////////////////////////////////////////////////////
				///////////////////////////////////////////////////////
				///////////////////////////////////////////////////////
				///////////////////////////////////////////////////////
				MI_flag=0;
				//MI_module();
					    	
				int i,j,k;	

				m_XXL=-SCALE_XXL*((double)XXL-(double)m_biasXXL);
				m_YXL=-SCALE_YXL*((double)YXL-(double)m_biasYXL);
				m_ZXL=SCALE_ZXL*((double)ZXL-(double)m_biasZXL);
				m_XGY=-SCALE_XGY*((double)XGY-(double)m_biasXGY);//-0.001212142/0.00015711
				m_YGY=SCALE_YGY*((double)YGY-(double)m_biasYGY);//+
				//if(ZGY<3000)	m_ZGY=SCALE_ZGY*((double)ZGY-(double)m_biasZGY+65536.0);
				//else 			m_ZGY=SCALE_ZGY*((double)ZGY-(double)m_biasZGY);//+
				m_ZGY=SCALE_ZGY*((double)ZGY-(double)m_biasZGY);
				//m_ZGY=-SCALE_ZGY*((double)ZGY-(double)m_biasZGY);
				//m_ZGY=-MI_Rate*PI/180.0;
				
				gyro_test=m_ZGY;

				CheckGyroBias(2);
				/*
				//hysteresis
				
				if(m_ZGY<Hys_TH*0.2 && m_ZGY>-Hys_TH*0.2)
				{
					m_ZGY=0;
				}	
			
				//moving averaging
				if(f_cnt<9)
				{
					m_ZGY_F[g_cnt]=m_ZGY;
					m_ZGY_filter=m_ZGY;
					//m_ZGY_F_sum+=m_ZGY_F[g_cnt];
					g_cnt++;
					f_cnt++;
				}
				
				else 
				{
					m_ZGY_F[g_cnt]=m_ZGY;
					//m_ZGY_F_sum+=m_ZGY_F[g_cnt];
					m_ZGY_filter=(m_ZGY_F[0]+m_ZGY_F[1]+m_ZGY_F[2]+m_ZGY_F[3]+m_ZGY_F[4]+m_ZGY_F[5]+m_ZGY_F[6]+
								  m_ZGY_F[7]+m_ZGY_F[8]+m_ZGY_F[9])/10;
					g_cnt++;
					if(g_cnt==10) g_cnt=0;
				}
				
				*/




				//m_XGY=0;
				//m_YGY=0;
				//m_ZGY=0;
				/*
				double Z_B=(double)ZGY-(double)m_biasZGY;//+
				if(Z_B>0)	m_ZGY=SCALE_ZGY_POS*Z_B;//+
				else		m_ZGY=SCALE_ZGY_NEG*Z_B;//+
				*/
				z_deg+=m_ZGY*T;
				//z_deg+=gyro_test*T;
				z_deg_filter+=m_ZGY_filter*T;
				
				indexG=(indexG+1)%DELAY_COMP_G;
				indexD=(indexG+DELAY_COMP_G-timeDelay)%DELAY_COMP_G;
	
				
				//m_XGY_buf[indexG]=m_XGY;
				//m_YGY_buf[indexG]=m_YGY;
				//m_ZGY_buf[indexG]=m_ZGY;
				//  m_ZGY_buf[indexG]=m_ZGY_filter;	
				m_ZGY_buf[indexG]=gyro_test;
								
				//m_XXL=0;
				//m_YXL=0;
				m_XXL_buf[indexG]=m_XXL;
				m_YXL_buf[indexG]=m_YXL;
				
				m_T_buf[indexG]=T;
				
				g_encoder_flag=0;
				m_visionUpdateFlag=0;
				
				
				
				stg_cnt++;
				if(stg_cnt == 10)
				{	
					stg_module();
					
					/*
					landmark *a = Reference;
					if(a->average_cnt == 0)
					{
						int linked_cnt=1;
						for(; a->next != NULL; a = a->next)
						{
							linked_cnt++;
						}
						a= Reference;

						for(int linked_output = 0; linked_output < linked_cnt; linked_output++)
						{
			
							sprintf(XYZ_buffer,"%d %d %d %d %d %d %d %d \n",stg_id,a->id,(int)(a->theta),(int)(a->x*100.0),(int)(a->y*100.0),linked_cnt,landmark_known_flag,test_cnt);
							uartC0SendTX((unsigned char*)XYZ_buffer);
							a = a->next;
						}
					}
					*/
					stg_cnt=0;
				}
				/*
				sprintf((char*)XYZ_buffer,"%d %d %d %d %d %d %d %d %d %d %d %d\n",temp_a->id,(int)(TJA*180.0/PI),(int)(TJX*100.0),(int)(TJY*100.0),
								(int)(TJA_prev*180/PI),(int)(TJX_prev*100.0),(int)(TJY_prev*100.0),(int)(MI_Angle),m_visionUpdateFlag,g_encoder_flag,
								MI_flag,stg_id);
		
								
				//sprintf((char*)XYZ_buffer,"%d %d %d \n",(int)(stg_angle),(int)(stg_x*100.0),(int)(stg_y*100.0));
				
				uartC0SendTX((unsigned char*)XYZ_buffer);
				*/
				//좌표변환 값이 튀었을때 예외처리
							
				
			
				
				
				host_interface();			
				
				m_enc_sum=0;
				m_XGY_sum=0;
				m_YGY_sum=0;
				m_ZGY_sum=0;
				/*
				double enc_sum_KF=0;
				int index;
				for(i=0;i<ENC_LPF;i++){
					index=(indexG-i+DELAY_COMP_G)%DELAY_COMP_G;
					enc_sum_KF+=m_enc_buf[index];
				}
				*/
				
				//m_enc_buf[indexG]=(m_dLeftInc+m_dRightInc)/2.0;

				enc_time_cnt+=T;
				
				if(g_encoder_flag==0)
				{
					m_enc_buf[indexG]=0;
					m_enc_timebuf[indexG]=0;
					m_enc_buf_left[indexG]=0;
					m_enc_buf_right[indexG]=0;
					
					//enc_time_cnt++;
				}
				else
				{
					m_enc_buf[indexG]=(m_dLeftInc+m_dRightInc)/2.0;
					m_enc_buf_left[indexG]=m_dLeftInc;
					m_enc_buf_right[indexG]=m_dRightInc;

					m_enc_timebuf[indexG]=enc_time_cnt;
 					//Z=m_enc_buf[indexG]/m_enc_timebuf[indexG];
					enc_time_cnt=0;
					
					m_dLeftInc=0;
					m_dRightInc=0;
				}

				int encoder_update_flag=0;
				if(m_enc_timebuf[indexD]>0)	encoder_update_flag=1;//k-d번째 시점에 encoder 들어왔으면 update하기
				
////////////////////////////////////robot_state 판별부분 -1 : slip / 0 : moving / 1 : stop

				Vacc_x+=m_XXL*T;
				Vacc_y+=m_YXL*T;
				if(g_encoder_flag)
				{
					//Venc_x = m_enc_buf[indexG]/m_enc_timebuf[indexG];
					double time_sum=0;
					double enc_sum=0;
					double enc_sum_right=0;
					double enc_sum_left=0;


					for(i=0;i<DELAY_COMP_G;i++)
					{
						time_sum+=m_enc_timebuf[i];
						enc_sum+=m_enc_buf[i];
						enc_sum_left +=m_enc_buf_left[i];
						enc_sum_right+=m_enc_buf_right[i];

					}
					if(time_sum>0)
					{
						Venc_x=enc_sum/time_sum;
						Venc_x_left =enc_sum_left/time_sum;
						Venc_x_right=enc_sum_right/time_sum;
					}		

				}
				
				robotState=RobotStateEstimation();
				//if(robotState<0)	robotState=0;
				robotState_buf[indexG]=robotState;
				//robotState_buf[indexG]=0;
/////////////////////////////////////////////////////////
				/*
				if(robotState==1)
				{
					m_XXL_buf[indexG]=0;
					m_YXL_buf[indexG]=0;
				
					if(Vacc_x<0.1)	Vacc_x*=0.3;
					else			Vacc_x*=0.8;
				
					if(Vacc_y<0.1)	Vacc_y*=0.3;
					else			Vacc_y*=0.8;
				}
				*/
				//double Z=enc_sum_KF/ENC_LPF/T;
				//double Z=m_enc_buf[indexG]/T;
				//enc_check+=m_enc_buf[indexG];

////////////////////////////////////////////////////////theta EKF 시작
				double Q1=0.0001;
				double R1=0.4;
				double P1m;
				
				//theta State Estimate Extrapolation

				x1m=x1p+m_T_buf[indexD]*m_ZGY_buf[indexD];

				if(x1m<-PI)
				{
					while(x1m<-PI)	x1m+=2*PI;
				}
				else if(x1m>PI)
				{
					while(x1m>PI)	x1m-=2*PI;
				}

				//x1m=x1p+T*m_ZGY;

				//theta Error Covariance Extrapolaion
				P1m=P1p+Q1;
				
				if(m_visionUpdateFlag)
				{
					if(Z_a<-PI)
					{
						while(Z_a<-PI)	Z_a+=2*PI;
					}
					else if(Z_a>PI)
					{
						while(Z_a>PI)	Z_a-=2*PI;
					}
					
					if(x1m<-0.5*PI && Z_a>0.5*PI)	Z_a-=2*PI;

					else if(x1m>0.5*PI && Z_a<-0.5*PI)	Z_a+=2*PI;

					double inv1=P1m+R1;
					//if(Z_a<0)	Z_a+=3.141592*2.0;

					if(inv1==0)
					{
						x1p=x1m;
						P1p=P1m;
					}
					else
					{

						//2. theta EKF Kalman Gain
						double K1=P1m*(1/inv1);
						//3. State Estimate Update
						double Z1v=Z_a-x1m;
						/*
						if(Z1v>PI)	Z1v=(Z1v-2*PI);
						else if(Z1v<PI)	Z1v=(Z1v+2*PI);
						*/
						x1p=x1m+K1*Z1v;
						// 4. error covariance update
						P1p=P1m-K1*P1m;
					}
				}
				else
				{
					x1p=x1m;
					P1p=P1m;
				}

/////////////////////////////theta EKF 끝
				
				//rotation matrix 계산
				double R[2][2];
								
				R[0][0]=cos(x1p);
				R[0][1]=-sin(x1p);
				R[1][0]=sin(x1p);
				R[1][1]=cos(x1p);
				
				if(vision_init_flag==0 && m_visionUpdateFlag==1){
				//if(m_visionUpdateFlag==1){
					x2p[0]=Z_x;
					x2p[1]=Z_y;
					//vision_init_flag=1;
				}	
				
				double TJvec1_r[2]={1 , 0};
				double TJvec2_w[2];

				TJvec2_w[0]=R[0][0]*TJvec1_r[0] + R[0][1]*TJvec1_r[1];
				TJvec2_w[1]=R[1][0]*TJvec1_r[0] + R[1][1]*TJvec1_r[1];
				
//////////////////////////////robot position/velocity estimator using EKF 시작

				double R2e[2][2]={{0.1,0},
								  {0,0.1}};
								
				double R2v[2][2]={{0.3,0},
								  {0,0.3}};
				//double R2v[2][2]={{0.001,0},
				//				  {0,0.001}};
							
				double Q2[4][4]={{0.0001,0,0,0},//0.00001
								 {0,0.0001,0,0},
								 {0,0,0.01,0},
								 {0,0,0,0.01}};
				
				double P2m[4][4]={{0,0,0,0},
				 			  	  {0,0,0,0},
								  {0,0,0,0},
								  {0,0,0,0}};



				//robot position/velocity State Estimate Extrapolation

				x2m[0]=x2p[0]+m_T_buf[indexD]*x2p[2];
				x2m[1]=x2p[1]+m_T_buf[indexD]*x2p[3];
				x2m[2]=x2p[2]+m_T_buf[indexD]*( R[0][0]*m_XXL_buf[indexD] + R[0][1]*m_YXL_buf[indexD] );
				x2m[3]=x2p[3]+m_T_buf[indexD]*( R[1][0]*m_XXL_buf[indexD] + R[1][1]*m_YXL_buf[indexD] );
				

				//robot p/v Error Covariance Extfrapolaion
				
				if(robotState_buf[indexD]==-1)
				{
					x2m[2]=R[0][0]*Vacc_x+R[0][1]*Vacc_y;
					x2m[3]=R[1][0]*Vacc_x+R[1][1]*Vacc_y;
				}
				
				double A2[4][4]={{1,0,m_T_buf[indexD],0},
								 {0,1,0,m_T_buf[indexD]},
								 {0,0,1,0},
								 {0,0,0,1}};
				
				double A2T[4][4]={{1,0,0,0},
								  {0,1,0,0},
								  {m_T_buf[indexD],0,1,0},
								  {0,m_T_buf[indexD],0,1}};
				
				double A2_P2p[4][4]={{0,0,0,0},
								  	 {0,0,0,0},
								  	 {0,0,0,0},
								  	 {0,0,0,0}};
				
				for( i=0;i<4;i++)
					for( j=0;j<4;j++)
						for( k=0;k<4;k++)
							A2_P2p[i][j]+= A2[i][k]*P2p[k][j];
				
				for( i=0;i<4;i++)
					for( j=0;j<4;j++){
						P2m[i][j]+=Q2[i][j];
						for( k=0;k<4;k++)
							P2m[i][j]+= A2_P2p[i][k]*A2T[k][j];
					}
				
//////////////////////////////////비전을 이용한 로봇 위치/속력 measurement update 시작
				if(m_visionUpdateFlag){
					
					double dist_xv=sqrt((Z_x-x2m[0])*(Z_x-x2m[0])+(Z_y-x2m[1])*(Z_y-x2m[1]));
					if(dist_xv<0.5)
					{
						double Z2v[2]={Z_x,Z_y};
	
						//2. Kalman Gain Matrix
					
						double K2v[4][2]={{0,0},
										  {0,0},
										  {0,0},
										  {0,0}};

						double inv_v[2][2];
						inv_v[0][0]=P2m[0][0]+R2v[0][0];
						inv_v[0][1]=P2m[0][1]+R2v[0][1];
						inv_v[1][0]=P2m[1][0]+R2v[1][0];
						inv_v[1][1]=P2m[1][1]+R2v[1][1];

						double det_v=inv_v[0][0]*inv_v[1][1]-inv_v[0][1]*inv_v[1][0];

						if(det_v==0)
						{
							for(i=0;i<4;i++)
							{
								x2p[i]=x2m[i];
								for(j=0;j<4;j++)
									P2p[i][j]=P2m[i][j];
							}
						}
						else
						{
							double inv_vv[2][2];
							inv_vv[0][0]=inv_v[1][1]/det_v;
							inv_vv[0][1]=-inv_v[0][1]/det_v;
							inv_vv[1][0]=-inv_v[1][0]/det_v;
							inv_vv[1][1]=inv_v[0][0]/det_v;
						
							K2v[0][0]=P2m[0][0]*inv_vv[0][0]+P2m[0][1]*inv_vv[1][0];
							K2v[0][1]=P2m[0][0]*inv_vv[0][1]+P2m[0][1]*inv_vv[1][1];

							K2v[1][0]=P2m[1][0]*inv_vv[0][0]+P2m[1][1]*inv_vv[1][0];
							K2v[1][1]=P2m[1][0]*inv_vv[0][1]+P2m[1][1]*inv_vv[1][1];

							K2v[2][0]=P2m[2][0]*inv_vv[0][0]+P2m[2][1]*inv_vv[1][0];
							K2v[2][1]=P2m[2][0]*inv_vv[0][1]+P2m[2][1]*inv_vv[1][1];

							K2v[3][0]=P2m[3][0]*inv_vv[0][0]+P2m[3][1]*inv_vv[1][0];
							K2v[3][1]=P2m[3][0]*inv_vv[0][1]+P2m[3][1]*inv_vv[1][1];
					
							//robot p/v state update
						
							Z2v[0]-=x2m[0];
							Z2v[1]-=x2m[1];
						
							x2p[0]=x2m[0]+(K2v[0][0]*Z2v[0]+K2v[0][1]*Z2v[1]);
							x2p[1]=x2m[1]+(K2v[1][0]*Z2v[0]+K2v[1][1]*Z2v[1]);
							x2p[2]=x2m[2]+(K2v[2][0]*Z2v[0]+K2v[2][1]*Z2v[1]);
							x2p[3]=x2m[3]+(K2v[3][0]*Z2v[0]+K2v[3][1]*Z2v[1]);

							//robot p/v Error Covariance update
						
							double KHv[4][4]={{K2v[0][0],K2v[0][1],0,0},
											  {K2v[1][0],K2v[1][1],0,0},
											  {K2v[2][0],K2v[2][1],0,0},
											  {K2v[3][0],K2v[3][1],0,0}};
						
							for( i=0;i<4;i++)
								for( j=0;j<4;j++){
									P2p[i][j]=P2m[i][j];
									for( k=0;k<4;k++)
										P2p[i][j]-= KHv[i][k]*P2m[k][j];
								}
						}
					}
					else	m_visionUpdateFlag=0;
				}
				
				
///////////////////////////////////////////////비전을 이용한 robot position-velocity measurement update 끝

////////////////////////////////////////////////엔코더를 이용한 로봇 위치/속력 measurement update 시작
				double slip_flag=robotState_buf[indexD];
				

				if((encoder_update_flag==1) && (slip_flag!=-1)){
										
					double Z=m_enc_buf[indexD]/m_enc_timebuf[indexD];
					//double Z=m_enc_buf[indexG]/T;
					double Z2e[2]={Z,0};;
					
					//2. Kalman Gain Matrix
					
					double K2e[4][2]={{0,0},
									  {0,0},
									  {0,0},
									  {0,0}};

					double inv_e[2][2];
					
					inv_e[0][0]=R[0][0]*(R[0][0]*P2m[2][2]+R[1][0]*P2m[3][2])+R[1][0]*(R[0][0]*P2m[2][3]+R[1][0]*P2m[3][3]) +R2e[0][0];
					inv_e[0][1]=R[0][1]*(R[0][0]*P2m[2][2]+R[1][0]*P2m[3][2])+R[1][1]*(R[0][0]*P2m[2][3]+R[1][0]*P2m[3][3]) +R2e[0][1];
					inv_e[1][0]=R[0][0]*(R[0][1]*P2m[2][2]+R[1][1]*P2m[3][2])+R[1][0]*(R[0][1]*P2m[2][3]+R[1][1]*P2m[3][3]) +R2e[1][0];
					inv_e[1][1]=R[0][1]*(R[0][1]*P2m[2][2]+R[1][1]*P2m[3][2])+R[1][1]*(R[0][1]*P2m[2][3]+R[1][1]*P2m[3][3]) +R2e[1][1];
									

					double det_e=inv_e[0][0]*inv_e[1][1]-inv_e[0][1]*inv_e[1][0];

					if(det_e==0)
					{
						for(i=0;i<4;i++)
						{
							x2p[i]=x2m[i];
							for(j=0;j<4;j++)
								P2p[i][j]=P2m[i][j];
						}
					}
					else
					{
						double inv_ee[2][2];
						inv_ee[0][0]=inv_e[1][1]/det_e;
						inv_ee[0][1]=-inv_e[0][1]/det_e;
						inv_ee[1][0]=-inv_e[1][0]/det_e;
						inv_ee[1][1]=inv_e[0][0]/det_e;
						
						double PHTv[4][2];
						
						PHTv[0][0]=P2m[0][2]*R[0][0]+P2m[0][3]*R[1][0];
						PHTv[0][1]=P2m[0][2]*R[0][1]+P2m[0][3]*R[1][1];
						
						PHTv[1][0]=P2m[1][2]*R[0][0]+P2m[1][3]*R[1][0];
						PHTv[1][1]=P2m[1][2]*R[0][1]+P2m[1][3]*R[1][1];

						PHTv[2][0]=P2m[2][2]*R[0][0]+P2m[2][3]*R[1][0];
						PHTv[2][1]=P2m[2][2]*R[0][1]+P2m[2][3]*R[1][1];
						
						PHTv[3][0]=P2m[3][2]*R[0][0]+P2m[3][3]*R[1][0];
						PHTv[3][1]=P2m[3][2]*R[0][1]+P2m[3][3]*R[1][1];

						
						K2e[0][0]=PHTv[0][0]*inv_ee[0][0]+PHTv[0][1]*inv_ee[1][0];
						K2e[0][1]=PHTv[0][0]*inv_ee[0][1]+PHTv[0][1]*inv_ee[1][1];

						K2e[1][0]=PHTv[1][0]*inv_ee[0][0]+PHTv[1][1]*inv_ee[1][0];
						K2e[1][1]=PHTv[1][0]*inv_ee[0][1]+PHTv[1][1]*inv_ee[1][1];

						K2e[2][0]=PHTv[2][0]*inv_ee[0][0]+PHTv[2][1]*inv_ee[1][0];
						K2e[2][1]=PHTv[2][0]*inv_ee[0][1]+PHTv[2][1]*inv_ee[1][1];

						K2e[3][0]=PHTv[3][0]*inv_ee[0][0]+PHTv[3][1]*inv_ee[1][0];
						K2e[3][1]=PHTv[3][0]*inv_ee[0][1]+PHTv[3][1]*inv_ee[1][1];
					
						//robot p/v state update

						Z2e[0]-=(R[0][0]*x2m[2]+R[1][0]*x2m[3]);
						Z2e[1]-=(R[0][1]*x2m[2]+R[1][1]*x2m[3]);
						
						x2p[0]=x2m[0]+(K2e[0][0]*Z2e[0]+K2e[0][1]*Z2e[1]);
						x2p[1]=x2m[1]+(K2e[1][0]*Z2e[0]+K2e[1][1]*Z2e[1]);
						x2p[2]=x2m[2]+(K2e[2][0]*Z2e[0]+K2e[2][1]*Z2e[1]);
						x2p[3]=x2m[3]+(K2e[3][0]*Z2e[0]+K2e[3][1]*Z2e[1]);

						//robot p/v Error Covariance update
						
						double KHe[4][4]={{0,0, K2e[0][0]*R[0][0] + K2e[0][1]*R[0][1] , K2e[0][0]*R[1][0] + K2e[0][1]*R[1][1] },
										  {0,0, K2e[1][0]*R[0][0] + K2e[1][1]*R[0][1] , K2e[1][0]*R[1][0] + K2e[1][1]*R[1][1] },
										  {0,0, K2e[2][0]*R[0][0] + K2e[2][1]*R[0][1] , K2e[2][0]*R[1][0] + K2e[2][1]*R[1][1] },
										  {0,0, K2e[3][0]*R[0][0] + K2e[3][1]*R[0][1] , K2e[3][0]*R[1][0] + K2e[3][1]*R[1][1] }};
						for( i=0;i<4;i++)
							for( j=0;j<4;j++){
								P2p[i][j]=P2m[i][j];
								for( k=0;k<4;k++)
									P2p[i][j]-= KHe[i][k]*P2m[k][j];
							}
					}
				}
				
				
				
				if(encoder_update_flag == 0 && m_visionUpdateFlag==0)
				{
					for(i=0;i<4;i++)
					{
						x2p[i]=x2m[i];
						for(j=0;j<4;j++)
							P2p[i][j]=P2m[i][j];
					}		
				}



				/////////////////////////////////////////////////////////////////
				//EKF 끝!!!
				//k-d 시점의 robot state 추정 끝
				//k-d  시점의 robot state 추정--> 현재 (k 시점) robot state 계산 시작
				double x1p_p=x1p;
				double x1m_p=x1m;				
				
				double x2p_p[4]={x2p[0],x2p[1],x2p[2],x2p[3]};
				double x2m_p[4]={x2m[0],x2m[1],x2m[2],x2m[3]};
								
				for( i=1; i<timeDelay+1; i++)
				{
					int indexD_p=(indexD+i)%DELAY_COMP_G;

					x1m_p=x1p_p+m_T_buf[indexD_p]*m_ZGY_buf[indexD_p];
													
					R[0][0]=cos(x1p);
					R[0][1]=-sin(x1p);
					R[1][0]=sin(x1p);
					R[1][1]=cos(x1p);
			
					x2m_p[0]=x2p_p[0]+m_T_buf[indexD_p]*x2p_p[2];
					x2m_p[1]=x2p_p[1]+m_T_buf[indexD_p]*x2p_p[3];

					if(m_enc_timebuf[indexD_p]==0)
					{

						x2m_p[2]=x2p_p[2]+m_T_buf[indexD_p]*(R[0][0]*m_XXL_buf[indexD_p]
										 				    +R[0][1]*m_YXL_buf[indexD_p]);
						x2m_p[3]=x2p_p[3]+m_T_buf[indexD_p]*(R[1][0]*m_XXL_buf[indexD_p]
										 				    +R[1][1]*m_YXL_buf[indexD_p]);


					}
					else
					{
						x2m_p[2]=R[0][0]*(m_enc_buf[indexD_p]/m_enc_timebuf[indexD_p]);
						x2m_p[3]=R[1][0]*(m_enc_buf[indexD_p]/m_enc_timebuf[indexD_p]);
					}
					

					x1p_p=x1m_p;

					x2p_p[0]=x2m_p[0];
					x2p_p[1]=x2m_p[1];
					x2p_p[2]=x2m_p[2];
					x2p_p[3]=x2m_p[3];
				}
				
				//k-d  시점의 robot state 추정--> 현재 (k 시점) robot state 계산 끝				
////////////////////////////////////////////////////////////////////
			
				TJA_prev=TJA;
				TJX_prev=TJX;
				TJY_prev=TJY;
				TJW_prev=TJW;
				TJZ_prev=TJZ;

				TJA=x1p;
				TJX=x2m_p[0];
				TJY=x2m_p[1];
				TJW=x2m_p[2];
				TJZ=x2m_p[3];
				
						
			
		
		sprintf((char*)XYZ_buffer,"%d %d %d %d %d %d %d %d %d %d \n",stg_id,(int)(TJA*180.0/PI),(int)(TJX*100.0),(int)(TJY*100.0),
								(int)(stg_angle),(int)(stg_x*100.0),(int)(stg_y*100.0),m_visionUpdateFlag,test_cnt,robotState);	
		
		//sprintf((char*)XYZ_buffer,"%d %d \n",(int)(gyro_test*100.0), (int)(z_deg*180.0/PI));	//MCU에 부착된 자이로 센서 테스트
		uartC0SendTX((unsigned char*)XYZ_buffer);
		
					

		
			}
		}
	}

	while(true) {
		nop();
	}
}

/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */

