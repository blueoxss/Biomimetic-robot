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





#define DELAY_COMP_G 20




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
	USART_Baudrate_Set(&USARTD0, 16, 0);		// 115200, 하기소닉 x,y,theta 받을때의 baudrate
	USARTD0.CTRLB|=0x04;                      //CLK2X


	/* Enable both RX and TX. */
	USART_Rx_Enable(&USARTD0);
	USART_Tx_Enable(&USARTD0);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;



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

	USART_Baudrate_Set(&USARTE0, 16 , 1);		// 57600, 하기소닉 보드에 encoder 값 넣을 때의 baudrate
	//USART_Baudrate_Set(&USARTE0, 16, 0);		// 115200
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
// int H_angle=0, H_x=0, H_y=0;
// int Encoder_angle=0, Encoder_x=0, Encoder_y=0;
unsigned int H_angle=0, H_x=0, H_y=0;
unsigned int Encoder_angle=0, Encoder_x=0, Encoder_y=0;
 double H_angle_deg=0;
 double Encoder_angle_deg=0;
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
char	H_encoderL_BM2=0, H_encoderR_BM2=0;

double	H_encoderL_sum=0, H_encoderR_sum=0;






/*
char * temp_pt;
char * keyword_pt;
unsigned char hagisonic_string_length=0;
*/
//communication test
char receive_data[100];

/*
void hagisonic_module()
{
//	char	hagisonic_string[300]={0};
//	unsigned int hagisonic_string_length=0;
	char * keyword_pt;
	unsigned char temp_char1, temp_char2;
	
	char * temp_pt;
//	char XYZ_buffer[100];

	static int hagisonic_i=0;	
	hagisonic_i=0;
	
	while (USART_RXBufferData_Available(&USARTD0_data))
	{
		hagisonic_string[hagisonic_i] = USART_RXBuffer_GetByte(&USARTD0_data);
		hagisonic_i++;
		
	
	}
	for(int i=0; i<hagisonic_i; i++)
		uartC0SendTXbit(hagisonic_string[i]);


//	hagisonic_string[hagisonic_i]='\0';
	
	
	//if(hagisonic_i>50) 

//		uartC0SendTX((unsigned char*)hagisonic_string);
		hagisonic_i=0;





}
*/

//#if 0
//char * keyword_pt, *temp_pt;
//unsigned int hagisonic_string_length=0;

void hagisonic_module()
{

//	unsigned int hagisonic_string_length=0;

	char * keyword_pt, *temp_pt;
	char temp_char1, temp_char2;
	
	//int 	i_flag=0;
	static unsigned int hagisonic_i=0;	
	
	while (USART_RXBufferData_Available(&USARTD0_data))
	{
		hagisonic_string[hagisonic_i] = USART_RXBuffer_GetByte(&USARTD0_data);
		hagisonic_i++;
		
	}

	i_cnt=hagisonic_i;
	//hagisonic_string[hagisonic_i]=0xFF;
	keyword_pt=hagisonic_string;
	//hagisonic_i=0;
	
	for(int i=0; i<(hagisonic_i/14); i++)
	{
		while(*keyword_pt!=0x02)
		{
			keyword_pt++;
		}
		if(*(keyword_pt+13)==0x03)
		{
			temp_pt=keyword_pt;
			temp_char1=*temp_pt;
			start_bit=(unsigned int)(temp_char1&0b11111111);
			/*
			//store theta data
			temp_pt++;
			temp_char1=*temp_pt;
			temp_pt++;
			temp_char2=*temp_pt;
					 
			H_angle=((int)temp_char1)*256;
			H_angle+=(int)temp_char2;

			//store x data
			temp_pt++;
			temp_char1=*temp_pt;
			temp_pt++;
			temp_char2=*temp_pt;
	
			H_x= ((int)temp_char1)*256;
			H_x+=(int)temp_char2;
			
			//store y data
			temp_pt++;
			temp_char1=*temp_pt;
			temp_pt++;
			temp_char2=*temp_pt;

			H_y=((int)temp_char1)*256;
			H_y+=(int)temp_char2;
			*/
			//store theta data
			temp_pt++;
			temp_char1=*temp_pt;
			temp_pt++;
			temp_char2=*temp_pt;
					 
			H_angle=((unsigned int)temp_char1)*256;
			H_angle+=(unsigned int)temp_char2;

			//store x data
			temp_pt++;
			temp_char1=*temp_pt;
			temp_pt++;
			temp_char2=*temp_pt;
	
			H_x= ((unsigned int)temp_char1)*256;
			H_x+=(unsigned int)temp_char2;
			
			//store y data
			temp_pt++;
			temp_char1=*temp_pt;
			temp_pt++;
			temp_char2=*temp_pt;

			H_y=((unsigned int)temp_char1)*256;
			H_y+=(unsigned int)temp_char2;

			temp_pt++;
			temp_char1=*temp_pt;
			temp_pt++;
			temp_char2=*temp_pt;

			Encoder_angle=((unsigned int)temp_char1)*256;
			Encoder_angle+=(unsigned int)temp_char2;

			temp_pt++;
			temp_char1=*temp_pt;
			temp_pt++;
			temp_char2=*temp_pt;

			Encoder_x= ((unsigned int)temp_char1)*256;
			Encoder_x+=(unsigned int)temp_char2;
		
			temp_pt++;
			temp_char1=*temp_pt;
			temp_pt++;
			temp_char2=*temp_pt;

			Encoder_y=((unsigned int)temp_char1)*256;
			Encoder_y+=(unsigned int)temp_char2;

			temp_pt++;
			temp_char1=*temp_pt;
			end_bit=(unsigned int)(temp_char1&0b11111111);
			
		
			//하기소닉 모듈에서 들어온 x,y,theta 데이터를 서울대 보드에서 fusion하는 변수에 저장
			Z_x=(double)(H_x)/1000.0;	
			Z_y=(double)(H_y)/1000.0;
			Z_a=(double)(H_angle)/100.0;
			//Z_q=cos(Z_q/2);
			
			m_visionUpdateFlag=1;
		}	
	}	
		
	hagisonic_i=0;
	
}

//#endif
/*
//이전에 구현한 하기소닉 모듈과 통신하기 위한 코드
unsigned int hagisonic_string_length=0;
void hagisonic_module()
{

	//unsigned int hagisonic_string_length=0;
	char * keyword_pt;
	unsigned char temp_char1, temp_char2;
	
	char * temp_pt;

	static int hagisonic_i=0;	
	while (USART_RXBufferData_Available(&USARTD0_data))
	{
		hagisonic_string[hagisonic_i] = USART_RXBuffer_GetByte(&USARTD0_data);
		hagisonic_i++;
		
	}	
	i_cnt=hagisonic_i;
	hagisonic_string[hagisonic_i]='\0';
	keyword_pt=hagisonic_string;

	while(*keyword_pt!='\0')
	{
		
		if(*keyword_pt==0x02)		//find start byte
		{
			hagisonic_string_length=strlen(keyword_pt);
			if(hagisonic_string_length>13)
			{				
				temp_pt=keyword_pt;
				if(*(temp_pt+13)==0x03)
				{
				//	temp_pt++;   // control byte 있을 때 필요
					//store theta data
					temp_char1=*temp_pt;
					start_bit=(unsigned int)(temp_char1&0b11111111);
					temp_pt++;
					temp_char1=*temp_pt;
					temp_pt++;
					temp_char2=*temp_pt;
										 
					H_angle= (unsigned int)(temp_char1&0b11111111)<<8;
					H_angle+=(unsigned int)(temp_char2&0b11111111);

					////store x data
					temp_pt++;
					temp_char1=*temp_pt;
					temp_pt++;
					temp_char2=*temp_pt;
						
					H_x= (unsigned int)(temp_char1&0b11111111)<<8;
					H_x+=(unsigned int)(temp_char2&0b11111111);

					//store y data
					temp_pt++;
					temp_char1=*temp_pt;
					temp_pt++;
					temp_char2=*temp_pt;

					H_y= (unsigned int)(temp_char1&0b11111111)<<8;
					H_y+=(unsigned int)(temp_char2&0b11111111);

					temp_pt++;
					temp_char1=*temp_pt;
					temp_pt++;
					temp_char2=*temp_pt;

					Encoder_angle= (unsigned int)(temp_char1&0b11111111)<<8;
					Encoder_angle+=(unsigned int)(temp_char2&0b11111111);
					
					temp_pt++;
					temp_char1=*temp_pt;
					temp_pt++;
					temp_char2=*temp_pt;

					Encoder_x= (unsigned int)(temp_char1&0b11111111)<<8;
					Encoder_x+=(unsigned int)(temp_char2&0b11111111);
							
					temp_pt++;
					temp_char1=*temp_pt;
					temp_pt++;
					temp_char2=*temp_pt;

					Encoder_y= (unsigned int)(temp_char1&0b11111111)<<8;
					Encoder_y+=(unsigned int)(temp_char2&0b11111111);
					
					temp_pt++;
					temp_char1=*temp_pt;
					end_bit=(unsigned int)(temp_char1&0b11111111);

						
					hagisonic_i=hagisonic_string_length-14;
					strcpy(hagisonic_string,keyword_pt+14);
					
					//현재 PC에서 들어오는 비전 데이터의 형태를 실제 값으로 변환
					Z_x=(double)(H_x)/1000.0;
					Z_y=(double)(H_y)/1000.0;
					Z_a=(double)(H_angle)/100.0;
					//Z_q=cos(Z_q/2);
					
					m_visionUpdateFlag=1;
				
					break;

				}
				else
				{
					keyword_pt++;			//end byte를 찾지 못해서 다시 start byte를 찾음
				}
			}
			else
			{
				hagisonic_i=hagisonic_string_length;		// length가 8 byte가 안되서 받은 데이터를 다 저장하고
				strcpy(hagisonic_string,keyword_pt);		// 그 뒤부터 다시 uart 통신으로 받아옴
				break;
			}
		}
		else
		{
			keyword_pt++;									//start byte를 못 찾아서 다음 byte로 넘어감.
		}
	}
	if(*keyword_pt=='\0')									// NULL값이 들어오면 초기화
	{
		hagisonic_i=0;
		hagisonic_string[0]='\0';
	}
		
		



}

*/

double m_dLeftInc_hagisonic=0;
double m_dRightInc_hagisonic=0;
int encoder_first=1;
char m_encoder_first_L=0;
char m_encoder_first_R=0;
char H_enc_Left_diff_first=0;
char H_enc_Right_diff_first=0;
char H_m_LeftEncoder_first=0;
char H_m_RightEncoder_first=0;
unsigned int H_m_LeftEncoder=0;
unsigned int H_m_RightEncoder=0;
double  m_dLeftInc_sum=0;
double  m_dRightInc_sum=0;

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
							
				
				H_m_LeftEncoder=m_LeftEncoder/100;
				H_m_RightEncoder=m_RightEncoder/100;
				
				while(H_m_LeftEncoder>255) H_m_LeftEncoder-=256;
				while(H_m_RightEncoder>255) H_m_RightEncoder-=256;
				
				
				if(encoder_first==1)
				{
					H_m_LeftEncoder_first=(char)(H_m_LeftEncoder);
					H_m_RightEncoder_first=(char)(H_m_RightEncoder);
					//encoder_first=0;
				}				
				
				H_encoderL_BM2=(char)((char)(H_m_LeftEncoder)-H_m_LeftEncoder_first);												
				H_encoderR_BM2=(char)((char)(H_m_RightEncoder)-H_m_RightEncoder_first);
				
				H_encoderR_BM2=-H_encoderR_BM2;

				if(encoder_first==1)
				{
					m_encoder_first_L=(char)((char)(m_LeftEncoder/256));
					m_encoder_first_R=(char)((char)(m_RightEncoder/256));
					//encoder_first=0;
				}
				
				H_encoderL=(char)((char)(m_LeftEncoder/256)-m_encoder_first_L);
				H_encoderR=(char)((char)(m_RightEncoder/256)-m_encoder_first_R);
								
				//H_encoderL=(char)(256-H_encoderL);
				//H_encoderR=(char)(256-H_encoderR);
			
				//H_encoderL=(char)(255-m_LeftEncoder/256);
				//H_encoderR=(char)(255-m_RightEncoder/256);

				//int lt=(int)(m_LeftEncoder-m_prevDLeft);
				lt=m_LeftEncoder -m_prevDLeft;
				rt=m_RightEncoder-m_prevDRight;
						
/*
				if(lt>32768)	m_dL=-(char)(65536-lt)/256;
				else 			m_dL=(double)lt/256;
				if(rt>32768)	m_dR=-(double)(65536-rt)/256;
				else 			m_dR=(double)rt/256;
*/								
				
				double m_dLeftInc_H=0;
				double m_dRightInc_H=0;
				
				if(lt>32768)	m_dLeftInc_H=-(double)(65536-lt);
				else 			m_dLeftInc_H=(double)lt;
				if(rt>32768)	m_dRightInc_H=-(double)(65536-rt);
				else 			m_dRightInc_H=(double)rt;

		
				
				if(lt>32768)	m_dLeftInc=-(double)(65536-lt)/REDUCTION_ENCODER;
				else 			m_dLeftInc=(double)lt/REDUCTION_ENCODER;
				if(rt>32768)	m_dRightInc=-(double)(65536-rt)/REDUCTION_ENCODER;
				else 			m_dRightInc=(double)rt/REDUCTION_ENCODER;
				
//				m_dLeftInc_sum +=m_dLeftInc;
//				m_dRightInc_sum+=m_dRightInc;

				double	H_enc_Left_diff=0;
				double	H_enc_Right_diff=0;
				
				H_enc_Left_diff =100.0/36.0*1000.0/1025.2*m_dLeftInc_H;	//예전에 100.0/16.1로 함.
				H_enc_Right_diff=100.0/36.0*1000.0/1025.2*m_dRightInc_H;
				
//				H_enc_Left_diff =150.0/55.4*m_dLeftInc_H;	//1.5 m 기준
//				H_enc_Right_diff=150.0/55.4*m_dRightInc_H;				

//				H_enc_Left_diff =100.0/16.1*m_dLeftInc_H;
//				H_enc_Right_diff=100.0/16.1*m_dRightInc_H;

//				H_enc_Left_diff =100.0/37.1*m_dLeftInc_H;// 36.9*m_dLeftInc_H;	//1 m 기준
//				H_enc_Right_diff=100.0/37.1*m_dRightInc_H;//36.9*m_dRightInc_H;

//				H_enc_Left_diff =m_dLeftInc_H;
//				H_enc_Right_diff=m_dRightInc_H;

				if(encoder_first==1)
				{
					H_enc_Left_diff_first=(char)((char)(H_enc_Left_diff/130.6));	// 130.6=78386/600
					H_enc_Right_diff_first=(char)((char)(H_enc_Right_diff/130.6));	//예전에 306.2로 함. (78386/256)
					
//					H_enc_Left_diff_first=(char)((char)(H_enc_Left_diff/306.2));
//					H_enc_Right_diff_first=(char)((char)(H_enc_Right_diff/306.2));

					encoder_first=0;
				}
				
				H_encoderL_sum+=H_enc_Left_diff;
				H_encoderR_sum+=H_enc_Right_diff;
				
				H_encoderL_BM=(char)((char)(H_encoderL_sum/130.6)-H_enc_Left_diff_first);
				H_encoderR_BM=(char)((char)(H_encoderR_sum/130.6)-H_enc_Right_diff_first);
				
//				H_encoderL_BM=(char)((char)(H_encoderL_sum/306.2)-H_enc_Left_diff_first);
//				H_encoderR_BM=(char)((char)(H_encoderR_sum/306.2)-H_enc_Right_diff_first);

				//H_encoderL_BM=(char)(H_encoderL_BM);
				//H_encoderR_BM=(char)(H_encoderR_BM);
				
				//H_encoderR_BM=H_encoderR_BM;
			
				/*
				if(lt>32768)	m_dLeftInc_hagisonic=-(double)(65536-lt)/45.0;
				else 			m_dLeftInc_hagisonic=(double)lt/45.0;
				if(rt>32768)	m_dRightInc_hagisonic=-(double)(65536-rt)/45.0;
				else 			m_dRightInc_hagisonic=(double)rt/45.0;

				Encoder_diff_Left=(char)m_dLeftInc_hagisonic;
				Encoder_diff_Right=(char)m_dRightInc_hagisonic;
				*/	
				m_dLeftInc_hagisonic=m_dLeftInc/0.00037699;
				m_dRightInc_hagisonic=m_dRightInc/0.00037699;

				//m_dLeftInc_hagisonic +=m_dLeftInc;
				//m_dRightInc_hagisonic+=m_dRightInc;

				Encoder_diff_Left=(char)m_dLeftInc_hagisonic;
				Encoder_diff_Right=(char)m_dRightInc_hagisonic;
					
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
#define SLIP_V_TH	0.2
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

	//double AccDifference=(XLSum[0]/10)-
	//double Vdif[2]={Vacc_x-Venc_x,Vacc_y-0};
	double Vdif[2]={Vacc_x-Venc_x,Vacc_y-0};
	//if(state ==0){	
		for(i=0;i<2;i++)
		{
			if(Vdif[i]>SLIP_V_TH || Vdif[i]<-SLIP_V_TH)		slip_cnt[i]++;//state= -1;//slip
			else	slip_cnt[i]=0;		

			if(slip_cnt[i]>SLIP_CNT_TH)	state=-1;
		}
	//}
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
	USART_InterruptDriver_Initialize(&USARTD0_data, &USARTD0, USART_DREINTLVL_HI_gc);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USARTD0_data.usart, USART_RXCINTLVL_HI_gc);

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
				int i,j,k;	

				m_XXL=-SCALE_XXL*((double)XXL-(double)m_biasXXL);
				m_YXL=-SCALE_YXL*((double)YXL-(double)m_biasYXL);
				m_ZXL=SCALE_ZXL*((double)ZXL-(double)m_biasZXL);
				m_XGY=-SCALE_XGY*((double)XGY-(double)m_biasXGY);//-0.001212142/0.00015711
				m_YGY=SCALE_YGY*((double)YGY-(double)m_biasYGY);//+
				//if(ZGY<3000)	m_ZGY=SCALE_ZGY*((double)ZGY-(double)m_biasZGY+65536.0);
				//else 			m_ZGY=SCALE_ZGY*((double)ZGY-(double)m_biasZGY);//+
				m_ZGY=SCALE_ZGY*((double)ZGY-(double)m_biasZGY);
				
				sprintf(XYZ_buffer,"%u %u %u %u %u %u \n",XXL, YXL, ZXL, XGY, YGY, ZGY);
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

