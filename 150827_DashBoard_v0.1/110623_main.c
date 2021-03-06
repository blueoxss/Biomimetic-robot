
#include "spi_driver.h"
#include "usart_driver.h"
#include "avr_compiler.h"
#include <math.h>
#include <string.h>

#define USART USARTD0

#define CLKSYS_IsReady( _oscSel ) ( OSC.STATUS & (_oscSel) )



// 알고리즘 MACRO
#define TARGET_X	2//2.2//0.96//2.02
#define TARGET_Y	0//0//1.00//1.62//0.0//1.76//1.86-0.1
#define TARGET_Z	0.2//-10.0000///-0.035//-0.075//0.26//0.029//

#define distance_th 3.5

#define SCALE_XXL	1.0
#define SCALE_YXL	1.0
#define SCALE_ZXL	1.0
#define SCALE_XGY	0.00015271631//8.75mDegree/sec*digit//-0.001214142//1/14.375*pi/180
#define SCALE_YGY	0.00015271631//0.001214142
#define SCALE_ZGY	0.00015271631//0.001214142

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
 
#define	CAM_OA				0.4799654//광각 50도/2,in radian    

#define IMAGE_X				573
#define IMAGE_Y				67
#define PIXEL_ERROR_SQ_TH	0
#define CAM_DELAY_FACTOR	4
#define VISION_FREQ_FACTOR	19


#define Tsample 10
#define Bsample 100

volatile unsigned char Acc_read=0;
//static unsigned int XXL_u,YXL_u,ZXL_u,XGY_u,YGY_u,ZGY_u;
//static unsigned int timer=0;
//

	unsigned int m_biasXXL;
	unsigned int m_biasYXL;
	unsigned int m_biasZXL;
	unsigned int m_biasXGY;
	unsigned int m_biasYGY;
	unsigned int m_biasZGY;

	double m_samplingTime;

	double m_targetX_robot;
	double m_targetY_robot;
	double m_targetZ_robot;

double m_sensorPanAngle;
double m_sensorTiltAngle;

	double m_panAngleVisionError;
	double m_tiltAngleVisionError;
	
	double m_panAngleVision;
	double m_tiltAngleVision;

	double m_panAngle;
	double m_tiltAngle;

	double m_panAngleOld;
	double m_tiltAngleOld;
	double m_panEncoder;
	double m_tiltEncoder;

	double panEncoder;
	double tiltEncoder;


unsigned char host_string[100];

	int int_temp=0;

int m_dLeftInc;
int m_LeftEncoder;
int m_prevDLeft;
int m_dRightInc;
int m_RightEncoder;
int m_prevDRight;


//function
int read_sensor_data(unsigned int *xxl,unsigned int *yxl,unsigned int *zxl);
void clk_init(void);
void port_init(void);
void interrupt_init(void);
void Init_L3G4200DH(void);
void Init_LIS331DLH(void);
void timer_Init(void);
void int_init(void);
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
uint8_t masterSendData_gyro[NUM_BYTES] = {0xe8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t masterSendData_gyro_init[NUM_BYTES] ={0x60, 0x3f, 0x00, 0x00, 0x80, 0x00, 0x00}; 

/*! \brief Data received from Accelerometer */
uint8_t ACC_DATA[NUM_BYTES];
/*! \brief Data received from Gyroscope */
uint8_t GYRO_DATA[NUM_BYTES];

/*! \brief Result of the test. */
bool success = true;

/* Instantiate pointer to ssPort. */
PORT_t *ssPort = &PORTC;

/*! USART data struct used in example. */
USART_data_t USART_data;

/*! Array to put received data in. */
uint8_t receiveArray[NUM_BYTES];


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

ISR(TCC1_OVF_vect)
{
	samplingFlag=1;
	sei();
}

ISR(USARTD0_RXC_vect)
{
	if(!USART_RXBufferData_Available(&USART_data))
	{
		USART_InterruptDriver_Initialize(&USART_data, &USARTD0, USART_DREINTLVL_LO_gc);
	}

	USART_RXComplete(&USART_data);
}



void timer_Init(void){
	
	TCC0.CTRLA=0x04;//prescaler : clk/8
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
	TCC1.CTRLB=0x00;
	TCC1.CTRLC=0x00;
	TCC1.CTRLD=0x00;
	TCC1.CTRLE=0x00;
	TCC1.INTCTRLA=0x03;
	TCC1.INTCTRLB=0x00;//interrupt 관련 부분, 다시 체크할것
	TCC1.INTFLAGS=0x01;//마지막 bit가 OVFIF: Overflow/Underflow Interrupt Flag
	TCC1.PERH=0b10011100;//PER 가 period를 조절할 수 있는 부분임
	TCC1.PERL=0b01000000;
}

void int_init(void){
	
	
	PMIC.INTPRI=0x00;
	PMIC.CTRL=0x07;//High level interrupt enable	
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
		
		read_gyro_data(&XGY_u, &YGY_u, &ZGY_u);
		read_sensor_data(&XXL_u, &YXL_u, &ZXL_u);

		sprintf((char*)XYZ_buffer,"[XXL%6u][YXL%6u][ZXL%6u][XGY%6u][YGY%6u][ZGY%6u]\r\n",XXL_u,YXL_u,ZXL_u,XGY_u,YGY_u,ZGY_u);
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

	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);

	/* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USARTD0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 115200 bps:
	 * Use the default I/O clock fequency that is 16 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USARTD0, 8 , 0);
//	USART_Baudrate_Set(&USARTD0, 8 , 1);


	/* Enable both RX and TX. */
	USART_Rx_Enable(&USARTD0);
	USART_Tx_Enable(&USARTD0);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;

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



void position_estimator(unsigned int XXL,unsigned int YXL,unsigned int ZXL,unsigned int XGY,unsigned int YGY,unsigned int ZGY)
{

	unsigned char XYZ_buffer[100];
//	double m_prevCounter;
//	double m_counter;

	double m_XXL;
	double m_YXL;
	double m_ZXL;
	double m_XGY;
	double m_YGY;
	double m_ZGY;

//	double m_XXL_sum;
//	double m_YXL_sum;
//	double m_ZXL_sum;
//	double m_XGY_sum;
//	double m_YGY_sum;
//	double m_ZGY_sum;

//	double m_angleX=0;
//	double m_angleY=0;
//	double m_angleZ=0;
	static double m_psi=0.0;
	static double m_theta=0.0;
	static double m_phi=0.0;

	double m_targetX=TARGET_X;
	double m_targetY=TARGET_Y;
	double m_targetZ=TARGET_Z;

	static double XGY_old;
	static double YGY_old;
	static double ZGY_old;

	double m_pX=0;
	double m_pY=0;
	double m_pZ=0;
//	double m_panAngle;
//	double m_tiltAngle;
//	double m_targetDistance;

//		double sensorPanAngle=0;
		double prevPsi=m_psi;
		double prevTheta=m_theta;
		double prevPhi=m_phi;
//		int dCount;
		double scale;

		double R11;
		double R12;
		double R13;
		double R21;
		double R22;
		double R23;
		double R31;
		double R32;
		double R33;

double tempX;
double tempY;
double tempZ;

//encoder
 
		// 현재 위치 , heading angle 계산
/*		m_dLeftInc=m_LeftEncoder-m_prevDLeft;
		if(m_dLeftInc>SIXTEENBITS/2)
				m_dLeftInc=m_dLeftInc-SIXTEENBITS;
		else if(m_dLeftInc<-SIXTEENBITS/2)
				m_dLeftInc=SIXTEENBITS+m_dLeftInc;
		m_dLeftInc  = (double) m_dLeftInc/(double) REDUCTION_ENCODER;
		
		m_dRightInc=m_RightEncoder -m_prevDRight;
		if(m_dRightInc>SIXTEENBITS/2)
				m_dRightInc=m_dRightInc-SIXTEENBITS;
		else if(m_dRightInc<-SIXTEENBITS/2)
				m_dRightInc=SIXTEENBITS+m_dRightInc;
		m_dRightInc  = (double) m_dRightInc/(double)REDUCTION_ENCODER;

		m_prevDLeft = m_LeftEncoder;
		m_prevDRight = m_RightEncoder;
*/



		m_XXL=SCALE_XXL*(double)(XXL-m_biasXXL);
		m_YXL=SCALE_YXL*(double)(YXL-m_biasYXL);
		m_ZXL=SCALE_ZXL*(double)(ZXL-m_biasZXL);
		m_XGY=SCALE_XGY*(double)(0.5*(XGY+XGY_old)-m_biasXGY);//-0.001212142/0.00015711
		m_YGY=SCALE_YGY*(double)(0.5*(YGY+YGY_old)-m_biasYGY);//+
		m_ZGY=SCALE_ZGY*(double)(0.5*(ZGY+ZGY_old)-m_biasZGY);//+

		XGY_old=XGY;
		YGY_old=YGY;
		ZGY_old=ZGY;

		if(cos(m_theta)==0){
			m_psi=prevPsi;
			m_theta=prevTheta;
			m_phi=prevPhi;
		}

		else{
			scale=1/cos(m_theta);
			m_psi=prevPsi+scale*(-sin(prevPhi)*m_XGY+cos(prevPhi)*m_ZGY)*m_samplingTime;
			m_theta=prevTheta+scale*(cos(prevTheta)*cos(prevPhi)*m_XGY+cos(prevTheta)*sin(prevPhi)*m_ZGY)*m_samplingTime;
			m_phi=prevPhi+scale*(sin(prevTheta)*sin(prevPhi)*m_XGY+cos(prevTheta)*m_YGY-sin(prevTheta)*cos(prevPhi)*m_ZGY)*m_samplingTime;
		}


		R11=cos(m_psi)*cos(m_phi)-sin(m_psi)*sin(m_theta)*sin(m_phi);
		R12=sin(m_psi)*cos(m_phi)+cos(m_psi)*sin(m_theta)*sin(m_phi);
		R13=-cos(m_theta)*sin(m_phi);
		R21=-sin(m_psi)*cos(m_theta);
		R22=cos(m_psi)*cos(m_theta);
		R23=sin(m_theta);
		R31=cos(m_psi)*sin(m_phi)+sin(m_psi)*sin(m_theta)*cos(m_phi);
		R32=sin(m_psi)*sin(m_phi)-cos(m_psi)*sin(m_theta)*cos(m_phi);
		R33=cos(m_theta)*cos(m_phi);

		// position update
		// the inverse of R = the transpose of R
		
		m_pX+=(m_dLeftInc+m_dRightInc)/2.0*R11;
		m_pY+=(m_dLeftInc+m_dRightInc)/2.0*R12;
		m_pZ+=(m_dLeftInc+m_dRightInc)/2.0*R13;
		

		//Target position update
		
		

		//3축 병진 변위 고려
		m_targetX_robot=m_targetX-m_pX;
		m_targetY_robot=m_targetY-m_pY;
		m_targetZ_robot=m_targetZ-m_pZ;

//		distance=sqrt(m_targetX_robot*m_targetX_robot + m_targetY_robot*m_targetY_robot + m_targetZ_robot*m_targetZ_robot);

		//3축 회전 변위 고려
		tempX=m_targetX_robot;
		tempY=m_targetY_robot;
		tempZ=m_targetZ_robot;

		m_targetX_robot=R11*tempX + R12*tempY + R13*tempZ-0.125;
		m_targetY_robot=R21*tempX + R22*tempY + R23*tempZ;//0.1은 로봇 중심에서 모터의 위치
		m_targetZ_robot=R31*tempX + R32*tempY + R33*tempZ-0.165;
		//비전만 포함시킬려면 이 부분을 주석처리 해야함
		//pan, tilt angle 결정
		m_sensorPanAngle=atan2(m_targetY_robot, m_targetX_robot);//-PI/2.0; // CCW가 +
		m_sensorTiltAngle=atan2(-m_targetZ_robot,sqrt(m_targetX_robot*m_targetX_robot+m_targetY_robot*m_targetY_robot)); //0.061156;//위를 보는게 +
		//m_tiltAngle=atan(TARGET_Z/sqrt(TARGET_X*TARGET_X+TARGET_Y*TARGET_Y))+0.035;

		//m_sensorPanAngle=0;//비전만 이용하지 않고 복합하려면 이부분을 주석처리해야함
		//m_sensorTiltAngle=0;//위와동일
		
		//if(m_sensorAngleBufferIndex>99)	AfxMessageBox(_T("!!!!!!!!!!!!!"));

//		m_angleX+=m_XGY*m_samplingTime;
//		m_angleY+=m_YGY*m_samplingTime;
//		m_angleZ+=m_ZGY*m_samplingTime;



		m_panAngle  = m_sensorPanAngle + m_panAngleVision  - m_panAngleOld;
		m_tiltAngle = m_sensorTiltAngle+ m_tiltAngleVision - m_tiltAngleOld;

	return;
}

int double_ints(double x, int mode)//mode 가 0 이면 x의 정수부를 return, mode가 1이면 x의 소수부를 return
{          
                     int x1,x2;
                     if(x>=0)//x=x1(정수부)+ x2(소수부)
                                {
                                          x1=(int) x;
                                          x2=(int) ((x-x1)*10000);//소숫점 아래 6자리를 표현하기 위함임
                                }
                     else
                     {
                                x1=(int) x;
                                x2=(int)((x1-x)*10000);
                     }

                     if(mode == 0)    return x1;
                     if(mode == 1)    return x2;
                     else return 0;
}



int main(void)
{
	unsigned int XXL,YXL,ZXL,XGY,YGY,ZGY;
	unsigned char XYZ_buffer[100];
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
	
	int i=0;
	 unsigned int host_string_length=0;
	 char * keyword_pt;
	 unsigned char temp_char1, temp_char2, temp_char3;
	
	char * temp_pt;
	int enc_left=0;
	int enc_left_old=0;
	int enc_right,enc_right_old;
	int pixX,image_x_old;
	int pixY,image_y_old;

	int m_panPixelError;
	int m_tiltPixelError;


	clk_init();
	port_init();
 	USART_INIT();
	
	timer_Init();
	int_init();
//	sei();
	

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


	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_data, &USARTD0, USART_DREINTLVL_LO_gc);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;

	/* Enable global interrupts. */
	sei();

	/* Initialize ACC & Gyro */
//	Init_L3G4200DH();
	Init_LIS331DLH();

		SPI_MasterCreateDataPacket(&dataPacket,
		                           masterSendData_gyro_init,
		                           GYRO_DATA,
		                           NUM_BYTES,
		                           &PORTC,
		                           PIN1_bm);

		SPI_MasterSSLow(ssPort, PIN1_bm);

		/* Transceive packet. */
		SPI_MasterTransceivePacket(&spiMasterC, &dataPacket);
		/* MASTER: Release SS to slave. */

		SPI_MasterSSHigh(ssPort, PIN1_bm);

	/* Read Sensor data */

sprintf((char*)host_string,"[E123,456,xx,yy]");
	while(true) 
	{

		if(samplingFlag)
	    {
			samplingFlag=0;
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
			                           masterSendData_gyro,
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

			timer=TCC0.CNT;
			diff_counter=timer-timer_old;


			ACC_DATA[2]=ACC_DATA[2]+0x80;
			ACC_DATA[4]=ACC_DATA[4]+0x80;
			ACC_DATA[6]=ACC_DATA[6]+0x80;
			XXL= (unsigned int)ACC_DATA[2]*256+ACC_DATA[1];
			YXL= (unsigned int)ACC_DATA[4]*256+ACC_DATA[3];
			ZXL= (unsigned int)ACC_DATA[6]*256+ACC_DATA[5];

			GYRO_DATA[2]=GYRO_DATA[2]+0x80;
			GYRO_DATA[4]=GYRO_DATA[4]+0x80;
			GYRO_DATA[6]=GYRO_DATA[6]+0x80;
			XGY= (unsigned int)GYRO_DATA[2]*256+GYRO_DATA[1];
			YGY= (unsigned int)GYRO_DATA[4]*256+GYRO_DATA[3];
			ZGY= (unsigned int)GYRO_DATA[6]*256+GYRO_DATA[5];


			timer_old=timer;

			m_samplingTime=(double)diff_counter/2000000;
			//sprintf((char*)XYZ_buffer,"[TIM%6u]\n\r",timer);		

		        //bias
		        //
			if(m_IMU_count<Tsample)
			{
				m_IMU_count++;
			}
			else if(m_IMU_count<Tsample+Bsample)
			{
				m_XXL_sum+=XXL;
				m_YXL_sum+=YXL;
				m_ZXL_sum+=ZXL;
				m_XGY_sum+=XGY;
				m_YGY_sum+=YGY;
				m_ZGY_sum+=ZGY;
				m_IMU_count++;
			}
			else if(m_IMU_count==Tsample+Bsample)
			{
				//SetTimer(25,1,NULL);
				m_biasXXL=m_XXL_sum/Bsample;
				m_biasYXL=m_YXL_sum/Bsample;
				m_biasZXL=m_ZXL_sum/Bsample;
				m_biasXGY=m_XGY_sum/Bsample;
				m_biasYGY=m_YGY_sum/Bsample;
				m_biasZGY=m_ZGY_sum/Bsample;
				m_IMU_count++;
		    }
		    else
		    { 

				position_estimator(XXL,YXL,ZXL,XGY,YGY,ZGY);
	//			sprintf((char*)XYZ_buffer,"[TIM%6u][XXL%6u][YXL%6u][ZXL%6u][XGY%6u][YGY%6u][ZGY%6u]\n\r",timer,XXL,YXL,ZXL,XGY,YGY,ZGY);
	//			sprintf((char*)XYZ_buffer,"[TIM%lf][XXL%lf][YXL%f][ZXL%6f][XGY%6f][YGY%6f][ZGY%6u]\n\r",m_samplingTime,m_targetX_robot,m_targetY_robot,m_targetZ_robot,m_sensorPanAngle,m_sensorTiltAngle,ZGY);
	//			sprintf((char*)XYZ_buffer,"[TIM%.5f][XXL%u]\n\r",0.0001,ZGY);

				sprintf((char*)XYZ_buffer,"[X_axis %2d.%5d][Y_axis %2d.%5d]][Z_axis %2d.%5d]\n\r",double_ints(m_samplingTime,0),double_ints(m_samplingTime,1),double_ints(m_targetX_robot,0),double_ints(m_targetX_robot,1),double_ints(m_targetY_robot,0),double_ints(m_targetY_robot,1));   
	
				uartSendTX(XYZ_buffer);

				while (USART_RXBufferData_Available(&USART_data))
				{
						host_string[i] = USART_RXBuffer_GetByte(&USART_data);
						i++;
				}
//				sprintf((char*)host_string,"LoL\n\r");

//				host_string[i]='\n';
//				host_string[i+1]='\r';
//				host_string[i+2]='\0';


				keyword_pt=strstr( host_string, "[E");	

				temp_pt=keyword_pt;		
				host_string_length=strlen(keyword_pt);

				if(host_string_length>15)
				{
					temp_pt++;
					temp_pt++;
					temp_char1=*temp_pt;
					temp_pt++;
					temp_char2=*temp_pt;
					temp_pt++;
					temp_char3=*temp_pt;
//					enc_left=temp_char1<<16+temp_char2<<8+temp_char3;
					m_LeftEncoder=(int)temp_char2<<8;
					m_LeftEncoder+=(int)temp_char3;
//					enc_right=(keyword_pt+6)*256*256+(keyword_pt+7)*256+(keyword_pt+8);
					temp_pt++;
					temp_pt++;
					temp_char1=*temp_pt;
					temp_pt++;
					temp_char2=*temp_pt;
					temp_pt++;
					temp_char3=*temp_pt;
					m_RightEncoder=(int)temp_char2<<8;
					m_RightEncoder+=(int)temp_char3;
//					image_x=(keyword_pt+10)*256+(keyword_pt+11);
					temp_pt++;
					temp_pt++;
					temp_char1=*temp_pt;
					temp_pt++;
					temp_char2=*temp_pt;
					pixX=(int)temp_char1<<8;
					pixX+=(int)temp_char2;

					temp_pt++;
					temp_pt++;
					temp_char1=*temp_pt;
					temp_pt++;
					temp_char2=*temp_pt;
					pixY=(int)temp_char1<<8;
					pixY+=(int)temp_char2;
/*					if(((keyword_pt+5)==(keyword_pt+9)==(keyword_pt+12)==',')&&((keyword_pt+15)==']'))
					{
					}
					else
					{
							enc_left=enc_left_old;
							enc_right=enc_right_old;
							image_x=image_x_old;
							image_y=image_y_old;
					}
*/
					i=host_string_length-16;
					strcpy(host_string,keyword_pt+16);

			//encoder
					m_dLeftInc=m_LeftEncoder-m_prevDLeft;
					if(m_dLeftInc>SIXTEENBITS/2)
							m_dLeftInc=m_dLeftInc-SIXTEENBITS;
					else if(m_dLeftInc<-SIXTEENBITS/2)
							m_dLeftInc=SIXTEENBITS+m_dLeftInc;
					m_dLeftInc  = (double) m_dLeftInc/(double) REDUCTION_ENCODER;
		
					m_dRightInc=m_RightEncoder -m_prevDRight;
					if(m_dRightInc>SIXTEENBITS/2)
							m_dRightInc=m_dRightInc-SIXTEENBITS;
					else if(m_dRightInc<-SIXTEENBITS/2)
							m_dRightInc=SIXTEENBITS+m_dRightInc;
					m_dRightInc  = (double) m_dRightInc/(double)REDUCTION_ENCODER;

					m_prevDLeft = m_LeftEncoder;
					m_prevDRight = m_RightEncoder;

					if(pixX*pixX+pixY*pixY>1000){
						m_panPixelError=pixX;
						m_tiltPixelError=pixY;
						m_panAngleVisionError  = - atan2(m_panPixelError, 160.0/tan(CAM_OA));		
						m_tiltAngleVisionError  = atan2(m_tiltPixelError, 160.0/tan(CAM_OA));
			
						m_panAngleVision = m_panAngleVisionError +panEncoder;				
						m_tiltAngleVision = m_tiltAngleVisionError +tiltEncoder;
//						m_panAngleOld=panAngleOld;				
//						m_tiltAngleOld=tiltAngleOld;
					}

				}


				sprintf((char*)XYZ_buffer,"Counter %6d\n\r",i);
				uartSendTX(host_string);
				uartSendTX(XYZ_buffer);

//				i=0;
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
