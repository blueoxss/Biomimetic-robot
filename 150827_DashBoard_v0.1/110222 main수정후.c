double taeilBuf[2];
#include "spi_driver.h"
#include "usart_driver.h"
#include "avr_compiler.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "Matrix.h"

#define USART USARTD0

#define CLKSYS_IsReady( _oscSel ) ( OSC.STATUS & (_oscSel) )

#define OKR_ONLY 1


//Vision 27Hz, 1 delay
#define ENCODER_INDEX 5		//4~6
#define VISION_LOOP_CNT 4
#define VISION_BUF_SIZE 1

/*
//Vision 27/2Hz, 1 delay
#define ENCODER_INDEX 7		//6~8
#define VISION_LOOP_CNT 8
#define VISION_BUF_SIZE 2
*/
/*
//Vision 27/4Hz, 1 delay
#define ENCODER_INDEX 11	//10~12
#define VISION_LOOP_CNT 16
#define VISION_BUF_SIZE 4
*/
/*
//Vision 27/8Hz, 1 delay
#define ENCODER_INDEX 19	//18~20
#define VISION_LOOP_CNT 32
#define VISION_BUF_SIZE 8
*/

// 알고리즘 MACRO
#define TARGET_X 	2.44//2.2//0.96//2.02
#define TARGET_Y	-0.75//0//1.00//1.62//0.0//1.76//1.86-0.1
#define TARGET_Z	0.2//-10.00///-0.035//-0.075//0.26//0.029//




#define FF_GAIN 10
#define DELAY_COMP_G 10
#define DELAY_COMP_E 5
#define DELAY_COMP_D 1
#define VE_GAIN 2.0
#define ACC_FF_LPF 3
#define ACC_FF_GAIN	-0.001
#define VIS_SIZE_BUF 2

#define  L_CA 0.0//0.035 ///Camera-Actuator 길이
#define  L_AR 0.235 ///Robot-Actuator z성분 길이	20.5+3
#define  L_RR 0.165 ///Robot-Actuator x성분 길이



#define distance_th 3.5

#define SCALE_XXL	0.0006125			// FS=00      1/16 mg/digit,0.0098/16(m/s^2)/digit
#define SCALE_YXL	0.0006125
#define SCALE_ZXL	0.0006125
#define SCALE_XGY	0.000146764//0.00015271631//8.75mDegree/sec*digit//-0.001214142//1/14.375*pi/180
#define SCALE_YGY	0.000146764//0.00015271631//0.001214142
#define SCALE_ZGY	0.000146764//0.00015271631//0.001214142
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
 
//#define	CAM_OA				0.4799654//광각 50도/2,in radian    
#define	CAM_OA				0.40//광각 50도/2,in radian 

#define IMAGE_X				573
#define IMAGE_Y				67
#define PIXEL_ERROR_SQ_TH	0
#define CAM_DELAY_FACTOR	4
#define VISION_FREQ_FACTOR	19

#define MOTOR_ENCODER 7168.0

#define Tsample 100
#define Bsample 500



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

	double m_samplingTime;

	double m_targetX_robot=TARGET_X;
	double m_targetY_robot=TARGET_Y;
	double m_targetZ_robot=TARGET_Z;

double m_sensorPanAngle;
double m_sensorTiltAngle;

	double m_panAngleVisionError;
	double m_tiltAngleVisionError;
	
	double m_panAngleVision;
	double m_tiltAngleVision;

	double m_panAngle;
	double m_tiltAngle;
	double panEncoderBuffer[ENCODER_INDEX]={0,};
	double tiltEncoderBuffer[ENCODER_INDEX]={0,};

	double m_panEncoder=0;
	double m_tiltEncoder=0;

	double panEncoder=0;
	double tiltEncoder=0;

	double m_panPixelError;
	double m_tiltPixelError;

char host_string[300];
char pan_encoder_string[50];
char tilt_encoder_string[50];
double gravityVect[3];

	int int_temp=0;

double m_dLeftInc=0;
unsigned int m_LeftEncoder=0;
unsigned int m_prevDLeft=0;
double  m_dRightInc=0;
unsigned int  m_RightEncoder=0;
unsigned int  m_prevDRight=0;
double m_distance[DELAY_COMP_D];
int m_visionUpdateFlag=0;
int loop_counter=0;
//function
int read_sensor_data(unsigned int *xxl,unsigned int *yxl,unsigned int *zxl);
void clk_init(void);
void port_init(void);
void interrupt_init(void);
void Init_L3G4200DH(void);
void Init_LIS331DLH(void);
void timer_Init(void);
void int_init(void);
double vel_KF_update();
/*! \brief The number of test data bytes. */
#define NUM_BYTES     7




double m_XGY_buf[DELAY_COMP_G];
double m_YGY_buf[DELAY_COMP_G];
double m_ZGY_buf[DELAY_COMP_G];
double m_enc_buf[DELAY_COMP_E];
double ve_gain=0;
double m_enc_sum=0;
int indexD=0;
int robotState=0;

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
//uint8_t masterSendData_vision[4] = {0x00, 0x01, 0x02, 0x03};
uint8_t masterSendData_vision[4] = {0x00, 0x01, 0x02, 0x03};

/*! \brief Data received from Accelerometer */
uint8_t ACC_DATA[NUM_BYTES];
/*! \brief Data received from Gyroscope */
uint8_t GYRO_DATA[NUM_BYTES];
/*! \brief Data received from Vision Board */
uint8_t VISION_DATA[4];

/*! \brief Result of the test. */
bool success = true;

/* Instantiate pointer to ssPort. */
PORT_t *ssPort = &PORTC;

/*! USART data struct used in example. */
USART_data_t USARTD0_data;

USART_data_t USARTE0_data;
USART_data_t USARTF0_data;

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

ISR(TCC1_OVF_vect)
{
	samplingFlag=1;
//	sei();
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

ISR(USARTF0_RXC_vect)
{
//	if(!USART_RXBufferData_Available(&USARTD0_data))
//	{
//		USART_InterruptDriver_Initialize(&USARTD0_data, &USARTD0, USART_DREINTLVL_LO_gc);
//	}
//
	USART_RXComplete(&USARTF0_data);
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
	TCC1.INTCTRLA=0x02;
	TCC1.INTCTRLB=0x00;//interrupt 관련 부분, 다시 체크할것
	TCC1.INTFLAGS=0x01;//마지막 bit가 OVFIF: Overflow/Underflow Interrupt Flag

	TCC1.PERH=0x6F;
	TCC1.PERL=0x53;	//108Hz
//	TCC1.PERH=0b10011100;
//	TCC1.PERL=0b01000000; //100Hz
	//TCC1.PERH=0b01101000;//PER 가 period를 조절할 수 있는 부분임
	//TCC1.PERL=0b00101011; //150Hz

	
}

void int_init(void){
	
	
	PMIC.INTPRI=0x00;
	PMIC.CTRL=0x07;//High level interrupt enable	
}


void port_init(void){
	
	PORTA.OUT=0x00;
	PORTA.DIR=0x00;

//	PORTB.OUT=
//	PORTB.DIR=

	PORTC.OUT=0xa7;
	PORTC.DIR= PORTC.DIR | 0xa7;
	
	PORTD.DIRSET = PIN3_bm;
	PORTD.DIRCLR = PIN2_bm;

/*	PORTE.OUT=
	PORTE.DIR=

	PORTF.OUT=
	PORTF.DIR=
	
	PORTH.OUT=
	PORTH.DIR=
*/	
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
	_delay_us(10);
	SPI_MasterTransceiveByte(&spiMasterC, 0x20);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	_delay_us(10);
	SPI_MasterSSHigh(ssPort, PIN0_bm);
	_delay_us(10);
  //CTRL_REG2
  //BOOT HPM1 HPM0 FDS HPen2 HPen1 HPCF1 HPCF0
  //0    0   0  0   0          0          00
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	_delay_us(10);
	SPI_MasterTransceiveByte(&spiMasterC, 0x21);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	_delay_us(10);
	SPI_MasterSSHigh(ssPort, PIN0_bm);
	_delay_us(10);
  //CTRL_REG3
  //IHL PP_OD LIR2 I2_CFG1 I2_CFG0 LIR1 I1_CFG1 I1_CFG0
  //0    0      0    0       0      0      1       0
  write_register=0x02;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	_delay_us(10);
	SPI_MasterTransceiveByte(&spiMasterC, 0x22);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	_delay_us(10);
	SPI_MasterSSHigh(ssPort, PIN0_bm);
	_delay_us(10);
  //CTRL_REG4
  //BDU BLE FS1 FS0 STsign 0 ST SIM
  //1    0   0   0    0    0  0  0
  write_register=0x80;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	_delay_us(10);
	SPI_MasterTransceiveByte(&spiMasterC, 0x23);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	_delay_us(10);
	SPI_MasterSSHigh(ssPort, PIN0_bm);
	_delay_us(10);
  //CTRL_REG5
  //0 0 0 0 0 0 TurnOn1 TurnOn0
  //0 0 0 0 0 0    0       0 
  write_register=0x00;
	SPI_MasterSSLow(ssPort, PIN0_bm);
	_delay_us(10);
	SPI_MasterTransceiveByte(&spiMasterC, 0x24);
	SPI_MasterTransceiveByte(&spiMasterC, write_register);
	_delay_us(10);
	SPI_MasterSSHigh(ssPort, PIN0_bm);

}




void USART_INIT(void)
{

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
//	USART_Baudrate_Set(&USARTD0, 8 , 0);

	USART_Baudrate_Set(&USARTD0, 16 , 1);
	USARTD0.CTRLB|=0x04;                      //CLK2X


	/* Enable both RX and TX. */
//	USART_Rx_Enable(&USARTD0);
//	USART_Tx_Enable(&USARTD0);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;



	/* This PORT setting is only valid to USARTE0 if other USARTs is used a
	 * different PORT and/or pins is used. */
	/* PIN3 (TXD0) as output. */
	PORTE.DIRSET = PIN3_bm;

	/* PC2 (RXD0) as input. */
	PORTE.DIRCLR = PIN2_bm;

	/* Use USARTE0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USARTE0_data, &USARTE0, USART_DREINTLVL_LO_gc);

	/* USARTE0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USARTE0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USARTE0_data.usart, USART_RXCINTLVL_LO_gc);

	USART_Baudrate_Set(&USARTE0, 16 , 1);

	USARTE0.CTRLB|=0x04;

	/* Enable both RX and TX. */
	USART_Rx_Enable(&USARTE0);
	USART_Tx_Enable(&USARTE0);

	/* Enable PMIC interrupt level low. */
//	PMIC.CTRL |= PMIC_LOLVLEX_bm;



	/* This PORT setting is only valid to USARTF0 if other USARTs is used a
	 * different PORT and/or pins is used. */
	/* PIN3 (TXD0) as output. */
	PORTF.DIRSET = PIN3_bm;

	/* PC2 (RXD0) as input. */
	PORTF.DIRCLR = PIN2_bm;

	/* Use USARTF0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USARTF0_data, &USARTF0, USART_DREINTLVL_LO_gc);

	/* USARTF0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USARTF0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USARTF0_data.usart, USART_RXCINTLVL_LO_gc);

	USART_Baudrate_Set(&USARTF0, 16 , 1);
	USARTF0.CTRLB|=0x04;

	/* Enable both RX and TX. */
	USART_Rx_Enable(&USARTF0);
	USART_Tx_Enable(&USARTF0);

	/* Enable PMIC interrupt level low. */
//	PMIC.CTRL |= PMIC_LOLVLEX_bm;
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
	
}void uartF0SendTXbit(unsigned char data){

	do{}while(!USART_IsTXDataRegisterEmpty(&USARTF0));
		USART_PutChar(&USARTF0, data);

}

void uartF0SendTX(unsigned char *string){
	int i=-1;
	do{
		uartF0SendTXbit(string[++i]);
	}
	while(string[i] != '\0');
	
}


int double_ints(double x, int mode)//mode 가 0 이면 x의 정수부를 return, mode가 1이면 x의 소수부를 return
{          
                     int x1,x2;
                     if(x>=0)//x=x1(정수부)+ x2(소수부)
                                {
                                          x1=(int) x;
                                          x2=(int) ((x-x1)*10000);//소숫점 아래 4자리를 표현하기 위함임
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

void encoder_interface(int PToption)
{
	
//	char * keyword_pt;
//	unsigned char temp_char1, temp_char2, temp_char3;
	
//	char * temp_pt;
//	int pixX;
//	unsigned int pixY;

	static int p=0;
	static int t=0;
//	int intbuffer;
	char *cr_pt,*cp;
	bool char_error=0;
	int i;
//	int encoder_string_length;

//	unsigned char XYZ_buffer[100];


	//Pan Encoder
	if(PToption==0){
		while (USART_RXBufferData_Available(&USARTE0_data))
				pan_encoder_string[p++] = (char)USART_RXBuffer_GetByte(&USARTE0_data);
		pan_encoder_string[p]='\0';
		cr_pt=strstr(pan_encoder_string,"\r\n");

		if(cr_pt==NULL)
			return;
		else{
			char_error=0;	
			for(cp=pan_encoder_string;cp<cr_pt;cp++)
				if(*cp!='-' && (*cp<'0'||*cp>'9'))
					char_error=1;
			if(!char_error){
				for(i=0;i<ENCODER_INDEX-1;i++)
					panEncoderBuffer[i+1]=panEncoderBuffer[i];
				panEncoderBuffer[0]=(double)atoi(pan_encoder_string)/MOTOR_ENCODER*2*PI;

			}
		}	
		p=strlen(pan_encoder_string)-(((int)cr_pt-(int)pan_encoder_string)/sizeof(char))-2; //CR, LF
		strcpy(pan_encoder_string,(cr_pt+2));
	}
		
	
	//Tilt Encoder
	else{
	
		while (USART_RXBufferData_Available(&USARTF0_data))
				tilt_encoder_string[t++] = (char)USART_RXBuffer_GetByte(&USARTF0_data);
		tilt_encoder_string[t]='\0';
		cr_pt=strstr(tilt_encoder_string,"\r\n");
		if(cr_pt==NULL)
			return;
		else{	
			char_error=0;
			for(cp=tilt_encoder_string;cp<cr_pt;cp++)
				if(*cp!='-' && (*cp<'0'||*cp>'9'))
					char_error=1;
			if(!char_error){
				for(i=0;i<ENCODER_INDEX-1;i++)
					tiltEncoderBuffer[i+1]=tiltEncoderBuffer[i];
				tiltEncoderBuffer[0]=(double)atoi(tilt_encoder_string)/MOTOR_ENCODER*2*PI;	
			}	
		}
		t=strlen(tilt_encoder_string)-(((int)cr_pt-(int)tilt_encoder_string)/sizeof(char))-2; //CR, LF
		strcpy(tilt_encoder_string,(cr_pt+2));
	}	

//	sprintf((char*)XYZ_buffer,"[%d]\r\n",intbuffer);   
//	uartSendTX(XYZ_buffer);

}

unsigned int pixX,pixY,step;
double size;
unsigned int pixXbuf[VISION_BUF_SIZE]={0,};
unsigned int pixYbuf[VISION_BUF_SIZE]={0,};
unsigned int stepbuf[VISION_BUF_SIZE]={0,};
int indexV=0;

//static double size_sum=VIS_SIZE_BUF*32;
const int resize_step_window_size[]={20,23,25,28,32,36,41,46,51,58,65,73,82,92,104,117,132,148,167,187,211,237,267,300,338};

void Altair_interface(){
		/* Create data packet (SS to slave by PC1). */
	int i;
	for(i=0;i<4;i++){
		SPI_MasterCreateDataPacket(&dataPacket,
		                           &masterSendData_vision[i],
		                           &VISION_DATA[i],
		                           1,
		                           &PORTC,
		                           PIN2_bm);

		/* Transceive packet. */
		SPI_MasterTransceivePacket(&spiMasterC, &dataPacket);
		if(i==0)	_delay_us(10);


		_delay_us(5);
	}
	if(VISION_DATA[0]==0xFF && VISION_DATA[1]==0xFF && VISION_DATA[2]==0xFF && VISION_DATA[3]==0xFF)
		return ;


	indexV=(indexV+1)%VISION_BUF_SIZE;
	stepbuf[indexV]=(unsigned int)(VISION_DATA[1]&0b00011111);
	pixXbuf[indexV]=(unsigned int)((VISION_DATA[1]&0b11000000)<<2)+(unsigned int)VISION_DATA[2];		//left
	pixYbuf[indexV]=(unsigned int)((VISION_DATA[1]&0b00100000)<<3)+(unsigned int)VISION_DATA[3];		//top
	
	if(pixXbuf[indexV]!=0 && 	pixYbuf[indexV]!=0){
		pixXbuf[indexV]+=(unsigned int)((double)resize_step_window_size[stepbuf[indexV]]*0.5);
		pixYbuf[indexV]+=(unsigned int)((double)resize_step_window_size[stepbuf[indexV]]*0.5);

		pixXbuf[indexV]=(unsigned int)(pixXbuf[indexV]*2);	//320*240 -> 640*480
		pixYbuf[indexV]=(unsigned int)(pixYbuf[indexV]*2)-10;	//320*240 -> 640*480
	}
}
	
	
void VisionUpdate()	{
	
//	static double size_sum=VIS_SIZE_BUF*32;
/*	static double size_buf[VIS_SIZE_BUF];
	static int size_index=0;
	if(pixX!=0 && pixY!=0){
		size_buf[size_index]=resize_step_window_size[step];
		size_sum+=size_buf[size_index];
		size_index=(size_index+1)%VIS_SIZE_BUF;
		size_sum-=size_buf[size_index];
		size=size_sum/VIS_SIZE_BUF;
		pixX+=(unsigned int)(size/2);
		pixY+=(unsigned int)(size/2)-5;

		pixX=(unsigned int)(pixX*2);	//320*240 -> 640*480
		pixY=(unsigned int)(pixY*2);	//320*240 -> 640*480
	}*/
	int index=(indexV+1)%VISION_BUF_SIZE;
	pixX=pixXbuf[index];
	pixY=pixYbuf[index];
	step=stepbuf[index];

	if(pixX!=0 && pixY!=0){
		pixX+=(unsigned int)((double)resize_step_window_size[step]*0.5);
		pixY+=(unsigned int)((double)resize_step_window_size[step]*0.5);

		pixX=(unsigned int)(pixX*2);	//320*240 -> 640*480
		pixY=(unsigned int)(pixY*2)-10;	//320*240 -> 640*480
	}
						

	if(pixX !=0 && pixY !=0 )
		{
			m_panPixelError=(int)pixX-320;
			m_tiltPixelError=(int)pixY-240;

			m_panAngleVisionError  = - atan2(m_panPixelError, 320.0/tan(CAM_OA));		
			m_tiltAngleVisionError  = atan2(m_tiltPixelError, 320.0/tan(CAM_OA));

			m_panEncoder=panEncoderBuffer[ENCODER_INDEX-1];
			m_tiltEncoder=tiltEncoderBuffer[ENCODER_INDEX-1];


			m_panAngleVision = m_panAngleVisionError +m_panEncoder;				
			m_tiltAngleVision = m_tiltAngleVisionError +m_tiltEncoder;

			double pixelError=sqrt(m_panPixelError*m_panPixelError+m_tiltPixelError*m_tiltPixelError);
			double visionError=atan2(pixelError, 320.0/tan(CAM_OA));
			double m_camX,m_camY,m_camZ;
			double distance=m_distance[indexD];
			if(pixelError==0){
				m_camX=0;
				m_camY=0;
				m_camZ=distance;
			}
			else{
			 	m_camX=distance*sin(visionError)*m_panPixelError/pixelError;
				m_camY=distance*sin(visionError)*m_tiltPixelError/pixelError;	
			 	m_camZ=distance*cos(visionError);
			}
			//char XYZ_buffer[100];
			//int FFF=((double)sqrt(m_camX*m_camX+m_camY*m_camY+m_camZ*m_camZ)==1.0)? 666:7;
			//double FFF=(double)sqrt(m_camX*m_camX+m_camY*m_camY+m_camZ*m_camZ)*10000.0;
			


			double ca=cos(m_panEncoder), sa=sin(m_panEncoder);
			double cb=cos(m_tiltEncoder), sb=sin(m_tiltEncoder);
			
			m_targetX_robot= sa*m_camX - ca*sb*m_camY + ca*cb*(m_camZ+L_CA) + L_RR;
			m_targetY_robot=-ca*m_camX - sa*sb*m_camY + sa*cb*(m_camZ+L_CA)  ;
			m_targetZ_robot= 			- cb*m_camY	   -    sb*(m_camZ+L_CA) + L_AR;
			
			/*
			double FFF=(double)sqrt(m_targetX_robot*m_targetX_robot+m_targetY_robot*m_targetY_robot+m_targetZ_robot*m_targetZ_robot)*10000.0;

			sprintf((char*)XYZ_buffer,"!! (%5d,%5d,%5d)  %7d !!\n",(int)( m_targetX_robot*10000.0),(int)(m_targetY_robot*10000.0),(int)(m_targetZ_robot*10000.0),(int)FFF);
					uartSendTX((unsigned char*)XYZ_buffer);*/
			m_visionUpdateFlag=1;
	}
}



void host_interface()
{
	unsigned int host_string_length=0;
	char * keyword_pt;
	unsigned char temp_char1, temp_char2, temp_char3;
	
	char * temp_pt;
	unsigned int pixX;
	unsigned int pixY;
	static int initial_ENC=1;

	static int i=0;

//	unsigned char XYZ_buffer[100];

	while (USART_RXBufferData_Available(&USARTD0_data))
	{
			host_string[i] = USART_RXBuffer_GetByte(&USARTD0_data);
			i++;
	}

	host_string[i]='\0';

	keyword_pt=strstr( host_string, "[E");	

	temp_pt=keyword_pt;		
	host_string_length=strlen(keyword_pt);

	if(host_string_length>15 && *(temp_pt+15)==']')
	{
		temp_pt++;
		temp_pt++;
		temp_char1=*temp_pt;
		temp_pt++;
		temp_char2=*temp_pt;
		temp_pt++;
		temp_char3=*temp_pt;
//					enc_left=temp_char1<<16+temp_char2<<8+temp_char3;
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
		temp_pt++;
		temp_pt++;
		temp_char1=*temp_pt;
		temp_pt++;
		temp_char2=*temp_pt;
		pixX= (unsigned int)(temp_char1&0b00001111)<<6;
		pixX+=(unsigned int)(temp_char2&0b00111111);
		temp_pt++;
		temp_pt++;
		temp_char1=*temp_pt;
		temp_pt++;
		temp_char2=*temp_pt;
		pixY= (unsigned int)(temp_char1&0b00001111)<<6;
		pixY+=(unsigned int)(temp_char2&0b00111111);
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


		
		int lt=(int)(m_LeftEncoder-m_prevDLeft);
		m_dLeftInc=(double)lt;
	/*	if(m_dLeftInc>(double)((double)SIXTEENBITS/2))
				m_dLeftInc=m_dLeftInc-(double)SIXTEENBITS;
		else if(m_dLeftInc<(double)(-(double)SIXTEENBITS/2))
				m_dLeftInc=(double)SIXTEENBITS+m_dLeftInc;*/
		m_dLeftInc  = (double) m_dLeftInc/(double) REDUCTION_ENCODER;

		int rt=(int)(m_RightEncoder-m_prevDRight);
		m_dRightInc=rt;
	/*	m_dRightInc=(double)m_RightEncoder -(double)m_prevDRight;
		if(m_dRightInc>(double)((double)SIXTEENBITS/2))
				m_dRightInc=m_dRightInc-(double)SIXTEENBITS;
		else if(m_dRightInc<(double)(-(double)SIXTEENBITS/2))
				m_dRightInc=(double)SIXTEENBITS+m_dRightInc;
	*/	m_dRightInc  = (double) m_dRightInc/(double)REDUCTION_ENCODER;
	if(initial_ENC){
		m_dLeftInc=0;
		m_dRightInc=0;
		initial_ENC=0;
	}
	if(abs(m_dLeftInc)>0.01  || abs(m_dRightInc)>0.01){
		m_dLeftInc=0;
		m_dRightInc=0;
		}

		
	static int indexE=0;

		m_enc_buf[indexE]=-(m_dLeftInc+m_dRightInc)/2.0;
		m_enc_sum+=m_enc_buf[(indexE)%DELAY_COMP_E];
		m_enc_sum-=m_enc_buf[(indexE+1)%DELAY_COMP_E];
		indexE++;
		indexE=indexE%DELAY_COMP_E;

		



/*

		m_dLeftInc=(double)m_LeftEncoder-(double)m_prevDLeft;
		if(m_dLeftInc>(double)((double)SIXTEENBITS/2))
				m_dLeftInc=m_dLeftInc-(double)SIXTEENBITS;
		else if(m_dLeftInc<(double)(-(double)SIXTEENBITS/2))
				m_dLeftInc=(double)SIXTEENBITS+m_dLeftInc;
		m_dLeftInc  = (double) m_dLeftInc/(double) REDUCTION_ENCODER;
*/	
/*

		m_dRightInc=(double)m_RightEncoder -(double)m_prevDRight;
		if(m_dRightInc>(double)((double)SIXTEENBITS/2))
				m_dRightInc=m_dRightInc-(double)SIXTEENBITS;
		else if(m_dRightInc<(double)(-(double)SIXTEENBITS/2))
				m_dRightInc=(double)SIXTEENBITS+m_dRightInc;
		m_dRightInc  = (double) m_dRightInc/(double)REDUCTION_ENCODER;
*/
		m_prevDLeft = m_LeftEncoder;
		m_prevDRight = m_RightEncoder;

//		if(pixX*pixX+pixY*pixY>1000)
		

	}
}

int Multiply4x4Mtx(double A[4][4], double B[4][4],double X[4][4])
{
	int i,j,k;
	double buffer[4][4];	
	for(i=0;i<4;i++){		
		for(j=0;j<4;j++){
			buffer[i][j]=0;
			for(k=0;k<4;k++)
				buffer[i][j]+= A[i][k]*B[k][j];			
		}
	}
	for(i=0;i<4;i++)
		for(j=0;j<4;j++)
			X[i][j]=buffer[i][j];
	return 0;
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
	
void position_estimator(unsigned int XXL,unsigned int YXL,unsigned int ZXL,unsigned int XGY,unsigned int YGY,unsigned int ZGY)
{

//	unsigned char XYZ_buffer[100];
//	double m_prevCounter;
//	double m_counter;

	static double m_XGY_sum=0;
	static double m_YGY_sum=0;
	static double m_ZGY_sum=0;
	static int indexG=0;
	int i,j;	


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



		m_XXL=SCALE_XXL*((double)XXL-(double)m_biasXXL);
		m_YXL=SCALE_YXL*((double)YXL-(double)m_biasYXL);
		m_ZXL=SCALE_ZXL*((double)ZXL-(double)m_biasZXL);
		m_XGY=-SCALE_XGY*((double)XGY-(double)m_biasXGY);//-0.001212142/0.00015711
		m_YGY=SCALE_YGY*((double)YGY-(double)m_biasYGY);//+
		m_ZGY=SCALE_ZGY*((double)ZGY-(double)m_biasZGY);//+
		/*m_XGY=0;
		m_YGY=0;
		m_ZGY=0;*/

		

		m_XGY_buf[indexG]=m_XGY;
		m_XGY_sum+=m_XGY_buf[(indexG)%DELAY_COMP_G];
		m_XGY_sum-=m_XGY_buf[(indexG+1)%DELAY_COMP_G];
		m_YGY_buf[indexG]=m_YGY;
		m_YGY_sum+=m_YGY_buf[(indexG)%DELAY_COMP_G];
		m_YGY_sum-=m_YGY_buf[(indexG+1)%DELAY_COMP_G];
		m_ZGY_buf[indexG]=m_ZGY;
		m_ZGY_sum+=m_ZGY_buf[(indexG)%DELAY_COMP_G];
		m_ZGY_sum-=m_ZGY_buf[(indexG+1)%DELAY_COMP_G];

		indexG=(indexG+1)%DELAY_COMP_G;

		ve_gain=0;
		
		robotState=RobotStateEstimation();
		

		double c1, c2, c3, s1, s2, s3;
		double R[4][4];
		double tmpVect[4]={0,};
		double targetVect[4];
		if (m_visionUpdateFlag==11){
		}
		else if(m_visionUpdateFlag==1){
				m_XGY_sum+=m_XGY*FF_GAIN;
				m_YGY_sum+=m_YGY*FF_GAIN;
				m_ZGY_sum+=m_ZGY*FF_GAIN;
			//	ve_gain=VE_GAIN;
				ve_gain=1;

				c1=cos(m_ZGY_sum*m_samplingTime); c2=cos(m_XGY_sum*m_samplingTime); c3=cos(m_YGY_sum*m_samplingTime);
				s1=sin(m_ZGY_sum*m_samplingTime); s2=sin(m_XGY_sum*m_samplingTime); s3=sin(m_YGY_sum*m_samplingTime);

				R[0][0]=c1*c3+s1*s2*s3;		R[0][1]=s1*c2;		R[0][2]=-c1*s3+s1*s2*c3;		R[0][3]=0;
				R[1][0]=c1*s2*s3-s1*c3;		R[1][1]=c1*c2;		R[1][2]=s1*s3+c1*s2*c3;			R[1][3]=0;
				R[2][0]=c2*s3;				R[2][1]=-s2;		R[2][2]=c2*c3;					R[2][3]=0;
				R[3][0]=0;					R[3][1]=0;			R[3][2]=0;						R[3][3]=1;
		

				//Robot coordinat update
				targetVect[0]=m_targetX_robot; targetVect[1]=m_targetY_robot;	targetVect[2]=m_targetZ_robot;	targetVect[3]=1;
				for(i=0;i<4;i++){
					tmpVect[i]=0;
					for(j=0;j<4;j++)
						tmpVect[i]+=R[i][j]*targetVect[j];
				}
				m_targetX_robot=tmpVect[0];
				m_targetY_robot=tmpVect[1];
				m_targetZ_robot=tmpVect[2];
				m_XGY_sum-=m_XGY*FF_GAIN;
				m_YGY_sum-=m_YGY*FF_GAIN;
				m_ZGY_sum-=m_ZGY*FF_GAIN;

		}
		else{
		
			//Robot rotation matrix update
			c1=cos(m_ZGY*m_samplingTime); c2=cos(m_XGY*m_samplingTime); c3=cos(m_YGY*m_samplingTime);
			s1=sin(m_ZGY*m_samplingTime); s2=sin(m_XGY*m_samplingTime); s3=sin(m_YGY*m_samplingTime);

			R[0][0]=c1*c3+s1*s2*s3;		R[0][1]=s1*c2;		R[0][2]=-c1*s3+s1*s2*c3;	R[0][3]=-(m_dLeftInc+m_dRightInc)/2.0;
			R[1][0]=c1*s2*s3-s1*c3;		R[1][1]=c1*c2;		R[1][2]=s1*s3+c1*s2*c3;		R[1][3]=0;
			R[2][0]=c2*s3;				R[2][1]=-s2;		R[2][2]=c2*c3;				R[2][3]=0;
			R[3][0]=0;					R[3][1]=0;			R[3][2]=0;					R[3][3]=1;


			//Gravity compensation (for accelerometer, using gyroscope)
			for(i=0;i<3;i++){
				tmpVect[i]=0;
				for(j=0;j<3;j++)
					tmpVect[i]+=R[i][j]*gravityVect[j];
			}
			for(i=0;i<3;i++)
				gravityVect[i]=tmpVect[i];	
			m_XXL0g	= m_XXL + gravityVect[0];
			m_YXL0g	= m_YXL + gravityVect[1];
			m_ZXL0g	= m_ZXL - gravityVect[2];
			
			//Rotation compensation
			m_XXL0g	-=  m_ZGY*m_ZGY/7.0*4.0/3*0;


			//Robot coordinat update
			targetVect[0]=m_targetX_robot; targetVect[1]=m_targetY_robot;	targetVect[2]=m_targetZ_robot;	targetVect[3]=1;
				
			for(i=0;i<4;i++){
				tmpVect[i]=0;
				for(j=0;j<4;j++)
					tmpVect[i]+=R[i][j]*targetVect[j];
			}
			m_targetX_robot=tmpVect[0];
			m_targetY_robot=tmpVect[1];
			m_targetZ_robot=tmpVect[2];

		}
		double m_targetX_act,m_targetY_act,m_targetZ_act;
		
		m_targetX_act=m_targetX_robot-L_RR;
		m_targetY_act=m_targetY_robot;				
		m_targetZ_act=m_targetZ_robot-L_AR;			//로봇 좌표계에서 본 액츄에이터 원점과 타겟 사이의 상대 위치
		
	
		static double acc_ff_buf[ACC_FF_LPF];
		static int indexA=0;
		static double acc_ff_sum=0;
		static double acc_ff_angle=0;
		if(robotState)	acc_ff_angle=acc_ff_angle/2;
		
		acc_ff_buf[indexA]=m_XXL0g;
		acc_ff_sum+=acc_ff_buf[indexA];
		indexA=(indexA+1)%ACC_FF_LPF;
		acc_ff_sum-=acc_ff_buf[indexA];

		//double acc_ff=0;
		if(m_visionUpdateFlag==1){
			//acc_ff=-acc_ff_sum/ACC_FF_LPF*ACC_FF_GAIN*100.0;
			//m_targetX_act+=acc_ff;	//Vision-Acclerometer delay compensation
			//m_targetX_act+=m_enc_sum*ve_gain;	//Vision-Encoder delay compensation
				
			
				//char XYZ_buffer[100];
				//sprintf((char*)XYZ_buffer,"%d,%d \r",(int)robotState,(int)(acc_ff*1000.0));
				//uartSendTX((unsigned char*)XYZ_buffer);

		}
		double r_sq=m_targetX_act*m_targetX_act+m_targetY_act*m_targetY_act;
		//acc_ff_angle+=m_XXL0g*ACC_FF_GAIN*m_targetY_act/r_sq*1.5*2.465/3*0;
		acc_ff_angle+=m_XXL0g;

		double acc_ff_gain=ACC_FF_GAIN*m_targetY_act/r_sq*1.5*2.465/3/3*2;
		if(m_targetX_act !=0)
			m_sensorPanAngle=atan2(m_targetY_act, m_targetX_act)+acc_ff_angle*acc_ff_gain;//-PI/2.0; // CCW가 +		
		else{
			if(m_targetY_act !=0)
				m_sensorPanAngle=PI/2*(abs(m_targetY_act)/m_targetY_act)+acc_ff_angle*acc_ff_gain;
		}		
						
		if(sqrt(m_targetX_act*m_targetX_act+m_targetY_act*m_targetY_act) !=0)	
			m_sensorTiltAngle=atan2(-m_targetZ_act,sqrt(m_targetX_act*m_targetX_act+m_targetY_act*m_targetY_act));
		
		if(m_visionUpdateFlag==1 ){
			m_visionUpdateFlag=0;
		//	m_targetX_act-=acc_ff;	//Vision-Acclerometer delay compensation
			m_targetX_act+=m_enc_sum*ve_gain;		//Vision-Encoder delay compensation
				char XYZ_buffer[100];
				sprintf((char*)XYZ_buffer,"%d %d %d %d\r",(int)m_panPixelError,(int)(acc_ff_angle*180.0/3.14),(int)(acc_ff_angle*acc_ff_gain*180.0/3.14*1000.0),(int)(robotState));
//	sprintf((char*)XYZ_buffer,"%d %d \r",(int)( m_enc_sum/DELAY_COMP_E/0.02*1000.0 ),(int)(vel_KF_update()*1000.0));
				uartSendTX((unsigned char*)XYZ_buffer);
	


		}
		
		m_panAngle  = m_sensorPanAngle; 
		m_tiltAngle = m_sensorTiltAngle;
		//if(m_dLeftInc+m_dRightInc !=0)
		m_distance[indexD]=sqrt(m_targetX_act*m_targetX_act+m_targetY_act*m_targetY_act+m_targetZ_act*m_targetZ_act);
		indexD=(indexD+1)%DELAY_COMP_D;
		taeilBuf[0]=m_targetX_act*100.0;
		taeilBuf[1]=m_targetY_act*100.0;
		m_dLeftInc=0;
		m_dRightInc=0;
		if(m_visionUpdateFlag==1)		m_visionUpdateFlag++;

//		unsigned char XYZ_buffer[100];
		//sprintf((char*)XYZ_buffer,"[P%2d.%5d][T%2d.%5d]%6d%6d\n\r",double_ints(m_panAngle,0),double_ints(m_panAngle,1),double_ints(m_tiltAngle,0),double_ints(m_tiltAngle,1),m_panPixelError,m_tiltPixelError);   
//		sprintf((char*)XYZ_buffer,"[P%2d.%5d][T%2d.%5d][P%2d.%5d]\n\r",double_ints(m_panAngle,0),double_ints(m_panAngle,1),double_ints(m_tiltAngle,0),double_ints(m_tiltAngle,1),double_ints(m_XGY,0),double_ints(m_XGY,1));   


//		sprintf((char*)XYZ_buffer,"EL%6d,ER%6d,VX%6d,VY%6d\n\r",m_LeftEncoder,m_RightEncoder,pixX,pixY);
//				uartSendTX(host_string);
		//uartSendTX(XYZ_buffer);


	return;
}

void PosCtrlMotor(int pantilt, double angle)
{
	int encoderCount;
	char temp[2][20];

	if(pantilt==0){ //pan actuation, tilt encoder
		
		if(angle<135.0/180*PI && angle>-135.0/180*PI){
			encoderCount=(int)(angle*(MOTOR_ENCODER/2/PI));
			sprintf((char*)temp[0],"LA%d\nM\n",encoderCount);
			uartE0SendTX((unsigned char*)temp[0]);
		}
			sprintf((char*)temp[1],"POS\n");
			uartF0SendTX((unsigned char*)temp[1]);		
	}
	else{ //tilt actuation, pan encoder
		if(angle<30.0/180*PI && angle>-90.0/180*PI){
			encoderCount=(int)(angle*(MOTOR_ENCODER/2/PI));
			sprintf((char*)temp[0],"LA%d\nM\n",encoderCount);
			uartF0SendTX((unsigned char*)temp[0]);
		}
		sprintf((char*)temp[1],"POS\n");
		uartE0SendTX((unsigned char*)temp[1]);
	}
		
}

unsigned int tempp[3];

#define XL_BUF		10
#define XL_STOP_TH	0.002

int RobotStateEstimation(){
	int i;
	static double  XLsqBuf[3][XL_BUF]={{0,},};
	static double  XLBuf[3][XL_BUF]={{0,},};
	static double XLsqSum[3]={0,0,0};
	static double XLSum[3]={0,0,0};
	static int indexX=0;
	double XLdata[3]={m_XXL, m_YXL, m_ZXL};
	double XLvar[3];
	for(i=0;i<1;i++){
		XLsqBuf[i][indexX]= XLdata[i]* XLdata[i];
		XLBuf[i][indexX]= XLdata[i];
		XLsqSum[i]+= XLsqBuf[i][indexX];
		XLSum[i]+= XLBuf[i][indexX];
		indexX=(indexX+1)%XL_BUF;
		XLsqSum[i]-= XLsqBuf[i][indexX];
		XLSum[i]-= XLBuf[i][indexX];
		XLvar[i]=(XLsqSum[i]-XLSum[i]*XLSum[i]/XL_BUF)/XL_BUF;
		if(XLvar[i]>XL_STOP_TH)	
			return 0;	// Robot is moving		
	}
	return 1;	// Robot is stopped
}


double vel_KF_update(){
	//Measurement
	double Xm[2];
	static double Xp[2];
	double Z[2]={ m_enc_sum/DELAY_COMP_E/0.015  , m_XXL0g };
	double Phi[2][2]= {{1				,m_samplingTime},
					 { 0				,1}};
	double PhiT[2][2]={{1				, 0},
					  {m_samplingTime	, 1}};
	double Q[2][2]={{1		, 0		},
					{0     	, 1		}};
	double R[2][2]={{10		, 0		},
					{0     	, 10		}};
	double K[2][2];
	double Pm[2][2];
	static double Pp[2][2];
	//H = identity

	//State Esimae Extrapolaion
	mtx_mul_2x1(Xm,Phi,Xp);

	//Error Covariance Extrapolaion
	mtx_mul_2x2(Pm,Phi,Pp);
	mtx_mul_2x2(Pm,Pm,PhiT);
	mtx_add_2x2(Pm,Pm,Q,1);

	//Kalman Gain Matrix
	mtx_add_2x2(K,Pm,R,1);
	mtx_inv_2x2(K,K);
	mtx_mul_2x2(K,Pm,K);

	//State Estimae Update
	mtx_add_2x1(Xp,Z,Xm,-1);
	mtx_mul_2x1(Xp,K,Xp);
	mtx_add_2x1(Xp,Xm,Xp,1);
	
	//Error Covarinance Update
	mtx_mul_2x2(Pp,K,Pm);
	mtx_add_2x2(Pp,Pm,Pp,-1);
	
	return 	Xp[0];	//velocity return
}








int main(void)
{
	unsigned int XXL,YXL,ZXL,XGY,YGY,ZGY;
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


	/* Use USARTD0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USARTD0_data, &USARTD0, USART_DREINTLVL_HI_gc);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USARTD0_data.usart, USART_RXCINTLVL_HI_gc);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_HILVLEX_bm;

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

//sprintf((char*)host_string,"[E123,456,xx,yy]");


/* Motor Driver Initialization*/
#if 0
	char buf[100];	
	_delay_ms(1000);

	/* Baud rate setting
	BSEL=2094, BSCALE=-7, CLK2X  => 57600 bps
	BSEL=3317, BSCALE=-4, CLK2X  =>  9600 bps
	*/

/*	USART_Baudrate_Set(&USARTE0, 3317 ,-4);
	USARTE0.CTRLB|=0x04;                  
	USART_Baudrate_Set(&USARTF0, 3317 ,-4);
	USARTF0.CTRLB|=0x04;                  */
	sprintf((char*)buf,"ANSW0\nRM115000\nKN304\nENCRES1792\n0BAUD57600\n");
	uartE0SendTX((unsigned char*)buf);
	uartF0SendTX((unsigned char*)buf);
/*	USART_Baudrate_Set(&USARTE0, 2094 ,-7);
	USARTE0.CTRLB|=0x04;                  
	USART_Baudrate_Set(&USARTF0, 2094 ,-7);
	USARTF0.CTRLB|=0x04;           */       
	sprintf((char*)buf,"EEPSAV\n");
	uartE0SendTX((unsigned char*)buf);
	uartF0SendTX((unsigned char*)buf);	

#endif


//Homing sequence
#if 1
	/* Tilt homing sequence*/
		
	char temp[100];
	_delay_ms(2000);
	//Clear RX buffer
	while (USART_RXBufferData_Available(&USARTF0_data))
			USART_RXBuffer_GetByte(&USARTF0_data);
	sprintf((char*)temp,"ANSW1\nEN\nGOHOSEQ\n");
	uartF0SendTX((unsigned char*)temp);
	_delay_ms(3000);
	while(USART_RXBuffer_GetByte(&USARTF0_data)=='x');
	sprintf((char*)temp,"LR-650\nM\n");
	//sprintf((char*)temp,"LR-570\nM\n");
	uartF0SendTX((unsigned char*)temp);
	
	
	/* Pan homing sequence*/
	//Clear RX buffer
	_delay_ms(1000);
	while (USART_RXBufferData_Available(&USARTE0_data))
			USART_RXBuffer_GetByte(&USARTE0_data);
	
	sprintf((char*)temp,"ANSW1\nEN\nGOHOSEQ\n");
	uartE0SendTX((unsigned char*)temp);
	_delay_ms(4000);
	while(USART_RXBuffer_GetByte(&USARTE0_data)=='x');
	sprintf((char*)temp,"LR-3210\nM\n");
	//sprintf((char*)temp,"LR-2620\nM\n");
	uartE0SendTX((unsigned char*)temp);
#endif	
	unsigned int oldT=0;
	unsigned int newT=0;	
	int i;
	for(i=0;i<DELAY_COMP_D;i++)
		m_distance[i]=sqrt((TARGET_X-L_AR)*(TARGET_X-L_AR)+TARGET_Y*TARGET_Y+(TARGET_Z-L_RR)*(TARGET_Z-L_RR));
	while(1) 
	{
		
		if(samplingFlag)
	    {
			
		
					newT=TCC0.CNT;
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
			_delay_us(5);
			/* Transceive packet. */
			SPI_MasterTransceivePacket(&spiMasterC, &dataPacket);
			/* MASTER: Release SS to slave. */
			_delay_us(5);
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
			_delay_us(5);
			SPI_MasterTransceivePacket(&spiMasterC, &dataPacket);
			/* MASTER: Release SS to slave. */
			_delay_us(5);
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
			XGY= (unsigned int)GYRO_DATA[4]*256+GYRO_DATA[3];
			YGY= (unsigned int)GYRO_DATA[2]*256+GYRO_DATA[1];
			ZGY= (unsigned int)GYRO_DATA[6]*256+GYRO_DATA[5];


			timer_old=timer;

			m_samplingTime=(double)diff_counter/2000000.0;
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
				m_biasXXL=(double)m_XXL_sum/(double)Bsample;
				m_biasYXL=(double)m_YXL_sum/(double)Bsample;
				m_biasZXL=(double)m_ZXL_sum/(double)Bsample-GRAVITY_COUNT;
				m_biasXGY=(double)m_XGY_sum/(double)Bsample;
				m_biasYGY=(double)m_YGY_sum/(double)Bsample;
				m_biasZGY=(double)m_ZGY_sum/(double)Bsample;
					
				
				gravityVect[0]=0;
				gravityVect[1]=0;
				gravityVect[2]=SCALE_ZXL*GRAVITY_COUNT;





				m_IMU_count++;


				USART_Rx_Enable(&USARTD0);
				USART_Tx_Enable(&USARTD0);
				char temp[100];
				sprintf((char*)temp,"ANSW0\nHO\nEN\n");
				uartE0SendTX((unsigned char*)temp);
				uartF0SendTX((unsigned char*)temp);
				
				while (USART_RXBufferData_Available(&USARTE0_data))
					USART_RXBuffer_GetByte(&USARTE0_data);
				while (USART_RXBufferData_Available(&USARTF0_data))
					USART_RXBuffer_GetByte(&USARTF0_data);
		    }
		    else
			 { 

				encoder_interface(0);
				encoder_interface(1);
				host_interface();
				if(loop_counter%4==1)
					Altair_interface();
				if(loop_counter%VISION_LOOP_CNT==1)
					VisionUpdate();

				//position_estimator(XXL,YXL,ZXL,XGY,YGY,ZGY);
				position_estimator(XXL,YXL,ZXL,XGY,YGY,ZGY);
				if(loop_counter%2==0)
				{
					PosCtrlMotor(0, m_panAngle);
//					char XYZ_buffer[100];			
//					sprintf((char*)XYZ_buffer,"%u\n\r",(unsigned int) (newT-oldT));
				}
				else{
					PosCtrlMotor(1, m_tiltAngle);
				}
					
			//	char XYZ_buffer[100];
				//sprintf((char*)XYZ_buffer,"%d \r",(int)(m_samplingTime*10000.0));
				//sprintf((char*)XYZ_buffer,"%d %d \r",(int)(m_panAngle*(MOTOR_ENCODER/2/PI)), (int)(panEncoderBuffer[0]*(MOTOR_ENCODER/2/PI)));
			//	sprintf((char*)XYZ_buffer,"%d %d \r",(int)taeilBuf[0], (int)taeilBuf[1]);
			//	sprintf((char*)XYZ_buffer,"%u \r",ZGY);
				//sprintf((char*)XYZ_buffer,"%d %d %d\r",
			//	sprintf((char*)XYZ_buffer,"%4d %4d %4d, %4d %4d %4d\r",
	//sprintf((char*)XYZ_buffer,"%4d %4d %4d \r",
			//	(int)(1000.0*(m_XXL)), (int)(1000.0*(m_YXL)),(int)(1000.0*(m_ZXL)),
				
				//(int)(1000.0*(-gravityVect[0])), (int)(1000.0*(-gravityVect[1])),(int)(1000.0*(gravityVect[2])));			//	(int)(1000.0*(m_XXL+gravityVect[0])), (int)(1000.0*(m_YXL+gravityVect[1])),(int)(1000.0*(m_ZXL-gravityVect[2])));
			
		//	sprintf((char*)XYZ_buffer,"%u %u %u \r",	XXL,YXL,ZXL);
		//		sprintf((char*)XYZ_buffer,"%u %u %u \r",	XGY,YGY,ZGY);
			//	sprintf((char*)XYZ_buffer,"%d %d \r",(int)(-0.5*m_XXL0g*m_samplingTime*100000.0),(int)(-0.5*m_XXL*m_samplingTime*100000.0));
					//(int)(-0.5*m_YXL0g*m_samplingTime*100000.0),(int)(-0.5*m_YXL*m_samplingTime*100000.0),
//					(int)(-0.5*m_ZXL0g*m_samplingTime*100000.0),(int)(-0.5*m_ZXL*m_samplingTime*100000.0));
//
//				uartSendTX((unsigned char*)XYZ_buffer);

					//(int)(taeilBuf* 10000.0)
					//(long)(m_panAngle*180.0/PI*100),(long)(m_tiltAngle*180.0/PI*100));
				/*	sprintf((char*)XYZ_buffer,"[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]\n\r",
					(long)(newT-oldT),(long)(m_RightEncoder),
					(long)(m_dLeftInc*10000000.0),(long)(m_dRightInc*10000000.0),(long)(m_distance*10000.0),

					//(long)XGY,(long)YGY,(long)ZGY,

					(long)(m_targetX_robot*10000.0),(long)(m_targetY_robot*10000.0),(long)(m_targetZ_robot*10000.0)
					//(long)(m_panAngle*180.0/PI*100),(long)(m_tiltAngle*180.0/PI*100));
					);
*/


					//double_ints(atan2(-1,700)*180.0/PI,0),double_ints(atan2(-1,700)*180.0/PI,1),
				/*	double_ints(m_camX,0),double_ints(m_camX,1),
					double_ints(m_camY,0),double_ints(m_camY,1),
					double_ints(m_camZ,0),double_ints(m_camZ,1));*/
					/*double_ints(m_targetX_robot,0),double_ints(m_targetX_robot,1),
					double_ints(m_targetY_robot,0),double_ints(m_targetY_robot,1),
					double_ints(m_targetZ_robot,0),double_ints(m_targetZ_robot,1),
					double_ints(m_panAngle*180.0/PI,0),double_ints(m_panAngle*180.0/PI,1),
					double_ints(m_tiltAngle*180.0/PI,0),double_ints(m_tiltAngle*180.0/PI,1));*/
					//sprintf((char*)XYZ_buffer,"[%d][%u][%3d][%d]\n\r",atoi(pan_encoder_string),m_RightEncoder,0,m_samplingTime );   
					//sprintf((char*)XYZ_buffer,"[%d.%d]\n\r",double_ints((double)m_samplingTime,0),double_ints((double)m_samplingTime,1) );   
				//	uartSendTX((unsigned char*)XYZ_buffer);

				
				
				oldT=TCC0.CNT;
				loop_counter++;
				if(loop_counter==32)
					loop_counter=0;
				
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
