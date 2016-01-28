/**************************************************************************************************
  Filename:     hal_uart.c
  Revised:      $Date: 2013-02-06 09:21:21 -0800 (Wed, 06 Feb 2013) $
  Revision:     $Revision: 33001 $

  Description:  This file contains the interface to the H/W UART driver.


  Copyright 2006-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 밃S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "hal_board_cfg.h"
#include "hal_defs.h"
#include "hal_drivers.h"
#include "hal_types.h"
#include "Changed_hal_uart.h"
#include "OSAL.h"

// GPS UART initialize value
#define UART_BAUD_M								59	// set buadrate to 9600bps
#define UART_BAUD_E								8	// set baudrate to 9600bps

#define PERCFG_U1CFG_ALT1							0x00
#define PERCFG_U1CFG								0x01

#define UART_BAUD_M_0								216
#define UART_BAUD_E_0								11		// set baudrate to 115200bps

#define PERCFG_U0CFG_ALT1							0x00
#define PERCFG_U0CFG								0x01

/******************************************************************************
 * @fn      HalUARTInit
 *
 * @brief   Initialize the UART
 *
 * @param   none
 *
 * @return  none
 *****************************************************************************/
void HalUARTInit(void)
{
        // UART0 init - MODEM
	P1_7 = 0;
	
	P0_6 = 0;
	P0_7 = 1;
	
	PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U0CFG_ALT1;
	P0SEL |= 0x0C;
	P0DIR |= 0x0C;
	PERCFG = 0x00;
	
	/* UART0 peripheral initialize */
	U0CSR = 0xC0;
	U0UCR = 0x02;
	U0GCR = 0x0B;
	U0BAUD = 216;
	URX0IE = 1;
	
	// UART1 init - GPS
	P2_0 = 0;
	P1_0 = 0;
		
	PERCFG = (PERCFG & ~PERCFG_U1CFG) | PERCFG_U1CFG_ALT1;
	P0SEL |= 0x3C;
	U1CSR = 0xC0;
	U1UCR = 0x02;
	U1GCR = 0x08;
	U1BAUD = 59;
	
	URX1IE = 1;
}

void uart0_puts(const char* str)
{
	while(*str)
	{
		U0DBUF = *str++;
		WAIT_TX0RDY;
	}
}

void uart1_puts(const char* str)
{
	while(*str)
	{
		U1DBUF = *str++;
		WAIT_TX1RDY;
	}
}

/******************************************************************************
******************************************************************************/
