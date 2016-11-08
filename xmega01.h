/*
 * xmega01.h
 *
 *  Created on: 20 de oct. de 2016
 *      Author: pablo
 */

#ifndef SRC_XMEGA01_H_
#define SRC_XMEGA01_H_

#include <avr/io.h>

#include <stdlib.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "avr_compiler.h"
#include "clksys_driver.h"

#include "TC_driver.h"
#include "pmic_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "FRTOS-IO.h"
#include "cmdline.h"
#include "sp5Klibs/rtc_sp5K.h"

#define SP6K_REV "1.0.3"
#define SP6K_DATE "@ 20161103"

#define SP6K_MODELO "sp6KV1 HW:xmega256A3 R1.0"
#define SP6K_VERSION "FW:FRTOS8"

void SET_systemMainClock(void);
void testPort(void);
void pvEnable2MHZclock(void);
void pvEnable32MHZclock(void);
void configureTimer(void);

#define tk1_STACK_SIZE		512
#define tk1_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
void tk1(void * pvParameters);
void tk2(void * pvParameters);
void tk3(void * pvParameters);
void tkCmd(void * pvParameters);
void initMCU(void);

s08 pvConfigBTbaud(char *s_baud);

#define WDT_IsSyncBusy() ( WDT.STATUS & WDT_SYNCBUSY_bm )
void WDT_EnableAndSetTimeout( WDT_PER_t period );
void WDT_DisableWindowMode( void );
#define WDT_Reset()	asm("wdr") //( watchdog_reset( ) )

s08 pvSetGprsPwr( u08 value );
s08 pvSetGprsSw( u08 value );
s08 pvSetGprsRts( u08 value );
s08 pvSetBtPwr( u08 value );
void pv_cmdRdRTC(void);
s08 u_wrRtc(char *s);

#define CHAR128	128
char d_printfBuff[CHAR128];
TaskHandle_t xHandle_tk1,xHandle_tk2,xHandle_tk3, xHandle_tkCmd;

#define CHAR256	256

#define USB_CTS_PIN		0
#define USB_CTS_PORT	PORTC
#define USB_RTS_PIN		1
#define USB_RTS_PORT	PORTC

#define BT_CTS_PIN		1
#define BT_CTS_PORT		PORTD
#define BT_RTS_PIN		4
#define BT_RTS_PORT		PORTD
#define BT_PWR_PIN		5
#define BT_PWR_PORT		PORTD

#define GPRS_CTS_PIN	1
#define GPRS_CTS_PORT	PORTF
#define GPRS_RTS_PIN	4
#define GPRS_RTS_PORT	PORTE
#define GPRS_HWPWR_PIN	7
#define GPRS_HWPWR_PORT	PORTD
#define GPRS_SWPWR_PIN	6
#define GPRS_SWPWR_PORT	PORTD

#define XBEE_CTS_PIN		7
#define XBEE_CTS_PORT	PORTF
#define XBEE_RTS_PIN		0
#define XBEE_RTS_PORT	PORTA
#define XBEE_RESET_PIN		4
#define XBEE_RESET_PORT	PORTF
#define XBEE_SLEEP_PIN		6
#define XBEE_SLEEP_PORT	PORTF

#define DEBUG_I2C	1

#endif /* SRC_XMEGA01_H_ */
