/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include "xmega01.h"

static char cmd_printfBuff[256];
char *argv[16];

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);
static u08 pv_makeArgv(void);

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdClearScreen(void);
static void cmdHelpFunction(void);
static void cmdResetFunction(void);
static void cmdWriteFunction(void);
static void cmdReadFunction(void);

/*------------------------------------------------------------------------------------*/
void tkCmd(void * pvParameters)
{

u08 c;
u08 ticks;
( void ) pvParameters;

	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	cmdlineInit();
	cmdlineSetOutputFunc(pvFreeRTOS_CMD_writeChar);

	cmdlineAddCommand((u08 *)("cls"), cmdClearScreen );
	cmdlineAddCommand((u08 *)("help"), cmdHelpFunction);
	cmdlineAddCommand((u08 *)("reset"), cmdResetFunction);
	cmdlineAddCommand((u08 *)("write"), cmdWriteFunction);
	cmdlineAddCommand((u08 *)("read"), cmdReadFunction);

	// Espero la notificacion para arrancar
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("starting tkCmd..\r\n\0"));
	FreeRTOS_CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Fijo el timeout del READ
	ticks = 5;
	FreeRTOS_ioctl( &pdUART_USB,ioctlSET_TIMEOUT, &ticks );
	FreeRTOS_ioctl( &pdUART_BT,ioctlSET_TIMEOUT, &ticks );

	// Prendo el BT
	pvSetBtPwr(1);

	// loop
	for( ;; )
	{

			c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
			// el read se bloquea 50ms. lo que genera la espera.
			while ( FreeRTOS_CMD_read( &c, 1 ) == 1 ) {
				cmdlineInputFunc(c);
			}

			/* run the cmdline execution functions */
			cmdlineMainLoop();
			//vTaskDelay( ( TickType_t)( 1 ));

	}

}
/*------------------------------------------------------------------------------------*/
static void cmdResetFunction(void)
{
	// Resetea al micro prendiendo el watchdog

	CCP = 0xD8;
	WDT.CTRL = 0x02;
	while(WDT_IsSyncBusy());

	CCP = 0xD8;
	WDT.WINCTRL = 0x01;
	while(WDT_IsSyncBusy());

//	portENTER_CRITICAL();
//	WDT_DisableWindowMode();
//	WDT_EnableAndSetTimeout( WDT_PER_32CLK_gc );
//	portEXIT_CRITICAL();

	//WDT_Reset();

	while(1)
		;

}
/*------------------------------------------------------------------------------------*/
static void cmdWriteFunction(void)
{
s08 retS = FALSE;
u08 argc;

	argc = pv_makeArgv();

	// BTBAUD (9600 | 115200)
	if ( !strcmp_P( strupr(argv[1]), PSTR("BTBAUD\0")))  {
		retS = pvConfigBTbaud( argv[2] );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSPWR\0") ) ) {
		retS = pvSetGprsPwr( (u08) atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsSW
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSSW\0") ) ) {
		retS = pvSetGprsSw( (u08) atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsRTS
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSRTS\0") ) ) {
		retS = pvSetGprsRts( (u08) atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// btPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("BTPWR\0") ) ) {
		retS = pvSetBtPwr( (u08) atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// GPRS
	// Envia un comando al modem.
	// AT+CGDCONT=1,"IP","SPYMOVIL.VPNANTEL"
	// AT*E2IPA=1,1
	// AT*E2IPI=0
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0") ) ) {
		snprintf( cmd_printfBuff,sizeof(cmd_printfBuff),"%s\r",argv[2] );
		FreeRTOS_ioctl( &pdUART_GPRS,ioctl_UART_CLEAR_RX_BUFFER, NULL);
		FreeRTOS_ioctl( &pdUART_GPRS,ioctl_UART_CLEAR_TX_BUFFER, NULL);
		FreeRTOS_write( &pdUART_GPRS, cmd_printfBuff, sizeof(cmd_printfBuff) );

		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("tx->[%s]\r\n\0"),argv[2] );
		FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// XBEE
	if (!strcmp_P( strupr(argv[1]), PSTR("XBEE\0") ) ) {
		snprintf( cmd_printfBuff,sizeof(cmd_printfBuff),"%s\r",argv[2] );
		FreeRTOS_ioctl( &pdUART_XBEE,ioctl_UART_CLEAR_RX_BUFFER, NULL);
		FreeRTOS_ioctl( &pdUART_XBEE,ioctl_UART_CLEAR_TX_BUFFER, NULL);
		FreeRTOS_write( &pdUART_XBEE, cmd_printfBuff, sizeof(cmd_printfBuff) );

		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("tx->[%s]\r\n\0"),argv[2] );
		FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		retS = u_wrRtc(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
static void cmdReadFunction(void)
{
char *p;
u08 argc;

	argc = pv_makeArgv();

	// GPRS RSP.
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0"))) {
		p = FreeRTOS_UART_getFifoPtr(&pdUART_GPRS);
		FreeRTOS_CMD_write( "rx->[", sizeof("rx->[")  );
		FreeRTOS_CMD_write( p, UART_GPRS_RXBUFFER_LEN );
		FreeRTOS_CMD_write( "]\r\n", sizeof("]\r\n")  );
		return;
	}

	// XBEE RSP.
	if (!strcmp_P( strupr(argv[1]), PSTR("XBEE\0"))) {
		p = FreeRTOS_UART_getFifoPtr(&pdUART_XBEE);
		FreeRTOS_CMD_write( "rx->[", sizeof("rx->[")  );
		FreeRTOS_CMD_write( p, UART_XBEE_RXBUFFER_LEN );
		FreeRTOS_CMD_write( "]\r\n", sizeof("]\r\n")  );
		return;
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		pv_cmdRdRTC();
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;

}
/*------------------------------------------------------------------------------------*/
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\x1B[2J\0"));
	FreeRTOS_CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void cmdHelpFunction(void)
{

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SP6K_MODELO, SP6K_VERSION, SP6K_REV, SP6K_DATE);
	FreeRTOS_CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Available commands are:\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-cls\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-reset\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-help\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-status\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-write rtc YYMMDDhhmm\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  btbaud {9600|115200}\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  btcmd {cmd}\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee addr lenght\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprspwr {0|1},gprssw {0|1}, btpwr{0|1} \r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprs|xbee {cmd}\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc YYMMDDhhmm\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-read\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprs|xbee\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

}
/*------------------------------------------------------------------------------------*/
static u08 pv_makeArgv(void)
{
// A partir de la linea de comando, genera un array de punteros a c/token
//
char *token = NULL;
char parseDelimiters[] = " ";
int i = 0;

	// inicialmente todos los punteros deben apuntar a NULL.
	memset(argv, 0, sizeof(argv) );

	// Genero los tokens delimitados por ' '.
	token = strtok(SP5K_CmdlineBuffer, parseDelimiters);
	argv[i++] = token;

	while ( (token = strtok(NULL, parseDelimiters)) != NULL ) {
		argv[i++] = token;
		if (i == 16) break;
	}
	return(( i - 1));
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_OK(void )
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ok\r\n\0"));
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_ERR(void)
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("error\r\n\0"));
	FreeRTOS_CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdRTC(void)
{
RtcTimeType_t rtcDateTime;
s08 retS = FALSE;
u08 pos;

	retS = RTC_read(&rtcDateTime);
	if (retS ) {
		pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n"));
		pos += snprintf_P( &cmd_printfBuff[pos],(sizeof(cmd_printfBuff) - pos ),PSTR("%02d/%02d/%04d "),rtcDateTime.day,rtcDateTime.month, rtcDateTime.year );
		pos += snprintf_P( &cmd_printfBuff[pos],(sizeof(cmd_printfBuff) - pos ),PSTR("%02d:%02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min, rtcDateTime.sec );
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	}
	FreeRTOS_CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
