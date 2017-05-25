/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */


#include "sp6Klibs/l_eeprom.h"
#include "sp6Klibs/l_uarts.h"
#include "xmega01.h"

static char cmd_printfBuff[256];
char *argv[16];

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);
static u08 pv_makeArgv(void);

bool pvConfigRTC(char *str );

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
	cmdlineSetOutputFunc( CMD_writeChar);

	// Registro los comandos y los mapeo al cmd string.
	cmdlineAddCommand((u08 *)("cls"), cmdClearScreen );
	cmdlineAddCommand((u08 *)("help"), cmdHelpFunction);
	cmdlineAddCommand((u08 *)("reset"), cmdResetFunction);
	cmdlineAddCommand((u08 *)("write"), cmdWriteFunction);
	cmdlineAddCommand((u08 *)("read"), cmdReadFunction);

	// Espero la notificacion para arrancar
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("starting tkCmd..\r\n\0"));
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Fijo el timeout del READ
	ticks = 5;
	FreeRTOS_ioctl( &pdUART_USB,ioctlSET_TIMEOUT, &ticks );
	FreeRTOS_ioctl( &pdUART_BT,ioctlSET_TIMEOUT, &ticks );

	// loop
	for( ;; )
	{

		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 50ms. lo que genera la espera.
		while ( CMD_read( &c, 1 ) == 1 ) {
			cmdlineInputFunc(c);
		}

		/* Ejecuto el interprete de comandos */
		cmdlineMainLoop();
		//vTaskDelay( ( TickType_t)( 1 ));

	}

}
/*------------------------------------------------------------------------------------*/
static void cmdResetFunction(void)
{
	// Resetea al micro prendiendo el watchdog

uint8_t temp;

 	//Software reset
	CCP = 0xD8;			// Protected configuration signature
	RST.CTRL = 0x01;	// Reset.

	while(1)
		;

/*
	temp = WDT_ENABLE_bm | WDT_CEN_bm |  WDT_PER_8KCLK_gc ;
	CCP = CCP_IOREG_gc;
	WDT.CTRL = temp;
	while(WDT_IsSyncBusy());

	WDT_Reset();

	while(1)
		;
*/
}
/*------------------------------------------------------------------------------------*/
static void cmdWriteFunction(void)
{
s08 retS = FALSE;
u08 argc;

	argc = pv_makeArgv();


	// TESTRTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTCTEST\0") ) ) {
		RTC_test();
		return;
	}

	// SENSORPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("SENSORPWR\0") ) ) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON\0") ) ) {
			retS = pvSetSensorPwr(1);
		} else if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0") ) ) {
			retS = pvSetSensorPwr(0);
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// AN3.6PWR
	if (!strcmp_P( strupr(argv[1]), PSTR("ANPWR\0") ) ) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON\0") ) ) {
			retS = pvSetAN3_6Pwr(1);
		} else if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0") ) ) {
			retS = pvSetAN3_6Pwr(0);
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// EEPROM
	// EE: write ee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0")) ) {
		retS = EE_test_write( argv[2], argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
static void cmdReadFunction(void)
{
char *p;
u08 argc;
bool retS;

	argc = pv_makeArgv();


	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {

		memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		retS = EE_test_read( argv[2], cmd_printfBuff, argv[3] );
		if ( retS ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			snprintf_P( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff),PSTR( "\r\n\0"));
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;

}
/*------------------------------------------------------------------------------------*/
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\x1B[2J\0"));
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void cmdHelpFunction(void)
{

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SP6K_MODELO, SP6K_VERSION, SP6K_REV, SP6K_DATE);
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Available commands are:\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-cls\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-reset\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-help\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-status\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-write\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  sensorpwr,anpwr {on|off}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee {pos}{string}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtctest\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-read\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee {pos}{lenght}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

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
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_ERR(void)
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("error\r\n\0"));
	CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
