/*
 * xmega01_utils.c
 *
 *  Created on: 1 de nov. de 2016
 *      Author: pablo
 */

#include "xmega01.h"

/*------------------------------------------------------------------------------------*/
void WDT_EnableAndSetTimeout( WDT_PER_t period )
{

uint8_t temp = WDT_ENABLE_bm | WDT_CEN_bm | period;

	CCP = CCP_IOREG_gc;
	WDT.CTRL = temp;

	/* Wait for WD to synchronize with new settings. */
	while(WDT_IsSyncBusy()){

	}
}
/*------------------------------------------------------------------------------------*/
void WDT_DisableWindowMode( void )
{
	uint8_t temp = (WDT.WINCTRL & ~WDT_WEN_bm) | WDT_WCEN_bm;
	CCP = CCP_IOREG_gc;
	WDT.WINCTRL = temp;
}
/*------------------------------------------------------------------------------------*/
void configureTimer(void)
{
	/* Configuro el TIMER0 ( 16 bits ) como fuente de interrupcion de ticks
	 * El micro trabaja a F_CPU = 32000000 ( 32Mhz ).
	 * En FreeRTOSConfig.h tengo 2 constantes que necesito
	 * configCPU_CLOCK_HZ = 32000000
	 * configTICK_RATE_HZ = 100.
	 * Esta ultima me marca el periodo que necesito generar de interrupciones ( 10ms)
	 *
	 * El TC0 lo uso como contador y la interrupcion es la de OVERFLOW.
	 * Requiero un prescaler y un valor PER que al alcanzarlo genera la interrupcion.
	 *
	 * El prescaler que uso de de 256 de modo que la frecuencia del contador
	 * es de 32Mhz/256 = 125000.
	 *
	 * Si cuento hasta 1250 ( PER ), entonces el periodo de interrupcion es de 10ms.
	 *
	 */

	TCC0.CTRLA = 0x06;		// Fijo el prescaler en 256.
	TCC0.PER = 0x4E2;		// PER = 1250
	// Con esto hago overflow c/10ms

	TCC0.INTCTRLA = 0x01;	// Habilito la interrupcion de overflow de TC0 y la configuro como low-level

	PMIC.CTRL |= PMIC_LOLVLEN_bm;	// Habilito las interrupciones de low-level

	TCC0.INTFLAGS = TC0_OVFIF_bm;	// Habilito las interrupciones en general
	sei();

}
//-----------------------------------------------------------
void SET_systemMainClock(void)
{
	// Configura el clock principal del sistema

//	pvEnable2MHZclock();
	pvEnable32MHZclock();


}
//-----------------------------------------------------------
void pvEnable2MHZclock(void)
{
	/*  Enable internal 2 MHz rc oscillator and wait until it's
	 *  stable. Set the 2 MHz rc oscillator as the main clock source.
	 *  Then disable other oscillators.
		*/
	CLKSYS_Enable( OSC_RC2MEN_bm );
	do {} while ( CLKSYS_IsReady( OSC_RC2MRDY_bm ) == 0 );
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC2M_gc );
	CLKSYS_Disable( OSC_RC32MEN_bm | OSC_RC32KEN_bm  );
//	CLKSYS_Disable( OSC_PLLEN_bm );

}
//-----------------------------------------------------------
void pvEnable32MHZclock(void)
{
	/*  Enable internal 32 MHz rc oscillator and wait until it's
	 *  stable. Set the 32 MHz rc oscillator as the main clock source.
	 *  Then disable other oscillators.
		*/
	CLKSYS_Enable( OSC_RC32MEN_bm );
	do {} while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );
//	CLKSYS_Disable( OSC_RC2MEN_bm | OSC_RC32KEN_bm  );
//	CLKSYS_Disable( OSC_PLLEN_bm );

}
//-----------------------------------------------------------
s08 pvConfigBTbaud(char *s_baud)
{
	// Reconfigura el puerto serial del blueTooth.

long baud;

	baud =  atol(s_baud);
	switch (baud) {
	case 9600:
//		USARTD0.BAUDCTRLA = (uint8_t) 1659;
//		USARTD0.BAUDCTRLB = ( -3 << USART_BSCALE0_bp)|(1659 >> 8);
		snprintf_P( d_printfBuff,sizeof(d_printfBuff),PSTR("BAUD9600\r\n"));
		FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
		break;
	case 115200:
//		USARTD0.BAUDCTRLA = (uint8_t) 2094;
//		USARTD0.BAUDCTRLB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);
		snprintf_P( d_printfBuff,sizeof(d_printfBuff),PSTR("BAUD115200\r\n"));
		FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
		break;
	}
	return(TRUE);
}
//-----------------------------------------------------------
void initMCU(void)
{
	// Inicializa los pines del micro
	// USB:
	USB_RTS_PORT.DIR &= ~(_BV(USB_RTS_PIN));	// RTS input

	USB_CTS_PORT.DIR |= _BV(USB_CTS_PIN);		// CTS output
	USB_CTS_PORT.OUT &= ~(_BV(USB_CTS_PIN));	// CTS Low

	// BT
	BT_PWR_PORT.DIR |= _BV(BT_PWR_PIN);			// PWR output
	BT_PWR_PORT.OUT |= _BV(BT_PWR_PIN);			// PWR High ( prendido )

	BT_RTS_PORT.DIR &= ~(_BV(BT_RTS_PIN));		// RTS input

	BT_CTS_PORT.DIR |= _BV(BT_CTS_PIN);			// CTS output
	BT_CTS_PORT.OUT &= ~(_BV(BT_CTS_PIN));		// CTS Low

	// GPRS:
	GPRS_RTS_PORT.DIR |= _BV(GPRS_RTS_PIN);		// RTS output
	GPRS_RTS_PORT.OUT &= ~(_BV(GPRS_RTS_PIN));	// RTS Low

	GPRS_CTS_PORT.DIR &= ~(_BV(GPRS_CTS_PIN));	// CTS input

	GPRS_HWPWR_PORT.DIR |= _BV(GPRS_HWPWR_PIN);		// HWPWR output
	GPRS_HWPWR_PORT.OUT &= ~(_BV(GPRS_HWPWR_PIN));	// HWPWR Low (apagado)

	GPRS_SWPWR_PORT.DIR |= _BV(GPRS_SWPWR_PIN);		// SWPWR output
	GPRS_SWPWR_PORT.OUT &= ~(_BV(GPRS_SWPWR_PIN));	// SWPWR Low

	// XBEE
	XBEE_RTS_PORT.DIR |= _BV(XBEE_RTS_PIN);			// RTS output
	XBEE_RTS_PORT.OUT &= ~(_BV(XBEE_RTS_PIN));		// RTS Low

	XBEE_CTS_PORT.DIR &= ~(_BV(XBEE_CTS_PIN)	);	// CTS input
	XBEE_SLEEP_PORT.DIR &= ~(_BV(XBEE_SLEEP_PIN));	// SLEEP input

//	XBEE_RESET_PORT.DIR |= _BV(XBEE_RESET_PIN);		// RESET output
//	XBEE_RESET_PORT.OUT &= ~(_BV(XBEE_RESET_PIN));	// RESET Low
	XBEE_RESET_PORT.DIR &= ~(_BV(XBEE_RESET_PIN));	// RESET input

}
//-----------------------------------------------------------
s08 pvSetGprsPwr( u08 value )
{

s08 retS = FALSE;

	switch ( value ) {
	case 0:
		// Apago el GPRS
		GPRS_HWPWR_PORT.OUT &= ~(_BV(GPRS_HWPWR_PIN));	// HWPWR Low (apagado)
		retS = TRUE;
		break;
	case 1:
		// Prendo el GPRS
		GPRS_HWPWR_PORT.OUT |= _BV(GPRS_HWPWR_PIN);	//  High
		retS = TRUE;
		break;
	default:
		retS = FALSE;
		break;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
s08 pvSetGprsSw( u08 value )
{

s08 retS = FALSE;

	switch ( value ) {
	case 0:
		//
		GPRS_SWPWR_PORT.OUT &= ~(_BV(GPRS_SWPWR_PIN));	// Low
		retS = TRUE;
		break;
	case 1:
		GPRS_SWPWR_PORT.OUT |= _BV(GPRS_SWPWR_PIN);	//  High
		retS = TRUE;
		break;
	default:
		retS = FALSE;
		break;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
s08 pvSetGprsRts( u08 value )
{

s08 retS = FALSE;

	switch ( value ) {
	case 0:
		// RTS Low
		GPRS_RTS_PORT.OUT &= ~(_BV(GPRS_RTS_PIN));	// RTS Low
		retS = TRUE;
		break;
	case 1:
		// RTS high
		GPRS_RTS_PORT.OUT |= _BV(GPRS_RTS_PIN);		// RTS High
		retS = TRUE;
		break;
	default:
		retS = FALSE;
		break;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
s08 pvSetBtPwr( u08 value )
{

s08 retS = FALSE;

	switch ( value ) {
	case 0:
		// Apago el BT
		BT_PWR_PORT.OUT &= ~(_BV(BT_PWR_PIN));	// PWR Low ( apagado )
		retS = TRUE;
		break;
	case 1:
		// Prendo el BT
		BT_PWR_PORT.OUT |= _BV(BT_PWR_PIN);		// PWR High ( prendido )
		retS = TRUE;
		break;
	default:
		retS = FALSE;
		break;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
s08 u_wrRtc(char *s)
{
u08 dateTimeStr[11];
char tmp[3];
s08 retS;
RtcTimeType_t rtcDateTime;


	/* YYMMDDhhmm */
	if ( s == NULL )
		return(FALSE);

	memcpy(dateTimeStr, s, 10);
	// year
	tmp[0] = dateTimeStr[0]; tmp[1] = dateTimeStr[1];	tmp[2] = '\0';
	rtcDateTime.year = atoi(tmp);
	// month
	tmp[0] = dateTimeStr[2]; tmp[1] = dateTimeStr[3];	tmp[2] = '\0';
	rtcDateTime.month = atoi(tmp);
	// day of month
	tmp[0] = dateTimeStr[4]; tmp[1] = dateTimeStr[5];	tmp[2] = '\0';
	rtcDateTime.day = atoi(tmp);
	// hour
	tmp[0] = dateTimeStr[6]; tmp[1] = dateTimeStr[7];	tmp[2] = '\0';
	rtcDateTime.hour = atoi(tmp);
	// minute
	tmp[0] = dateTimeStr[8]; tmp[1] = dateTimeStr[9];	tmp[2] = '\0';
	rtcDateTime.min = atoi(tmp);

	retS = RTC_write(&rtcDateTime);
	return(retS);
}
//------------------------------------------------------------------------------------
