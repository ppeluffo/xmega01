/*
 * xmega01_utils.c
 *
 *  Created on: 1 de nov. de 2016
 *      Author: pablo
 */

#include "xmega01.h"

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

	// SPI MEM_CS
	MEM_CS_PORT.DIR |= _BV(MEM_CS_PIN);				// MEM_CS output
	MEM_CS_PORT.OUT |= _BV(MEM_CS_PIN);				// MEM_CS High ( disabled )

	// Salidas
	PWR_SENSOR_PORT.DIR |= _BV(PWR_SENSOR_PIN);			// Output
	PWR_SENSOR_PORT.PIN4CTRL = PORT_OPC_WIREDOR_gc;		// Wired OR
	PWR_SENSOR_PORT.OUT &= ~(_BV(PWR_SENSOR_PIN));		// Low

	AN_PWR_3_6V_PORT.DIR |= _BV(AN_PWR_3_6V_PIN);			// Output
	AN_PWR_3_6V_PORT.PIN4CTRL = PORT_OPC_WIREDOR_gc;	// Wired OR
	AN_PWR_3_6V_PORT.OUT &= ~(_BV(AN_PWR_3_6V_PIN));	// Low

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
bool pvSetSensorPwr( uint8_t value )
{

bool retS = false;

	switch ( value ) {
	case 0:
		// Apago
		PWR_SENSOR_PORT.OUT &= ~(_BV(PWR_SENSOR_PIN));	// Low
//		CMD_write( "SENSOR_OFF", sizeof("SENSOR_OFF") );
		retS = true;
		break;
	case 1:
		// Prendo
		PWR_SENSOR_PORT.OUT |= _BV(PWR_SENSOR_PIN);	//  High
//		CMD_write( "SENSOR_ON", sizeof("SENSOR_ON") );
		retS = true;
		break;
	default:
		retS = false;
		break;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
bool pvSetAN3_6Pwr( uint8_t value )
{

bool retS = false;

	switch ( value ) {
	case 0:
		// Apago
		AN_PWR_3_6V_PORT.OUT &= ~(_BV(AN_PWR_3_6V_PIN));	// Low
		retS = true;
		break;
	case 1:
		// Prendo
		AN_PWR_3_6V_PORT.OUT |= _BV(AN_PWR_3_6V_PIN);	//  High
		retS = true;
		break;
	default:
		retS = false;
		break;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
