/*
 * sp5K_i2c.c
 *
 *  Created on: 18/10/2015
 *      Author: pablo
 */


#include "sp5K_i2c.h"
#include "xmega01.h"

void pv_I2C_MasterTransactionFinished( uint8_t result);
void pv_I2C_MasterArbitrationLostBusErrorHandler(void);
void pv_I2C_MasterInterruptHandler(void);
void pv_I2C_MasterWriteHandler(void);
void pv_I2C_MasterReadHandler(void);

//------------------------------------------------------------------------------------
void I2C_MasterInit(int bitrateKHz)
{
u16 bitrate_div;

	// calculate bitrate division
	bitrate_div = ((F_CPU / (2 * ( bitrateKHz * 1000) )) - 5);
	TWIE.MASTER.BAUD = (u08) bitrate_div;

	TWIE.MASTER.CTRLA = TWI_MASTER_INTLVL_LO_gc |
	                    TWI_MASTER_RIEN_bm |
	                    TWI_MASTER_WIEN_bm |
	                    TWI_MASTER_ENABLE_bm;
	TWIE.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
}
//------------------------------------------------------------------------------------
s08 I2C_masterWriteRead ( const u08 slaveI2CAddress, const u08 devAddressLength, const u16 byteAddress, char *pvBuffer, size_t bytesToWrite, size_t bytesToRead  )
{

u08 I2CwriteBuffer_pos = 0;
u08 i;
u08 slaveAddress;

	// Toda operacion I2C ya sea de write o read, comienza escribiendo la direccion del SLAVE en el bus.
	// Luego continua escribiendo la direccion interna en el slave ( 1, 2 o 3 bytes ) desde donde queremos
	// leer o escribir.
	// A partir de aqui podemos leer o escribir en el slave.

	/*Parameter sanity check. */
	if (bytesToWrite > I2C_WRITE_BUFFER_SIZE) {
		return false;
	}
	if (bytesToRead > I2C_READ_BUFFER_SIZE) {
		return false;
	}

	I2CdataStruct.I2CslaveAddress = slaveI2CAddress;

	// Preparo el buffer de escritura con los datos a escribir.
	memset(I2CdataStruct.I2CreadBuffer, '\0', sizeof(I2CdataStruct.I2CreadBuffer) );
	memset(I2CdataStruct.I2CwriteBuffer, '\0', sizeof(I2CdataStruct.I2CwriteBuffer) );

	// Guardo la direccion interna del slave donde leer/escribir.
	// Pass3) Envio la direccion fisica donde comenzar a escribir.
	// En las memorias es una direccion de 2 bytes.En el DS1344 o el BusController es de 1 byte
	// Envio primero el High 8 bit i2c address.
	switch ( devAddressLength ) {
	case 2:
		I2CdataStruct.I2CwriteBuffer[I2CwriteBuffer_pos++] = (byteAddress) >> 8;
		I2CdataStruct.I2CwriteBuffer[I2CwriteBuffer_pos++] = (byteAddress) & 0x00FF;
		break;
	case 1:
		I2CdataStruct.I2CwriteBuffer[I2CwriteBuffer_pos++] = (byteAddress) & 0x00FF;
		break;
	default:
		return(FALSE);
	}
	//
	I2CdataStruct.I2CbytesToRead = bytesToRead;
	I2CdataStruct.I2CbytesToWrite = bytesToWrite + I2CwriteBuffer_pos;
	I2CdataStruct.I2CbytesRead = 0;
	I2CdataStruct.I2CbytesWritten = 0;
	//
	/*Initiate transaction if bus is ready. */
	if ( TWIE.MASTER.STATUS == TWIM_STATUS_READY) {

		TWIE.MASTER.STATUS = TWIM_STATUS_BUSY;
		I2CdataStruct.I2Cresult = TWIM_RESULT_UNKNOWN;

		// Si la operacion es de escritura, cargo el I2Cwrite buffer con los datos.
		for ( i =0; i < bytesToWrite; i++) {
			I2CdataStruct.I2CwriteBuffer[I2CwriteBuffer_pos++] = pvBuffer[i];
		}

		// Arranco la secuencia con un ciclo de escritura de la direccion del slave
		// El resto se hace por interrupcion.

		/* If write command, send the START condition + Address +
		 * 'R/_W = 0'
		 */
		slaveAddress = slaveI2CAddress & ~0x01;
		TWIE.MASTER.ADDR = slaveAddress;
		return true;
	} else {
		return false;
	}

}
//------------------------------------------------------------------------------------
s08 I2C_masterWrite ( const u08 slaveI2CAddress, const u08 devAddressLength, const u16 byteAddress, char *pvBuffer, size_t xBytes  )
{
/*
u08 tryes = 0;
u08 i2c_status;
char txbyte;
s08 retV = FALSE;
u08 i;

i2c_retry:

#ifdef DEBUG_I2C
	snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MW0: 0x%02x,0x%02x,0x%02x,0x%02x\r\n\0"),devAddress,devAddressLength, (u16)(byteAddress), xBytes );
	FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif

	if (tryes++ >= I2C_MAXTRIES) goto i2c_quit;

	// Pass1) START: Debe retornar 0x08 (I2C_START) o 0x10 (I2C_REP_START) en condiciones normales
	pvI2C_sendStart();
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
	snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MW1: 0x%02x\r\n\0"),i2c_status );
	FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif
	if ( i2c_status == TW_MT_ARB_LOST) goto i2c_retry;
	if ( (i2c_status != TW_START) && (i2c_status != TW_REP_START) ) goto i2c_quit;

	// Pass2) (SLA_W) Send slave address. Debo recibir 0x18 ( SLA_ACK )
	txbyte = devAddress | TW_WRITE;
	pvI2C_sendByte(txbyte);
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
	snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MW2: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
	FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif
	// Check the TWSR status
	if ((i2c_status == TW_MT_SLA_NACK) || (i2c_status == TW_MT_ARB_LOST)) goto i2c_retry;
	if (i2c_status != TW_MT_SLA_ACK) goto i2c_quit;

	// Pass3) Envio la direccion fisica donde comenzar a escribir.
	// En las memorias es una direccion de 2 bytes.En el DS1344 o el BusController es de 1 byte
	// Envio primero el High 8 bit i2c address. Debo recibir 0x28 ( DATA_ACK)
	if ( devAddressLength == 2 ) {
		txbyte = (byteAddress) >> 8;
		pvI2C_sendByte(txbyte);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
		snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MW3H: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
		FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif
		if (i2c_status != TW_MT_DATA_ACK) goto i2c_quit;
	}

	// Envio el Low 8 byte i2c address.
	if ( devAddressLength >= 1 ) {
		txbyte = (byteAddress) & 0x00FF;
		pvI2C_sendByte(txbyte);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
		snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MW3L: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
		FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif
		if (i2c_status != TW_MT_DATA_ACK) goto i2c_quit;
	}

	// Pass4) Trasmito todos los bytes del buffer. Debo recibir 0x28 (DATA_ACK)
	for ( i=0; i < xBytes; i++ ) {
		txbyte = *pvBuffer++;
		pvI2C_sendByte(txbyte);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
		snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MW4: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
		FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif
		if (i2c_status != TW_MT_DATA_ACK) goto i2c_quit;
	}

	retV = TRUE;

i2c_quit:

	// Pass5) STOP
	pvI2C_sendStop();
	vTaskDelay( ( TickType_t)( 20 / portTICK_RATE_MS ) );

	// En caso de error libero la interface.
	if (retV == FALSE)
		pvI2C_disable();

	return(retV);

*/
}
//------------------------------------------------------------------------------------
s08 I2C_masterRead  ( const u08 devAddress, const u08 devAddressLength, const u16 byteAddress, char *pvBuffer, size_t xBytes  )
{
	// En el caso del ADC, el read no lleva la parte de mandar la SLA+W. !!!!!
/*
u08 tryes = 0;
u08 i2c_status;
char txbyte;
s08 retV = FALSE;
u08 i;

i2c_retry:

#ifdef DEBUG_I2C
	snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MR0: 0x%02x,0x%02x,0x%02x,0x%02x\r\n\0"),devAddress,devAddressLength, (u16)(byteAddress), xBytes );
	FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif

	if (tryes++ >= I2C_MAXTRIES) goto i2c_quit;

	// Pass1) START: Debe retornar 0x08 (I2C_START) o 0x10 (I2C_REP_START) en condiciones normales
	pvI2C_sendStart();
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
	snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MR1: 0x%02x\r\n\0"),i2c_status );
	FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif
	if ( i2c_status == TW_MT_ARB_LOST) goto i2c_retry;
	if ( (i2c_status != TW_START) && (i2c_status != TW_REP_START) ) goto i2c_quit;

	// En el caso del ADC, el read no lleva la parte de mandar la SLA+W. !!!!!
	if ( devAddress == ADS7828_ADDR)
		goto SRL_R;

	// Pass2) (SLA_W) Send slave address. Debo recibir 0x18 ( SLA_ACK )
	txbyte = devAddress | TW_WRITE;
	pvI2C_sendByte(txbyte);
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	// Check the TWSR status
	i2c_status = pvI2C_getStatus();
	if ((i2c_status == TW_MT_SLA_NACK) || (i2c_status == TW_MT_ARB_LOST)) goto i2c_retry;
	if (i2c_status != TW_MT_SLA_ACK) goto i2c_quit;
#ifdef DEBUG_I2C
	snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MR2: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
	FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif
	// Pass3) Envio la direccion fisica donde comenzar a escribir.
	// En las memorias es una direccion de 2 bytes.En el DS1344 o el BusController es de 1 byte
	// Envio primero el High 8 bit i2c address. Debo recibir 0x28 ( DATA_ACK)
	if ( devAddressLength == 2 ) {
		txbyte = (byteAddress) >> 8;
		pvI2C_sendByte(txbyte);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
		snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MR3H: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
		FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif
		if (i2c_status != TW_MT_DATA_ACK) goto i2c_quit;
	}

	// Envio el Low 8 byte i2c address.
	if ( devAddressLength >= 1 ) {
		txbyte = (byteAddress) & 0x00FF;
		pvI2C_sendByte(txbyte);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
		snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MR3L: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
		FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif
		if (i2c_status != TW_MT_DATA_ACK) goto i2c_quit;
	}


	// Pass4) REPEATED START. Debo recibir 0x10 ( REPEATED_START)
	pvI2C_sendStart();
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
	snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MR4: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
	FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif
	if (i2c_status == TW_MT_ARB_LOST) goto i2c_retry;
	if ( (i2c_status != TW_START) && (i2c_status != TW_REP_START) ) goto i2c_quit;

SRL_R:
	// Pass5) (SLA_R) Send slave address + READ. Debo recibir un 0x40 ( SLA_R ACK)
	txbyte = devAddress | TW_READ;
	pvI2C_sendByte(txbyte);
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	// Check the TWSR status
	i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
	snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MR5: 0x%02x,0x%02x\r\n\0"),txbyte,i2c_status );
	FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif
	if ((i2c_status == TW_MR_SLA_NACK) || (i2c_status == TW_MR_ARB_LOST)) goto i2c_retry;
	if (i2c_status != TW_MR_SLA_ACK) goto i2c_quit;

	// Pass6) Leo todos los bytes requeridos y respondo con ACK.
	for ( i=0; i < (xBytes-1); i++ ) {
		pvI2C_readByte(ACK);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
		snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MR6(%d): 0x%02x,0x%02x\r\n\0"),i,TWDR,i2c_status );
		FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif
		if (i2c_status != TW_MR_DATA_ACK) goto i2c_quit;
		*pvBuffer++ = TWDR;
	}

	// Pass7) Acepto el ultimo byte y respondo con NACK
	pvI2C_readByte(NACK);
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	i2c_status = pvI2C_getStatus();
#ifdef DEBUG_I2C
	snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MR6(%d): 0x%02x,0x%02x\r\n\0"),i,TWDR,i2c_status );
	FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
#endif
	if (i2c_status != TW_MR_DATA_NACK) goto i2c_quit;
	*pvBuffer++ = TWDR;

	// I2C read OK.
	retV = TRUE;

i2c_quit:

	// Pass5) STOP
	pvI2C_sendStop();

	// En caso de error libero la interface.
	if (retV == FALSE)
		pvI2C_disable();

//	if ( retV ) {
//		snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MR RET TRUE ( %d )\r\n\0"),retV );
//	} else {
//		snprintf_P( d_printfBuff,CHAR128,PSTR("I2C_MR RET FALSE ( %d )\r\n\0"),retV );
//	}

//	FreeRTOS_CMD_write( d_printfBuff, sizeof(d_printfBuff) );
	return(retV);
*/
}
//------------------------------------------------------------------------------------
// FUNCIONES AUXILIARES PRIVADAS DE I2C
//------------------------------------------------------------------------------------
/*! TWIE Master Interrupt vector. */
ISR(TWIE_TWIM_vect)
{
	pv_I2C_MasterInterruptHandler();
}
//------------------------------------------------------------------------------------
void pv_I2C_MasterInterruptHandler(void)
{
	uint8_t currentStatus = TWIE.MASTER.STATUS;

	/* If arbitration lost or bus error. */
	if ((currentStatus & TWI_MASTER_ARBLOST_bm) ||
	    (currentStatus & TWI_MASTER_BUSERR_bm)) {

		pv_I2C_MasterArbitrationLostBusErrorHandler();
	}

	/* If master write interrupt. */
	else if (currentStatus & TWI_MASTER_WIF_bm) {
		pv_I2C_MasterWriteHandler();
	}

	/* If master read interrupt. */
	else if (currentStatus & TWI_MASTER_RIF_bm) {
		pv_I2C_MasterReadHandler();
	}

	/* If unexpected state. */
	else {
		pv_I2C_MasterTransactionFinished( TWIM_RESULT_FAIL);
	}
}
//------------------------------------------------------------------------------------
void pv_I2C_MasterArbitrationLostBusErrorHandler(void)
{
	uint8_t currentStatus = TWIE.MASTER.STATUS;

	/* If bus error. */
	if (currentStatus & TWI_MASTER_BUSERR_bm) {
		I2CdataStruct.I2Cresult = TWIM_RESULT_BUS_ERROR;
	}
	/* If arbitration lost. */
	else {
		I2CdataStruct.I2Cresult = TWIM_RESULT_ARBITRATION_LOST;
	}

	/* Clear interrupt flag. */
	TWIE.MASTER.STATUS = currentStatus | TWI_MASTER_ARBLOST_bm;

	I2CdataStruct.I2Cstatus = TWIM_STATUS_READY;
}
//------------------------------------------------------------------------------------
void pv_I2C_MasterWriteHandler(void)
{
	/* Local variables used in if tests to avoid compiler warning. */

uint8_t data;
uint8_t readAddress;

	/* If NOT acknowledged (NACK) by slave cancel the transaction. */
	if ( TWIE.MASTER.STATUS & TWI_MASTER_RXACK_bm) {
		TWIE.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		I2CdataStruct.I2Cresult = TWIM_RESULT_NACK_RECEIVED;
		I2CdataStruct.I2Cstatus = TWIM_STATUS_READY;
	}

	/* If more bytes to write, send data. */
	else if ( I2CdataStruct.I2CbytesWritten < I2CdataStruct.I2CbytesToWrite ) {
		data = I2CdataStruct.I2CwriteBuffer[I2CdataStruct.I2CbytesWritten];
		TWIE.MASTER.DATA = data;
		++I2CdataStruct.I2CbytesWritten;
	}

	/* If bytes to read, send repeated START condition + Address +
	 * 'R/_W = 1'
	 */
	else if (I2CdataStruct.I2CbytesRead < I2CdataStruct.I2CbytesToRead ) {
		readAddress = I2CdataStruct.I2CslaveAddress | 0x01;
		TWIE.MASTER.ADDR = readAddress;
	}

	/* If transaction finished, send STOP condition and set RESULT OK. */
	else {
		TWIE.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		pv_I2C_MasterTransactionFinished(TWIM_RESULT_OK);
	}
}
//------------------------------------------------------------------------------------
void pv_I2C_MasterReadHandler(void)
{

uint8_t data;

	/* Fetch data if bytes to be read. */
	if ( I2CdataStruct.I2CbytesRead < I2C_READ_BUFFER_SIZE ) {
		data = TWIE.MASTER.DATA;
		I2CdataStruct.I2CreadBuffer[I2CdataStruct.I2CbytesRead] = data;
		I2CdataStruct.I2CbytesRead++;
	}

	/* If buffer overflow, issue STOP and BUFFER_OVERFLOW condition. */
	else {
		TWIE.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		pv_I2C_MasterTransactionFinished( TWIM_RESULT_BUFFER_OVERFLOW);
	}

	/* If more bytes to read, issue ACK and start a byte read. */
	if ( I2CdataStruct.I2CbytesRead < I2CdataStruct.I2CbytesToRead ) {
		TWIE.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;
	}

	/* If transaction finished, issue NACK and STOP condition. */
	else {
		TWIE.MASTER.CTRLC = TWI_MASTER_ACKACT_bm |
		                               TWI_MASTER_CMD_STOP_gc;
		pv_I2C_MasterTransactionFinished( TWIM_RESULT_OK);
	}
}
//------------------------------------------------------------------------------------
void pv_I2C_MasterTransactionFinished( uint8_t result)
{
	I2CdataStruct.I2Cresult = result;
	I2CdataStruct.I2Cstatus = TWIM_STATUS_READY;
}
//------------------------------------------------------------------------------------
