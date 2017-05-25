/*
 * ee_sp5K.c
 *
 *  Created on: 26/10/2015
 *      Author: pablo
 */

#include "../sp6Klibs/l_eeprom.h"

//------------------------------------------------------------------------------------
bool EE_read( uint16_t eeAddress, char *data, uint8_t length )
{
	// Lee en la EE, desde la posicion 'address', 'length' bytes
	// y los deja en el buffer apuntado por 'data'

size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;

		// Lo primero es obtener el semaforo
		FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
		// Luego indicamos el periferico i2c en el cual queremos leer
		val = EE_ADDR;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
		// Luego indicamos la direccion desde donde leer: largo ( 2 bytes )
		val = 2;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
		// y direccion
		val = eeAddress;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);

		// Por ultimo leemos.
		xBytes = length;
		xReturn = FreeRTOS_read(&pdI2C, data, xBytes);
		// Y libero el semaforo.
		FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

		if (xReturn != xBytes ) {
			return ( false );
		}

		return(true);

}
//------------------------------------------------------------------------------------
bool EE_write( uint16_t eeAddress, char *data, uint8_t length )
{
	// Escribe en la EE a partir de la posicion 'address', la cantidad
	// 'length' de bytes apuntados por 'data'
	// Puedo estar escribiendo un pageWrite entonces debo controlar no
	// salime de la pagina.
	//
size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;
uint16_t n, pageBytesFree;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = EE_ADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
	// Luego indicamos la direccion a partir de donde escribir: largo ( 2 bytes )
	val = 2;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// y direccion
	val = eeAddress;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);

	// Por ultimo escribimos xBytes.
	// Controlo no hacer page overflow
	xBytes = length;
	n = length % EE_PAGESIZE;
	pageBytesFree = (n+1)*EE_PAGESIZE - length;
	if ( pageBytesFree < length ) {
		xBytes = pageBytesFree;
	}
	xReturn = FreeRTOS_write(&pdI2C, data, xBytes);
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	if (xReturn != xBytes ) {
		return ( false );
	}

	return(true);

}
//------------------------------------------------------------------------------------
bool EE_test_write(char *s0, char *s1)
{
uint8_t length = 0;
char *p;
bool retS = false;

	p = s1;
	while (*p != 0) {
		p++;
		length++;
	}
//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("S=[%s](%d)\r\n\0"),s1, length);
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );


	retS = EE_write( (uint16_t)(atoi(s0)), s1, length );
	return(retS);
}
//-----------------------------------------------------------------------------------
bool EE_test_read(char *s0, char *s1, char *s2)
{

bool retS = false;

	retS = EE_read( (uint16_t)(atoi(s0)), s1, (uint8_t)(atoi(s2)) );
	return(retS);
}
//-----------------------------------------------------------------------------------
