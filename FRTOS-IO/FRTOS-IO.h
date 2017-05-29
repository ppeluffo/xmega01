/*
 * FRTOS-IO.h
 *
 *  Created on: 2/10/2015
 *      Author: pablo
 */

#ifndef SRC_FRTOS_IO_FRTOS_IO_H_
#define SRC_FRTOS_IO_FRTOS_IO_H_

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "../drivers/sp6Kboard/sp6K_uart.h"
#include "../drivers/sp6Kboard/sp6K_i2c.h"
#include "../drivers/sp6Kboard/sp6K_rtc.h"
#include "../drivers/sp6Kboard/sp6K_spi.h"

// Describo los diferentes perifericos a los  que puede acceder la aplicacion.
// Para c/u voy a implementar funciones de init,write,read.
typedef enum {
	pUART_USB = 0,
	pUART_BT,
	pUART_GPRS,
	pUART_XBEE,
	pI2C,
	pRTC,
	pSPI,
} t_perifericos;

//#define DEBUG_I2C

/* Peripheral handles are void * for data hiding purposes. */
typedef const void * Peripheral_Descriptor_t;

/* Types that define valid read(), write() and ioctl() functions. */
typedef size_t ( *Peripheral_write_Function_t )( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes );
typedef size_t ( *Peripheral_read_Function_t )( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes );
typedef portBASE_TYPE ( *Peripheral_ioctl_Function_t )( Peripheral_Descriptor_t const pxPeripheral, uint32_t ulRequest, void *pvValue );

//------------------------------------------------------------------------------------
// Estructura generica de todos los perifericos.
// Los elementos particulares quedan en una estructura que se accede por el punter
// phDevice

typedef struct {

	t_perifericos	portId;
	SemaphoreHandle_t xBusSemaphore;		//
	uint8_t xBlockTime;						// ticks to block in read operations. Set by ioctl

	Peripheral_write_Function_t write;		// The function used to write to the peripheral
	Peripheral_read_Function_t read;		// The function used to read from the peripheral
	Peripheral_ioctl_Function_t ioctl;		// The function used for ioctl access to the peripheral

	void *phDevice;			// puntero a una estructura con los miembros adecuados al periferico

} Peripheral_Control_t;

// Dispositivos a los que voy a poder acceder.
Peripheral_Control_t pdUART_USB, pdUART_BT, pdUART_GPRS, pdUART_XBEE, pdI2C, pdRTC, pdSPI;

Peripheral_Descriptor_t FreeRTOS_open(const uint8_t port, const uint32_t flags);
int FreeRTOS_ioctl( Peripheral_Descriptor_t const xPeripheral, uint32_t ulRequest, void *pvValue );
#define FreeRTOS_write( xPeripheral, pvBuffer, xBytes ) ( ( Peripheral_Control_t * ) xPeripheral )->write( ( ( Peripheral_Control_t * ) xPeripheral ), ( pvBuffer ), ( xBytes ) )
#define FreeRTOS_read( xPeripheral, pvBuffer, xBytes ) ( ( Peripheral_Control_t * ) xPeripheral )->read( ( ( Peripheral_Control_t * ) xPeripheral ), ( pvBuffer ), ( xBytes ) )

//------------------------------------------------------------------------------------
// Estructura particular de UARTs
//------------------------------------------------------------------------------------
#define FIFO	0
#define QUEUE	1
//-----------------------------------------------------------------------
// Defino el tipo de almacenamiento que requiere la aplicacion
// 0: queue, 1: fifo
#define UART_RXFIFO		1
#define UART_TXFIFO		2
#define UART_RXQUEUE	4
#define UART_TXQUEUE	8
//-----------------------------------------------------------------------
// Defino el tamanio de los buffers
#define  UART_USB_RXBUFFER_LEN ( ( uint8_t ) ( 128 ))
#define  UART_USB_TXBUFFER_LEN ( ( uint8_t ) ( 128 ))

#define  UART_BT_RXBUFFER_LEN ( ( uint8_t ) ( 128 ))
#define  UART_BT_TXBUFFER_LEN ( ( uint8_t ) ( 128 ))

#define  UART_GPRS_RXBUFFER_LEN ( ( uint16_t ) ( 640 ))
#define  UART_GPRS_TXBUFFER_LEN ( ( uint8_t ) ( 128 ))

#define  UART_XBEE_RXBUFFER_LEN ( ( uint8_t ) ( 128 ))
#define  UART_XBEE_TXBUFFER_LEN ( ( uint8_t ) ( 128 ))

typedef struct {

	// En las UART solo requiero un estructura de almacenamiento de datos.
	uint8_t rxBufferType;
	void *rxStruct;			// uso puntero void para poder asignarle cualquier estructura.
	uint16_t rxBufferLength;

	uint8_t txBufferType;
	void *txStruct;
	uint16_t txBufferLength;

} UART_device_control_t;

//------------------------------------------------------------------------------------

int FreeRTOS_UART_open( Peripheral_Control_t * const pxPeripheralControl, const uint32_t flags );
size_t FreeRTOS_UART_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes );
size_t FreeRTOS_UART_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes );
portBASE_TYPE FreeRTOS_UART_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue );

char *FreeRTOS_UART_getFifoPtr(Peripheral_Control_t *UART);

//------------------------------------------------------------------------------------
// Estructura particular de I2Cs
//------------------------------------------------------------------------------------

typedef struct {

	// En las I2C solo requiero un estructura de almacenamiento de datos.
	uint8_t devAddress;
	uint8_t byteAddressLength;
	uint16_t byteAddress;

} I2C_device_control_t;

portBASE_TYPE FreeRTOS_I2C_open( Peripheral_Control_t * const pxPeripheralControl, const uint32_t flags );
size_t FreeRTOS_I2C_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes );
size_t FreeRTOS_I2C_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes );
portBASE_TYPE FreeRTOS_I2C_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue );

//------------------------------------------------------------------------------------
// Estructura particular de RTC
//------------------------------------------------------------------------------------

typedef struct {

	// En las I2C solo requiero un estructura de almacenamiento de datos.
	uint8_t devAddress;
	uint8_t byteAddressLength;
	uint16_t byteAddress;

} RTC_device_control_t;

portBASE_TYPE FreeRTOS_RTC_open( Peripheral_Control_t * const pxPeripheralControl, const uint32_t flags );
size_t FreeRTOS_RTC_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes );
size_t FreeRTOS_RTC_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes );
portBASE_TYPE FreeRTOS_RTC_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue );

//------------------------------------------------------------------------------------
// Estructura particular de SPI
//------------------------------------------------------------------------------------

typedef struct {

	// En las SPI solo requiero un estructura de almacenamiento de datos.
	uint8_t devAddress;
	uint8_t byteAddressLength;
	uint16_t byteAddress;

} SPI_device_control_t;

portBASE_TYPE FreeRTOS_SPI_open( Peripheral_Control_t * const pxPeripheralControl, const uint32_t flags );
size_t FreeRTOS_SPI_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes );
size_t FreeRTOS_SPI_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes );
portBASE_TYPE FreeRTOS_SPI_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue );

//------------------------------------------------------------------------------------

#define ioctlQUEUE_FLUSH			1
#define ioctlUART_ENABLE			2
#define ioctlUART_DISABLE			3
#define ioctlOBTAIN_BUS_SEMPH		4
#define ioctlRELEASE_BUS_SEMPH		5

#define ioctlSET_TIMEOUT			6
#define ioctl_UART_CLEAR_RX_BUFFER	7
#define ioctl_UART_CLEAR_TX_BUFFER	8

#define ioctl_I2C_SET_DEVADDRESS			10
#define ioctl_I2C_SET_BYTEADDRESS			11
#define ioctl_I2C_SET_BYTEADDRESSLENGTH		12


#endif /* SRC_FRTOS_IO_FRTOS_IO_H_ */
