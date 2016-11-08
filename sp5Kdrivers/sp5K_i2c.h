/*
 * sp5K_i2c.h
 *
 *  Created on: 18/10/2015
 *      Author: pablo
 */

#ifndef SRC_SP5KDRIVERS_SP5K_I2C_H_
#define SRC_SP5KDRIVERS_SP5K_I2C_H_

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/twi.h>

#include "avrlibdefs.h"
#include "avrlibtypes.h"
#include "global.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "FRTOS-IO.h"

void I2C_MasterInit(int bitrateKHz);
s08 I2C_masterWriteRead ( const u08  slaveI2CAddress, const u08 devAddressLength, const u16 byteAddress, char *pvBuffer, size_t bytesToWrite, size_t bytesToRead  );

#define	I2C_WRITE_BUFFER_SIZE ( 64 + 3 )
#define	I2C_READ_BUFFER_SIZE 64

/*! Transaction status defines. */
#define TWIM_STATUS_READY              0
#define TWIM_STATUS_BUSY               1


/*! Transaction result enumeration. */
typedef enum TWIM_RESULT_enum {
	TWIM_RESULT_UNKNOWN          = (0x00<<0),
	TWIM_RESULT_OK               = (0x01<<0),
	TWIM_RESULT_BUFFER_OVERFLOW  = (0x02<<0),
	TWIM_RESULT_ARBITRATION_LOST = (0x03<<0),
	TWIM_RESULT_BUS_ERROR        = (0x04<<0),
	TWIM_RESULT_NACK_RECEIVED    = (0x05<<0),
	TWIM_RESULT_FAIL             = (0x06<<0),
} TWIM_RESULT_t;

struct {
	u08 I2CwriteBuffer[I2C_WRITE_BUFFER_SIZE];
	u08 I2CreadBuffer[I2C_READ_BUFFER_SIZE];
	u08 I2CbytesToWrite;
	u08 I2CbytesToRead;
	u08 I2CbytesWritten;                       /*!< Number of bytes written */
	u08 I2CbytesRead;                          /*!< Number of bytes read */
	u08 I2Cstatus;                             /*!< Status of transaction */
	u08 I2Cresult;                             /*!< Result of transaction */
	u08 I2CslaveAddress;

} I2CdataStruct;

#endif /* SRC_SP5KDRIVERS_SP5K_I2C_H_ */
