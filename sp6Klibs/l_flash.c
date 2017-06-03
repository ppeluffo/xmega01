/*
 * l_flash.c
 *
 *  Created on: 1 de jun. de 2017
 *      Author: pablo
 *
 *  Implemento los comandos de acceso a la memoria flash SPI 25AA1024.
 *
 */

#include "../sp6Klibs/l_flash.h"
#include "../xmega01.h"

//------------------------------------------------------------------------------------
bool FLASH_write ( const uint32_t address, char *pvBuffer, size_t xBytes )
{


bool retV = false;
uint8_t i;

#ifdef DEBUG_SPI
	snprintf_P( debug_printfBuff,CHAR128,PSTR("SPI_FLASH_WR: 0x%02x,0x%02x\r\n\0"),address, xBytes );
	CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );
#endif

	// No debo cruzar fronteras de paginas ( no lo controlo )
	if ( xBytes > SPIFLASH_PAGESIZE) {
		return(false);
	}

	// enable write
	FLASH_write_enable();

	// clock out dummy byte to waste time
	SPI_tranceive(0xFF);

	// begin write
	SPIFLASH_ASSERT_CS;

	// issue write command
	SPI_tranceive(SPIFLASH_CMD_WRITE);

	// send address
	SPI_tranceive(address>>24);
	SPI_tranceive(address>>16);
	SPI_tranceive(address>>8);
	SPI_tranceive(address>>0);

	// transfer data
	for(i=0; i<xBytes; i++) {
		SPI_tranceive(*pvBuffer++);
	}

	// end write
	SPIFLASH_RELEASE_CS;

	// clock out dummy byte to waste time
	SPI_tranceive(0xFF);

	// wait until write is done
	SPIFLASH_ASSERT_CS;
	SPI_tranceive(SPIFLASH_CMD_RDSR);			// Envio el comando
	while( SPI_tranceive(0xFF) & SPIFLASH_STATUS_BUSY) {
		vTaskDelay( ( TickType_t)( 1 ));
	}

	SPIFLASH_RELEASE_CS;

	// clock out dummy byte to waste time
	SPI_tranceive(0xFF);

	return(retV);
}
//------------------------------------------------------------------------------------
bool FLASH_read  ( const uint32_t address, char *pvBuffer, size_t xBytes )
{

		// No debo leer fuera de los limites de la pagina (no lo controlo )

bool retV = false;

#ifdef DEBUG_I2C
	snprintf_P( debug_printfBuff,CHAR128,PSTR("SPI_FLASH_RD: 0x%02x,0x%02x\r\n\0"),address,xBytes );
	CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );
#endif

	if ( xBytes > SPIFLASH_PAGESIZE) {
		return(false);
	}

	// begin read
	SPIFLASH_ASSERT_CS;

	// issue read command
	SPI_tranceive(SPIFLASH_CMD_READ);

	// send address
	SPI_tranceive(address>>24);
	SPI_tranceive(address>>16);
	SPI_tranceive(address>>8);
	SPI_tranceive(address>>0);

	// read data
	while(xBytes--)
		*pvBuffer++ = SPI_tranceive(0xFF);
	// end read

	SPIFLASH_RELEASE_CS;

	return(retV);
}
//------------------------------------------------------------------------------------
uint8_t FLASH_read_status(void)
{

uint8_t status;

	SPIFLASH_ASSERT_CS;
	SPI_tranceive(SPIFLASH_CMD_RDSR);			// Envio el comando
	status = SPI_tranceive(0xFF);				// Leo la respuesta
	SPIFLASH_RELEASE_CS;
	return(status);
}
//------------------------------------------------------------------------------------
void FLASH_write_enable(void)
{
	// Habilito el latch de escritura

	SPIFLASH_ASSERT_CS;
	SPI_tranceive(SPIFLASH_CMD_WREN);			// Envio el comando
	SPIFLASH_RELEASE_CS;

}
//------------------------------------------------------------------------------------
void FLASH_write_disable(void)
{
	// Deshabilito el latch de escritura

	SPIFLASH_ASSERT_CS;
	SPI_tranceive(SPIFLASH_CMD_WRDI);			// Envio el comando
	SPIFLASH_RELEASE_CS;

}
//------------------------------------------------------------------------------------
void FLASH_chip_erase(void)
{
	FLASH_write_enable();

	// clock out dummy byte to waste time
	SPI_tranceive(0xFF);

	// chip erase
	SPIFLASH_ASSERT_CS;
	SPI_tranceive(SPIFLASH_CMD_CHIPERASE);
	SPIFLASH_RELEASE_CS;

	// clock out dummy byte to waste time
	SPI_tranceive(0xFF);

	// wait until write is done
	SPIFLASH_ASSERT_CS;
	SPI_tranceive(SPIFLASH_CMD_RDSR);			// Envio el comando
	while( SPI_tranceive(0xFF) & SPIFLASH_STATUS_BUSY) {
		vTaskDelay( ( TickType_t)( 1 ));
	}

	SPIFLASH_RELEASE_CS;
}
//------------------------------------------------------------------------------------
void FLASH_deep_power_down(void)
{
	// Pone a la memoria en estado de latencia.
	// Solo puede ser sacada con el comando Release_power_down.

	SPIFLASH_ASSERT_CS;
	SPI_tranceive(SPIFLASH_CMD_ENTERDEEPWR);			// Envio el comando
	SPIFLASH_RELEASE_CS;

}
//------------------------------------------------------------------------------------
void FLASH_release_power_down(void)
{
	// Pone a la memoria en estado activo

	SPIFLASH_ASSERT_CS;
	SPI_tranceive(SPIFLASH_CMD_RELEASEDEEPWR);			// Envio el comando
	SPIFLASH_RELEASE_CS;

	// Requiere un tiempo despertarse
	vTaskDelay( ( TickType_t)( 10 ));

}
//------------------------------------------------------------------------------------
