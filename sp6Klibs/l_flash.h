/*
 * l_flash.h
 *
 *  Created on: 1 de jun. de 2017
 *      Author: pablo
 */

#ifndef SRC_SP6KLIBS_L_FLASH_H_
#define SRC_SP6KLIBS_L_FLASH_H_

#include "FRTOS-IO.h"

// device commands
#define SPIFLASH_CMD_READ		0x03	// read
#define SPIFLASH_CMD_WRITE		0x02	// page program ??
#define SPIFLASH_CMD_WREN		0x06	// write enable
#define SPIFLASH_CMD_WRDI		0x04	// write disable
#define SPIFLASH_CMD_RDSR		0x05	// read status register
#define SPIFLASH_CMD_WRSR		0x01	// write status register

#define SPIFLASH_CMD_CHIPERASE	0xC7	// chip erase

#define SPIFLASH_CMD_RELEASEDEEPWR	0xAB	// Release deep power-down
#define SPIFLASH_CMD_ENTERDEEPWR	0xB9	// Enter deep power-down

// status register bits
#define SPIFLASH_STATUS_BUSY	0x01	// busy, write in progress
#define SPIFLASH_STATUS_WEN		0x02	// write enable
#define SPIFLASH_STATUS_BP0		0x04	// block protect 0
#define SPIFLASH_STATUS_BP1		0x08	// block protect 1
#define SPIFLASH_STATUS_BP2		0x10	// block protect 2
#define SPIFLASH_STATUS_WPEN	0x80	// write protect enabled

// device constants
#define SPIFLASH_PAGESIZE		256		// 256 bytes/page
//------------------------------------------------------------------------------------

uint8_t FLASH_read_status(void);
void FLASH_write_enable(void);
void FLASH_write_disable(void);
void FLASH_chip_erase(void);
void FLASH_deep_power_down(void);
void FLASH_release_power_down(void);

bool FLASH_write ( const uint32_t address, char *pvBuffer, size_t xBytes );
bool FLASH_read  ( const uint32_t address, char *pvBuffer, size_t xBytes );

#endif /* SRC_SP6KLIBS_L_FLASH_H_ */
