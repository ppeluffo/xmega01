/*
 * sp6K_spi.h
 *
 *  Created on: 28 de may. de 2017
 *      Author: pablo
 */

#ifndef SRC_DRIVERS_SP6KBOARD_SP6K_SPI_H_
#define SRC_DRIVERS_SP6KBOARD_SP6K_SPI_H_

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/twi.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "../../sp6Klibs/l_uarts.h"
#include "FRTOS-IO.h"

#define SPI_SS_bm             0x10 /*!< \brief Bit mask for the SS pin. */
#define SPI_MOSI_bm           0x20 /*!< \brief Bit mask for the MOSI pin. */
#define SPI_MISO_bm           0x40 /*!< \brief Bit mask for the MISO pin. */
#define SPI_SCK_bm            0x80 /*!< \brief Bit mask for the SCK pin. */


#define SPIFLASH_ASSERT_CS ( PORTC.OUTCLR |= PIN4_bm )
#define SPIFLASH_RELEASE_CS ( PORTC.OUTSET |= PIN4_bm )


void SPI_init(void);
uint8_t SPI_tranceive(uint8_t data);

#define DEBUG_SPI

#endif /* SRC_DRIVERS_SP6KBOARD_SP6K_SPI_H_ */
