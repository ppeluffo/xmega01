/*
 * sp6K_spi.c
 *
 *  Created on: 28 de may. de 2017
 *      Author: pablo
 */

#include "sp6K_spi.h"
#include "../xmega01.h"

//------------------------------------------------------------------------------------
void SPI_init(void)
{
	// Inicializa la interface SPI implementada en el puerto C.
bool clk2x = false;
bool lsbFirst = false;
SPI_MODE_t mode = SPI_MODE_0_gc;
SPI_PRESCALER_t clockDivision = SPI_PRESCALER_DIV4_gc;

	SPIC.CTRL   = clockDivision |                  			/* SPI prescaler. */
	                      (clk2x ? SPI_CLK2X_bm : 0) |     /* SPI Clock double. */
	                      SPI_ENABLE_bm |                  /* Enable SPI module. */
	                      (lsbFirst ? SPI_DORD_bm  : 0) |  /* Data order. */
	                      SPI_MASTER_bm |                  /* SPI master. */
	                      mode;                            /* SPI mode. */


	/* MOSI and SCK as output. */
	PORTC.DIRSET  = SPI_MOSI_bm | SPI_SCK_bm;

	/* Init SS pin as output with wired AND and pull-up. */
	PORTC.DIRSET = PIN4_bm;
	PORTC.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;

	/* Set SS output to high. (No slave addressed). */
	SPIFLASH_ASSERT_CS;

}
//------------------------------------------------------------------------------------
uint8_t SPI_tranceive(uint8_t data)

{
	// Trasmite y recibe desde el bus SPI.
	// Si solo estoy escribiendo, descarto el valor de retorno.
	// Si estoy leyendo, data es solo para activar el clock.
	//
	// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// NO muevo el CS. Esto lo hace la funcion de ordern superior ya que depende
	// de c/dispositivo !!!!!!
	// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

uint8_t rxData;

	// Trasmito ( real data o dummy for clock )
	SPIC.DATA = data;

	// Espero
	while(!(SPIC.STATUS & SPI_IF_bm));

	// Leo
	rxData = SPIC.DATA;

	return(rxData);

}
//------------------------------------------------------------------------------------
