/*
 * ee_sp5K.h
 *
 *  Created on: 26/10/2015
 *      Author: pablo
 */

#ifndef SRC_SP6KLIBS_L_EEPROM_H_
#define SRC_SP6KLIBS_L_EEPROM_H_


#include "FRTOS-IO.h"

//------------------------------------------------------------------------------------
// Identificacion en el bus I2C en el board sp6KX_LOGICA
#define EE_ADDR			0xA0
#define EE_PAGESIZE		64

bool EE_read( uint16_t eeAddress, char *data, uint8_t length );
bool EE_write( uint16_t eeAddress, char *data, uint8_t length );
bool EE_test_write(char *s0, char *s1);
bool EE_test_read(char *s0, char *s1, char *s2);

#endif /* SRC_SP6KLIBS_L_EEPROM_H_ */
