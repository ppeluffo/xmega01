/*
 * sp6K_rtc.h
 *
 *  Created on: 24 de may. de 2017
 *      Author: pablo
 */

#ifndef SRC_DRIVERS_SP6KBOARD_SP6K_RTC_H_
#define SRC_DRIVERS_SP6KBOARD_SP6K_RTC_H_

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <time.h>
#include <inttypes.h>

#include "FreeRTOS.h"
#include "FRTOS-IO.h"

#define RTC_CYCLES_1S     1024

void RTC_test(void);
void RTC_init(void);
void RTC_write_time(struct tm *date );
void RTC_read_time (struct tm *date );


#endif /* SRC_DRIVERS_SP6KBOARD_SP6K_RTC_H_ */
