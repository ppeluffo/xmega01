/*
 * xmega01_tkControl.c
 *
 *  Created on: 17 de nov. de 2016
 *      Author: pablo
 */

#include "xmega01.h"

/*------------------------------------------------------------------------------------*/
void tkControl(void * pvParameters)
{

( void ) pvParameters;

	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	// loop
	for( ;; )
	{

		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	}

}
/*------------------------------------------------------------------------------------*/
