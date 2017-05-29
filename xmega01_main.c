/*
 * main.c
 *
 *  Created on: 18 de oct. de 2016
 *      Author: pablo
 *
 *  avrdude -v -Pusb -c avrispmkII -p x256a3 -F -e -U flash:w:xmega01.hex
 *  avrdude -v -Pusb -c avrispmkII -p x256a3 -F -e
 *
 *  1- En FRTOS_Write_UART cambio el taskYIELD por taskDelay porque sino se cuelga.
 *     Este delay hacia que los mensajes del cmdmode fuesen lentos y entonces cambio en cmdline.c
 *     la forma de mostrarlos usando directamente FRTOS-IO.
 *
 *  PENDIENTE:
 *  Hacer andar el watchdog
 *  Cambiar la velocidad y reconffigurar el BT
 *
 * 2017-05-16:
 * Configuro el RTC.
 * Rutinas de calendario.
 *
 */


#include "xmega01.h"

static void configure_systemMainClock(void);

int a;

char cmd_printfBuff[64];

int main( void )
{


	initMCU();

	// Clock principal del sistema
	configure_systemMainClock();
	//RTC_init();

	FreeRTOS_open(pUART_USB, ( UART_RXFIFO + UART_TXQUEUE ));
	FreeRTOS_open(pUART_BT, ( UART_RXFIFO + UART_TXQUEUE ));
	FreeRTOS_open(pUART_GPRS, ( UART_RXFIFO + UART_TXQUEUE ));
	FreeRTOS_open(pUART_XBEE, ( UART_RXFIFO + UART_TXQUEUE ));
	FreeRTOS_open(pI2C, 0 );
	FreeRTOS_open(pRTC, 0 );
	FreeRTOS_open(pSPI, 0 );

	xTaskCreate(tk1, "TK1", tk1_STACK_SIZE, NULL, tk1_TASK_PRIORITY,  &xHandle_tk1 );
	xTaskCreate(tk2, "TK2", tk1_STACK_SIZE, NULL, tk1_TASK_PRIORITY,  &xHandle_tk2 );
//	xTaskCreate(tk3, "CMD3", tk1_STACK_SIZE, NULL, tk1_TASK_PRIORITY,  &xHandle_tk3 );
	xTaskCreate(tkCmd, "CMD", tk1_STACK_SIZE, NULL, tk1_TASK_PRIORITY,  &xHandle_tkCmd );

	xTaskCreate(tkControl, "CTL", tk1_STACK_SIZE, NULL, tk1_TASK_PRIORITY,  &xHandle_tkControl );

	/* Arranco el RTOS. */
	vTaskStartScheduler();

	// En caso de panico, aqui terminamos.
	exit (1);

}
//-----------------------------------------------------------
void vApplicationIdleHook( void )
{

	for(;;) {

	}

}
//-----------------------------------------------------------
static void configure_systemMainClock(void)
{
/*	Configura el clock principal del sistema
	Inicialmente se arranca en 2Mhz.
	La configuracion del reloj tiene 2 componentes: el clock y el oscilador.
	Primero configuramos el oscilador de 32Mhz. Cuando este estable podemos
	configurar el clock del sistema para que use dicho oscilador.
	Opcionalmente podriamos deshabilitar el oscilador de 2Mhz para ahorrar energia.
*/

	// Habilito el oscilador de 32Mhz
	OSC.CTRL |= OSC_RC32MEN_bm;
	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC32MRDY_bm) == 0 );
	// Seteo el clock para que use el oscilador de 32Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
	//

}
//-----------------------------------------------------------
void tk1(void * pvParameters)
{

	// Tarea que genera una onda cuadrada en el pin PA.2
	// Se usa como realimentacion que el sistema esta andando.

( void ) pvParameters;
char LED1;

	PORTA.DIR |= _BV(2);	// 	salida

	for( ;; )
	{

		LED1 = PORTA.IN;
		LED1 ^= _BV(2);
		PORTA.OUT = LED1;
		vTaskDelay( ( TickType_t)( 50 / portTICK_RATE_MS ) );

		LED1 = PORTA.IN;
		LED1 ^= _BV(2);
		PORTA.OUT = LED1;
		vTaskDelay( ( TickType_t)( 950 / portTICK_RATE_MS ) );
	}

}
/*------------------------------------------------------------------------------------*/
void tk2(void * pvParameters)
{

	// Toggle pin PA.2

( void ) pvParameters;
char LED1;

	PORTA.DIR |= _BV(3);	// 	salida

	for( ;; )
	{
		LED1 = PORTA.IN;
		LED1 ^= _BV(3);
		PORTA.OUT = LED1;
		vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

		LED1 = PORTA.IN;
		LED1 ^= _BV(3);
		PORTA.OUT = LED1;
		vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	}

}
/*------------------------------------------------------------------------------------*/
void tk3_1(void * pvParameters)
{

	// Toggle pin PA.2

( void ) pvParameters;

char cChar;
char c;

	for( ;; )
	{
		for ( cChar = 'A'; cChar < 'Z'; cChar++) {
			memset(cmd_printfBuff,'\0', sizeof(cmd_printfBuff));
			snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Tarea de trasmision de datos seriales tk3 (%c)..\r\n\0"),cChar);
			//snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("101010100"));
			FreeRTOS_write( &pdUART_USB, cmd_printfBuff, sizeof(cmd_printfBuff) );

		//	FreeRTOS_write( &pdUART_BT, cmd_printfBuff, sizeof(cmd_printfBuff) );
		//	USARTD0.DATA = cChar;
			//vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		}
	}

}
/*------------------------------------------------------------------------------------*/
void tk3(void * pvParameters)
{

( void ) pvParameters;

char cChar;
char c;
int i = 0;


	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
/*
	// Loopback
	while (1 ) {

		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 50ms. lo que genera la espera.
		while ( FreeRTOS_read( &pdUART_USB, &c, sizeof(char) ) == 1 ) {
			FreeRTOS_write( &pdUART_USB, &c, sizeof(char) );
			if ( c ==  '\r' ) {
			//	vTaskDelay( ( TickType_t)( 10 ) );
				c = '\n';
				FreeRTOS_write( &pdUART_USB, &c, sizeof(char) );
			}
		}
	}

*/
//	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("AT+BAUD8"));
//	FreeRTOS_write( &pdUART_BT, cmd_printfBuff, sizeof(cmd_printfBuff) );
//	while(1)
//		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );


	// Loop de WRITE.
	for( ;; )
	{
		for ( cChar = 'A'; cChar < 'Z'; cChar++) {
			i++;
			memset(cmd_printfBuff,'\0', sizeof(cmd_printfBuff));
		//	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("tk3 (%c)(%d)..\r\n\0"),cChar,i);
			snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Tarea de trasmision de datos seriales tk3 (%c)(%d)..\r\n\0"),cChar,i);

			FreeRTOS_write( &pdUART_XBEE, cmd_printfBuff, sizeof(cmd_printfBuff) );
			vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
		//	taskYIELD();
		}
	}

}
/*------------------------------------------------------------------------------------*/
