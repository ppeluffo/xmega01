/*
 * sp5K_uart.c
 *
 *  Created on: 4/10/2015
 *      Author: pablo
 *
 *  TRASMISION:
 *  El mecanismo elegido por eficiencia es tener una queue en la cual se escriben los
 *  caracteres a trasmitir y se habilita la interrupcion.
 *  Esta, saca los caracteres y los va enviando a la UART.
 *  Esto hace, que la rutina de interrupcion deba saber a priori cual es la cola de trasmision.
 *  Como es una ISR, no se le puede pasar como parametro.
 *  Por otro lado, no  importaria mucho tenerlo definido de antemano ya que se trata de un
 *  sistema embebido.
 */


#include "sp5K_uart.h"
#include "FRTOS-IO.h"

/*------------------------------------------------------------------------------------*/
void pvUARTInit( const int UARTx )
{

	// La inicializacion de las UART se hace como:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

	portENTER_CRITICAL();

	//
	switch ( UARTx ) {
	case pUART_USB:

		// Corresponde a PORTC

		PORTC.DIRSET   = PIN3_bm;	/* PC3 (TXD0) as output. */
		PORTC.DIRCLR   = PIN2_bm;	/* PC2 (RXD0) as input. */

		/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
		USARTC0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 32 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 */
		USARTC0.BAUDCTRLA = (uint8_t) 2094;
		USARTC0.BAUDCTRLB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);

	//	USARTC0.BAUDCTRLA = (uint8_t) 1659;
	//	USARTC0.BAUDCTRLB = ( -3 << USART_BSCALE0_bp)|(1659 >> 8);

		// Habilito la TX y RX
		USARTC0.CTRLB |= USART_RXEN_bm;
		USARTC0.CTRLB |= USART_TXEN_bm;

		// Habilito la interrupcion de Recepcion ( low level )
		// low level, RXint enabled
		USARTC0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
		USARTC0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
		//USARTC0.CTRLA = ( USARTC0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

		break;

	case pUART_BT:

			// Corresponde a PORTD

			PORTD.DIRSET = PIN3_bm;
			PORTD.DIRCLR = PIN2_bm;

			/* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
			USARTD0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;


			/* Set Baudrate to 115200 bps:
			 * Use the default I/O clock frequency that is 32 MHz.
			 * Los valores los extraigo de la planilla provista por Atmel
			 */
			USARTD0.BAUDCTRLA = (uint8_t) 2094;
			USARTD0.BAUDCTRLB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);

			// Habilito la TX y RX
			USARTD0.CTRLB |= USART_RXEN_bm;
			USARTD0.CTRLB |= USART_TXEN_bm;

			// Habilito la interrupcion de Recepcion ( low level )
			// low level, RXint enabled
			USARTD0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
			USARTD0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
			//USARTC0.CTRLA = ( USARTC0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

			break;

	case pUART_GPRS:

			// Corresponde a PORTE

			PORTE.DIRSET = PIN3_bm;
			PORTE.DIRCLR = PIN2_bm;

			/* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
			USARTE0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;


			/* Set Baudrate to 115200 bps:
			 * Use the default I/O clock frequency that is 32 MHz.
			 * Los valores los extraigo de la planilla provista por Atmel
			 */
			USARTE0.BAUDCTRLA = (uint8_t) 2094;
			USARTE0.BAUDCTRLB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);

			// Habilito la TX y RX
			USARTE0.CTRLB |= USART_RXEN_bm;
			USARTE0.CTRLB |= USART_TXEN_bm;

			// Habilito la interrupcion de Recepcion ( low level )
			// low level, RXint enabled
			USARTE0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
			USARTE0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0

			break;

	case pUART_XBEE:

			// Corresponde a PORTF

			PORTF.DIRSET = PIN3_bm;
			PORTF.DIRCLR = PIN2_bm;

			/* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
			USARTF0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;


			/* Set Baudrate to 115200 bps:
			 * Use the default I/O clock frequency that is 32 MHz.
			 * Los valores los extraigo de la planilla provista por Atmel
			 */
		//	USARTF0.BAUDCTRLA = (uint8_t) 2094;
		//	USARTF0.BAUDCTRLB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);

			USARTF0.BAUDCTRLA = (uint8_t) 1659;
			USARTF0.BAUDCTRLB = ( -3 << USART_BSCALE0_bp)|(1659 >> 8);

			// Habilito la TX y RX
			USARTF0.CTRLB |= USART_RXEN_bm;
			USARTF0.CTRLB |= USART_TXEN_bm;

			// Habilito la interrupcion de Recepcion ( low level )
			// low level, RXint enabled
			USARTF0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
			USARTF0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
			//USARTC0.CTRLA = ( USARTC0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

			break;
	}

	portEXIT_CRITICAL();

	return;
}
/*------------------------------------------------------------------------------------*/
void vUartInterruptOn(int UARTx)
{

	// Habilito la interrupcion TX del UART lo que hace que se ejecute la ISR_TX y
	// esta vaya a la TXqueue y si hay datos los comienze a trasmitir.

uint8_t tempCTRLA;

	switch(UARTx) {
	case pUART_USB:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTC0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTC0.CTRLA = tempCTRLA;
		break;

	case pUART_BT:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTD0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTD0.CTRLA = tempCTRLA;
		break;

	case pUART_GPRS:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTE0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTE0.CTRLA = tempCTRLA;
		break;

	case pUART_XBEE:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTF0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTF0.CTRLA = tempCTRLA;
		break;
	}


}
/*------------------------------------------------------------------------------------*/
void vUartInterruptOff(int UARTx)
{

uint8_t tempCTRLA;

	switch(UARTx) {
	case pUART_USB:
		// TXint disabled
		// Espero que no halla nada en el DREG
		tempCTRLA = USARTC0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTC0.CTRLA = tempCTRLA;
		break;
	case pUART_BT:
		// TXint disabled
		tempCTRLA = USARTD0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTD0.CTRLA = tempCTRLA;
		break;
	case pUART_GPRS:
		// TXint disabled
		tempCTRLA = USARTE0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTE0.CTRLA = tempCTRLA;
		break;
	case pUART_XBEE:
		// TXint disabled
		tempCTRLA = USARTF0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTF0.CTRLA = tempCTRLA;
		break;
	}

}
/*------------------------------------------------------------------------------------*/
// FIFO: FUNCIONES PRIVADAS DE MANEJO DE FIFOS.
// QUEUE: Son provistas por el FreeRTOS.
/*------------------------------------------------------------------------------------*/
fifoHandle_t xFifoCreate ( const u16 length,int flags  )
{
fifo_handle_s *pxNewFifo;
fifoHandle_t xReturn = NULL;
int8_t *pcAllocatedBuffer;
u08 *dataBuffer;

	// Aloco el espacio para el buffer de datos.
	dataBuffer = ( u08 * ) pvPortMalloc( length + 1);

	// Aloco espacio para la estructura.
	pcAllocatedBuffer = ( int8_t * ) pvPortMalloc( sizeof(fifo_handle_s ));
	if( pcAllocatedBuffer != NULL )
	{
		pxNewFifo = ( fifoHandle_t * ) pcAllocatedBuffer;
		pxNewFifo->head = 0;	// start
		pxNewFifo->tail = 0;	// end
		pxNewFifo->uxMessageWaiting = 0;
		pxNewFifo->buff = dataBuffer;
		pxNewFifo->length = length;		// REVISAR

		xReturn = pxNewFifo;
	}

	return(xReturn);

}
/*------------------------------------------------------------------------------------*/
int xFifoReset( fifoHandle_t xFifo )
{
fifo_handle_s *pxFifo;

	taskENTER_CRITICAL();
	pxFifo = xFifo;
	pxFifo->head = 0;
	pxFifo->tail = 0;
	pxFifo->uxMessageWaiting = 0;
	memset(pxFifo->buff,0, pxFifo->length );	// REVISAR
	taskEXIT_CRITICAL();
	return(0);
}
/*------------------------------------------------------------------------------------*/
BaseType_t xFifoSend( fifoHandle_t xFifo,const char *cChar, TickType_t xTicksToWait )
{
	// Si el buffer esta lleno, descarto el valor recibido
	// Guardo en el lugar apuntado por head y avanzo.

fifo_handle_s *pxFifo = xFifo;
s08 ret = errQUEUE_FULL;

	taskENTER_CRITICAL();
	// Si el buffer esta vacio ajusto los punteros
	if( pxFifo->uxMessageWaiting == 0) {
		pxFifo->head = pxFifo->tail = 0;
	}

	if ( pxFifo->uxMessageWaiting < pxFifo->length ) {
		pxFifo->buff[pxFifo->head] = *cChar;
		++pxFifo->uxMessageWaiting;
		// Avanzo en modo circular
		pxFifo->head = ( pxFifo->head  + 1 ) % ( pxFifo->length );
		ret = pdTRUE;
    }
	taskEXIT_CRITICAL();
	return(ret);

}
/*------------------------------------------------------------------------------------*/
BaseType_t xFifoSendFromISR( fifoHandle_t xFifo,const char *cChar, TickType_t xTicksToWait )
{
	// Si el buffer esta lleno, descarto el valor recibido
	// Guardo en el lugar apuntado por head y avanzo.

fifo_handle_s *pxFifo = xFifo;
s08 ret = errQUEUE_FULL;

	// Si el buffer esta vacio ajusto los punteros !!!
	if( pxFifo->uxMessageWaiting == 0) {
		pxFifo->head = pxFifo->tail = 0;
	}

	if ( pxFifo->uxMessageWaiting < pxFifo->length ) {
		pxFifo->buff[pxFifo->head] = *cChar;
		++pxFifo->uxMessageWaiting;
		// Avanzo en modo circular
		pxFifo->head = ( pxFifo->head  + 1 ) % ( pxFifo->length );
		ret = pdTRUE;
    }
	return(ret);

}
/*------------------------------------------------------------------------------------*/
BaseType_t xFifoReceive( fifoHandle_t xFifo, char *cChar, TickType_t xTicksToWait )
{

fifo_handle_s *pxFifo = xFifo;
s08 ret = pdFALSE;

	taskENTER_CRITICAL();
	//  Si el buffer esta vacio retorno.
	if( pxFifo->uxMessageWaiting == 0) {
		//pxFifo->head = pxFifo->tail = 0;
		taskEXIT_CRITICAL();
		return(ret);
	}

	*cChar = pxFifo->buff[pxFifo->tail];
	--pxFifo->uxMessageWaiting;
	// Avanzo en modo circular
	pxFifo->tail = ( pxFifo->tail  + 1 ) % ( pxFifo->length );
	ret = pdTRUE;
	taskEXIT_CRITICAL();

	return(ret);
}
/*------------------------------------------------------------------------------------*/
BaseType_t xFifoReceiveFromISR( fifoHandle_t xFifo, char *cChar, TickType_t xTicksToWait )
{

fifo_handle_s *pxFifo = xFifo;
s08 ret = pdFALSE;

	// Cannot block in an ISR

	//  Si el buffer esta vacio retorno.
	if( pxFifo->uxMessageWaiting == 0) {
		pxFifo->head = pxFifo->tail = 0;
		return(ret);
	}

	*cChar = pxFifo->buff[pxFifo->tail];
	--pxFifo->uxMessageWaiting;
	// Avanzo en modo circular
	pxFifo->tail = ( pxFifo->tail  + 1 ) % ( pxFifo->length );
	ret = pdTRUE;

	return(ret);
}
/*------------------------------------------------------------------------------------*/
// UART ISR:
/*------------------------------------------------------------------------------------*/
ISR(USARTC0_DRE_vect)
{
/* Handler (ISR) de TX0.
 * El handler maneja la trasmision de datos por la uart0.
 * Para trasmitir, usando la funcion xUart0PutChar se encolan los datos y prende la
 * flag de interrupcion.
 * La rutina de interrupcion ejecuta este handler (SIGNAL) en el cual si hay
 * datos en la cola los extrae y los trasmite.
 * Si la cola esta vacia (luego del ultimo) apaga la interrupcion.
*/

s08 cTaskWoken;
char cChar;
s08 res = pdFALSE;
UART_device_control_t *pUart;

	pUart = pdUART_USB.phDevice;

	if ( pUart->txBufferType == QUEUE ) {
		res = xQueueReceiveFromISR( pUart->txStruct, &cChar, &cTaskWoken );
	} else {
		res = xFifoReceiveFromISR( pUart->txStruct, &cChar, 0 );
	}

	if( res == pdTRUE ) {
		/* Send the next character queued for Tx. */
		USARTC0.DATA = cChar;
	} else {
		/* Queue empty, nothing to send. */
//		if ( ! ((USARTC0.STATUS & USART_DREIF_bm) != 0) )
			vUartInterruptOff(pUART_USB);
	}
}
/*------------------------------------------------------------------------------------*/
ISR(USARTC0_RXC_vect)
{
/* Handler (ISR) de RX0.
 *  Receive complete interrupt service routine.
 * Este handler se encarga de la recepcion de datos.
 * Cuando llega algun datos por el puerto serial lo recibe este handler y lo va
 * guardando en la cola de recepcion
*/

char cChar;
UART_device_control_t *pUart;

	pUart = pdUART_USB.phDevice;

	/* Get the character and post it on the queue of Rxed characters.
	 * If the post causes a task to wake force a context switch as the woken task
	 * may have a higher priority than the task we have interrupted.
    */
	cChar = USARTC0.DATA;

	if ( pUart->rxBufferType == QUEUE ) {
		if( xQueueSendFromISR( pUart->rxStruct, &cChar, 0 ) ) {
			taskYIELD();
		}
	} else {
		if( xFifoSendFromISR( pUart->rxStruct, &cChar, 0 ) ) {
			taskYIELD();
		}
	}
}
/*------------------------------------------------------------------------------------*/
ISR(USARTD0_DRE_vect)
{

s08 cTaskWoken;
char cChar;
s08 res = pdFALSE;
UART_device_control_t *pUart;

	pUart = pdUART_BT.phDevice;

	if ( pUart->txBufferType == QUEUE ) {
		res = xQueueReceiveFromISR( pUart->txStruct, &cChar, &cTaskWoken );
	} else {
		res = xFifoReceiveFromISR( pUart->txStruct, &cChar, 0 );
	}

	if( res == pdTRUE ) {
		/* Send the next character queued for Tx. */
		USARTD0.DATA = cChar;
	} else {
		/* Queue empty, nothing to send. */
		vUartInterruptOff(pUART_BT);
	}
}
/*------------------------------------------------------------------------------------*/
ISR(USARTD0_RXC_vect)
{

char cChar;
UART_device_control_t *pUart;

	pUart = pdUART_BT.phDevice;

	/* Get the character and post it on the queue of Rxed characters.
	 * If the post causes a task to wake force a context switch as the woken task
	 * may have a higher priority than the task we have interrupted.
    */
	cChar = USARTD0.DATA;

	if ( pUart->rxBufferType == QUEUE ) {
		if( xQueueSendFromISR( pUart->rxStruct, &cChar, 0 ) ) {
			taskYIELD();
		}
	} else {
		if( xFifoSendFromISR( pUart->rxStruct, &cChar, 0 ) ) {
			taskYIELD();
		}
	}
}
/*------------------------------------------------------------------------------------*/
ISR(USARTE0_DRE_vect)
{

s08 cTaskWoken;
char cChar;
s08 res = pdFALSE;
UART_device_control_t *pUart;

	pUart = pdUART_GPRS.phDevice;

	if ( pUart->txBufferType == QUEUE ) {
		res = xQueueReceiveFromISR( pUart->txStruct, &cChar, &cTaskWoken );
	} else {
		res = xFifoReceiveFromISR( pUart->txStruct, &cChar, 0 );
	}

	if( res == pdTRUE ) {
		/* Send the next character queued for Tx. */
		USARTE0.DATA = cChar;
	} else {
		/* Queue empty, nothing to send. */
		vUartInterruptOff(pUART_GPRS);
	}
}
/*------------------------------------------------------------------------------------*/
ISR(USARTE0_RXC_vect)
{

char cChar;
UART_device_control_t *pUart;

	pUart = pdUART_GPRS.phDevice;

	/* Get the character and post it on the queue of Rxed characters.
	 * If the post causes a task to wake force a context switch as the woken task
	 * may have a higher priority than the task we have interrupted.
    */
	cChar = USARTE0.DATA;

	if ( pUart->rxBufferType == QUEUE ) {
		if( xQueueSendFromISR( pUart->rxStruct, &cChar, 0 ) ) {
			taskYIELD();
		}
	} else {
		if( xFifoSendFromISR( pUart->rxStruct, &cChar, 0 ) ) {
			taskYIELD();
		}
	}
}
/*------------------------------------------------------------------------------------*/
ISR(USARTF0_DRE_vect)
{

s08 cTaskWoken;
char cChar;
s08 res = pdFALSE;
UART_device_control_t *pUart;

	pUart = pdUART_XBEE.phDevice;

	if ( pUart->txBufferType == QUEUE ) {
		res = xQueueReceiveFromISR( pUart->txStruct, &cChar, &cTaskWoken );
	} else {
		res = xFifoReceiveFromISR( pUart->txStruct, &cChar, 0 );
	}

	if( res == pdTRUE ) {
		/* Send the next character queued for Tx. */
		USARTF0.DATA = cChar;
	} else {
		/* Queue empty, nothing to send. */
		vUartInterruptOff(pUART_GPRS);
	}
}
/*------------------------------------------------------------------------------------*/
ISR(USARTF0_RXC_vect)
{

char cChar;
UART_device_control_t *pUart;

	pUart = pdUART_XBEE.phDevice;

	/* Get the character and post it on the queue of Rxed characters.
	 * If the post causes a task to wake force a context switch as the woken task
	 * may have a higher priority than the task we have interrupted.
    */
	cChar = USARTF0.DATA;

	if ( pUart->rxBufferType == QUEUE ) {
		if( xQueueSendFromISR( pUart->rxStruct, &cChar, 0 ) ) {
			taskYIELD();
		}
	} else {
		if( xFifoSendFromISR( pUart->rxStruct, &cChar, 0 ) ) {
			taskYIELD();
		}
	}
}
/*-
size_t uxFifoSpacesAvailable( fifoHandle_t xFifo )
{

fifo_handle_s *pxFifo = xFifo;

	return( pxFifo->length - pxFifo->uxMessageWaiting);
}
/*------------------------------------------------------------------------------------*/
size_t uxFifoMessagesWaiting( fifoHandle_t xFifo )
{

fifo_handle_s *pxFifo = xFifo;

	return(pxFifo->uxMessageWaiting);
}
/*------------------------------------------------------------------------------------*/
