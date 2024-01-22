#include "DriverCursorstick.h"
#include "TerminalTask.h"
#include "DriverUSART.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <util/delay.h> // Voor _delay_ms functie

static QueueHandle_t CursorstickQueue;

void DriverCursorstickInit(void)
{ 
	PORTB.DIRCLR=0b11111000;
	PORTB.PIN3CTRL=0b01011001; //Pull up, inverted
	PORTB.PIN4CTRL=0b01011001; //Pull up, inverted	
	PORTB.PIN5CTRL=0b01011001; //Pull up, inverted
	PORTB.PIN6CTRL=0b01011001; //Pull up, inverted
	PORTB.PIN7CTRL=0b01011001; //Pull up, inverted
	PORTB.INT0MASK=0b11111000; //Interrupt on all cursor stick lines
	PORTB.INTCTRL=0b11;		   //High level inter
	
	CursorstickQueue=xQueueCreate(CURSOR_FIFO_LENGTH,1);

}

uint8_t DriverCursorstickGet(void)
{
	uint8_t ret=0;
	if (PORTB.IN & (1<<3)) ret|=(1<<4);
	if (PORTB.IN & (1<<4)) ret|=(1<<3);
	if (PORTB.IN & (1<<5)) ret|=(1<<2);
	if (PORTB.IN & (1<<6)) ret|=(1<<1);
	if (PORTB.IN & (1<<7)) ret|=(1<<0);

	return ret;
}

uint8_t DriverCursorStickGetFifo(TickType_t BlockTime)
{
	uint8_t ButtonState;
	BaseType_t res;
	res=xQueueReceive(CursorstickQueue,&ButtonState,BlockTime);
	if (res==pdTRUE) return ButtonState;
	else return 0;

}

ISR (PORTB_INT0_vect)
{
	sleep_disable();
	PMIC.CTRL |= 0b111;
	static uint32_t LastIntTime=0;
	uint32_t CurTime;
	uint8_t ButtonState;
	BaseType_t xHigherPriorityTaskWoken=pdFALSE;
	
	CurTime=portGET_RUN_TIME_COUNTER_VALUE();
	ButtonState=DriverCursorstickGet();
	
	if ((CurTime-LastIntTime)>CURSOR_MIN_INTERVAL) //debounce
	if (ButtonState>0)
	{
		xQueueSendToBackFromISR(CursorstickQueue,&ButtonState,&xHigherPriorityTaskWoken);
		LastIntTime=CurTime;
	}

	DriverPowerVccAuxSet(1);

	//DriverUSARTInit();
	printf("wakeup\r\n");
	//InitTerminalTask();
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

