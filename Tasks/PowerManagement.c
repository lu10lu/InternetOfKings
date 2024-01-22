/*
 * PowerManagement.c
 *
 * Created: 5/12/2023 15:19:27
 *  Author: lucas
 */ 

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "trace.h"

#include "PowerManagement.h"
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include "DriverPower.h"
// Define the pin for the external interrupt
#define INTERRUPT_PIN_bm PIN3_bm
void enableInterrupt(void){
		PORTB.DIRCLR=0b00001000;
		PORTB.PIN3CTRL=0b01011001; //Pull up, inverted
		PORTB.INT0MASK=0b00001000; //Interrupt on all cursor stick lines
		PORTB.INTCTRL=0b11;		   //Enable high level interrrupt
}

void enterSleepMode(void) {
	// Set the sleep mode. For XMEGA, you might use SLEEP_SMODE_PDOWN_gc for power down.
	// Check the specific group configuration (gc) value for your microcontroller.
	PMIC.CTRL = 0b111;

	set_sleep_mode(SLEEP_SMODE_PDOWN_gc);

	// Disable any peripherals here to save power.
	// For example, power_adc_disable(); (if such function exists or equivalent)

	// Enable global interrupts so that the MCU can wake up from an interrupt.
	sei();
	
	vTaskDelay(2);
	// Enable sleep.
	
	
	PMIC.CTRL &=0b11111100; // shut off medium and low lever interrupt
	DriverPowerVccAuxSet(0);
	vTaskDelay(2);
	sleep_mode();
	//sleep_enable();

	//vTaskDelay(75);
	// Enter sleep mode.
	//sleep_cpu();

	// Sleep mode will be exited upon receiving an interrupt.
	//vTaskDelay(50);
	// Disable sleep after waking up.
	//sleep_disable();
}

