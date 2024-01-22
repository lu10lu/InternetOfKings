#include "StartupTask.h"

#include "FreeRTOS.h"
#include "task.h"


#include "DriverUSART.h"
#include "DriverCursorStick.h"
#include "DriverPower.h"
#include "DriverTwiMaster.h"
#include "Driverpl9823.h"
#include "DriverAdc.h"
#include "DriverLed.h"
#include "DriverMPU6050.h"
#include "DriverDbgUSART.h"
#include "DriverAdps9960.h"
#include "DriverOled.h"
#include "DriverVL53L0X.h"

#include "OledMenuTask.h"
#include "MotorPosTask.h"
#include "MotorSpeedTask.h"
#include "ADCTask.h"
#include "LineFollowerSpeedTask.h"
#include "LineFollowerDirectTask.h"
#include "RGBTask.h"
#include "GyroTask.h"
#include "TerminalTask.h"
#include "MotionTask.h"
#include "PowerManagement.h"

#include <stdio.h>


//Private function prototypes
static void WorkerStartup(void *pvParameters);

//Function definitions
void InitStartupTask()
{
	xTaskCreate( WorkerStartup, "startup", 256, NULL, tskIDLE_PRIORITY+3, NULL );	
}

static void WorkerStartup(void *pvParameters)
{
	int res;
	//Voor oled helemaal af te zetten kijk naar i2c lijn
	DriverPowerVccAuxSet(1);//Enable Auxillary power line
	DriverCursorstickInit();//Initialize cursor stick
	//DriverLedInit();		//Initialize LED's
	DriverUSARTInit();		//USART init and link to stdio ---ON--
	DriverPowerInit();		//Initialize aux power driver ---ON--
	DriverTWIMInit();		//Initialize TWI in master mode
	DriverPL9823Init();		//Initialize PL9823 LEDs
	DriverAdcInit();		//Initialize ADC driver
	
	DriverOLEDInit(2);		//Initialize OLED display
	//DriverAdps9960Init();	//Initialize color sensor	
	DriverVL53L0XInit();	//Initialize rangefinder
	DriverOLEDSleep();
	vTaskDelay(50);
	
	//Enable test output (T21)
	PORTA.DIRSET=1<<5;	
	//Initialize application tasks			
	
	//InitOLEDMenuTask();
	//InitADCTask();
	InitMotorPosTask(); //---ON--
	InitMotorSpeedTask(); //---ON--
	//InitLineFollowerSpeedTask();
	//InitLineFollowerDirectTask();
	InitRGBTask();
	//InitGyroTask();
	InitTerminalTask(); //---ON--
	InitMotionTask(); //---ON--
	
	vTaskSuspend(NULL);

}
