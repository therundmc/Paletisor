
/*************************************************************************************************
 * 					ITS-PLC Startup Project
 *
 * 					(c) Polytech Montpellier - MEA
 *
 * 					20/09/2015 - Laurent Latorre
 *
 * 					FreeRTOS getting started project to work with ITS-PLC Simulator
 *
 * 					Creates 3 asynchronous tasks :
 *
 * 					- Graphical User Interface (TouchScreen and I/O extender interface)
 * 					- Task1 : Actuator A00 toggle
 * 					- Task2 : Actuator A01 copies the state of Sensor S00
 *
 *************************************************************************************************/


// Local defines
// Actions

#define A0 0x00
#define A1 0x01
#define A2 0x02
#define A3 0x03
#define A4 0x04
#define A5 0x05

	// Comment the following line if you want to remove the splash screen
	// This will speed-up the flash process as no bitmap will be loaded

#define DISPLAY_SPLASH


// Include main headers

#include "stm32f4xx.h"
#include "fonts.h"

// Include BSP headers

#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_sdram.h"
#include "stm32f429i_discovery_ioe.h"

// Include FreeRTOS headers

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"

// Declare Tasks

void vTaskElevateurPoussoir (void *pvParameters);
void vTaskGlissiere   (void *pvParameters);
void vTaskButoir   (void *pvParameters);
void vTaskPinces (void *pvParameters);
void vTaskAscenseur (void *pvParameters);
void vTaskTapisPalette (void *pvParameters);
void vTaskEntree (void *pvParameters);
void vTaskSortie (void *pvParameters);

// Trace Labels

traceLabel user_event_channel;


// Global variables

static	uint16_t	   its_sensors;			// 16 bits representing sensors states from ITS-PLC
static	uint16_t	   forced_sensors;		// 16 bits representing sensors states from User Interface
static	uint16_t	   sensors;				// Bitwise OR between the two above
static	uint16_t	   actuators;			// 16 bits representing actuators states

static  uint8_t		   dbg_mode;			// ONLINE/OFFLINE upon detection of the I/O Extender on boot


// Main program

int main(void)
{

    // Start IO Extenders (Touchpad & PCF IO Extenders)

    IOE_Config();

    // Test if ITS-PLC I/O Extender is connected

    if (I2C_WritePCFRegister(PCF_B_WRITE_ADDR, (int8_t)0x00))
    {
    	// I/O Extender found, ONLINE mode -> Green LED

    	dbg_mode = DBG_ONLINE;
    	STM_EVAL_LEDOn(LED3);
    	STM_EVAL_LEDOff(LED4);
    }
    else
    {
    	// I/O Extender not found, OFFLINE mode -> Red LED

    	dbg_mode = DBG_OFFLINE;
    	STM_EVAL_LEDOff(LED3);
    	STM_EVAL_LEDOn(LED4);
	}

    // Init sensors & actuators variables

    actuators = 0x0000;
    forced_sensors = 0x000;

	// Launch FreeRTOS Trace recording

	vTraceInitTraceData();
	uiTraceStart();

	// Create Tasks

	xTaskCreate(vTaskElevateurPoussoir,   "TaskElevateurPoussoir",   128, NULL, 1, NULL);		// 128 bytes stack, priority 1, Pas de paramètres
	xTaskCreate(vTaskGlissiere,   "TaskGlissiere",   128, NULL, 1, NULL);
	xTaskCreate(vTaskButoir, "TaskButoir", 128, NULL, 1, NULL);
	xTaskCreate(vTaskPinces,   "TaskPinces",   128, NULL, 1, NULL);
	xTaskCreate(vTaskAscenseur,   "TaskAscenseur",   128, NULL, 1, NULL);
	xTaskCreate(vTaskTapisPalette, "TaskTapisPalette", 128, NULL, 1, NULL);
	xTaskCreate(vTaskEntree,   "TaskEntree",   128, NULL, 1, NULL);
	xTaskCreate(vTaskSortie,   "TaskSortie",   128, NULL, 1, NULL);


	user_event_channel = xTraceOpenLabel("UEV");

	// Start the Scheduler

	vTaskStartScheduler();

	//Create Queue

	xQueueSortie = xQueueCreate( 10, sizeof( unsigned long ) );

    while(1)
    {
    	// The program should never be here...
    }
}

void vTaskGlissiere(void *pvParameters)
{

    static TP_STATE* 	TP_State;
    uint16_t			i,j,k,l;
    char				label[5];
    uint8_t				tp_release;

    TP_State = State_0;

    switch(TP_State)
    {
    case(State_0):
		ulTaskNotifyTake( pdTRUE,portMAX_DELAY);
    	TP_State=State_1;
    	break;
    case(State_1):
		xQueueSend( xQueueSortie,
		                       ( void * ) &ulVar,
		                       ( TickType_t ) 10 ) != pdPASS )





    }


}

/*
 * 	TASK 1
 *
 * 	- Actuator A00 blinking
 * 	- Green LED3 blinking
 *
 * 	This task is executed every 800ms
 *
 */

void vTask1 (void *pvParameters)
{
	static uint16_t	n;

	while(1)
	{
		actuators = actuators ^ 0x0001;

		vTracePrintF(user_event_channel, "Hello from task 1 : %d", n);

		n++;

		// Wait for 800ms
		vTaskDelay(800);
	}
}

/*
 * 	TASK 2
 *
 * 	- Actuator A01 copies the state of Sensor S00
 * 	- Red LED4 blinking
 *
 * 	This task is executed every 100ms
 *
 */

void vTask2 (void *pvParameters)
{
	while(1)
	{
    	if ((sensors & 0x0001) == 0x0001)
    	{
    		actuators = actuators | 0x0002;
    	}
    	else
    	{
    		actuators = actuators & 0xFFFD;
    	}

		// Wait for 100ms
		vTaskDelay(100);
	}
}

