/**
  ******************************************************************************
  * @file    STM32F4-Discovery FreeRTOS demo\main.c
  * @author  T.O.M.A.S. Team
  * @version V1.1.0
  * @date    14-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "hw_config.h"
#include "main.h"

/** @addtogroup STM32F4-Discovery_Demo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DELAY 125     /* msec */
#define queueSIZE	6

/* angle */
#define G 2
/* Private macro -------------------------------------------------------------*/
/*********************************************************************************************************/


#define PWM_MOTOR_MIN 100
#define PWM_MOTOR_MAX 1000
#define TEST 200

#define PWM_Motor1 TIM4->CCR1   // 無刷 PWM
#define PWM_Motor2 TIM4->CCR2   // 無刷 PWM
#define PWM_Motor3 TIM4->CCR3   // 無刷 PWM
#define PWM_Motor4 TIM4->CCR4   // 無刷 PWM

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/



/* Private functions ---------------------------------------------------------*/

void pwm(void)
{
  volatile int i;
  int n = 1;
  uint16_t brightness = 0;      
  uint16_t who_run = 1;

 //Delay_1ms(50);

  Motor_Control(PWM_MOTOR_MAX, PWM_MOTOR_MAX, PWM_MOTOR_MAX, PWM_MOTOR_MAX);

 Delay_1ms(100);

  Motor_Control(PWM_MOTOR_MIN, PWM_MOTOR_MIN, PWM_MOTOR_MIN, PWM_MOTOR_MIN);
  
 Delay_1ms(100);

  while(1)  // Do not exit
  {
   Motor_Control(TEST, TEST, TEST, TEST);
   
   Delay_1ms(100);

   Motor_Control(TEST+50, TEST+50, TEST+50, TEST+50);

   Delay_1ms(100);



    //Light LEDs in turn
    // switch(who_run){
    //     case 0:
    //         TIM4->CCR1 = brightness - 1; // set brightness
    //         break;
    //     case 1:
    //         TIM4->CCR2 = brightness - 1; // set brightness
    //         break;
    //     case 2:
    //         TIM4->CCR3 = brightness - 1; // set brightness
    //         break;
    //     case 3:
    //         TIM4->CCR4 = brightness - 1; // set brightness
    //         break;
    // }

  }
 
  return(0); // System will implode
} 


void Delay_1ms( vu32 nCnt_1ms )
{
    u32 nCnt;
	  for(; nCnt_1ms != 0; nCnt_1ms--)
		    for(nCnt = 56580; nCnt != 0; nCnt--);
}
  
void Motor_Control(u16 Motor1, u16 Motor2, u16 Motor3, u16 Motor4)
{
	if(Motor1>PWM_MOTOR_MAX)      Motor1 = PWM_MOTOR_MAX;
	else if(Motor1<PWM_MOTOR_MIN) Motor1 = PWM_MOTOR_MIN;
		
	if(Motor2>PWM_MOTOR_MAX)      Motor2 = PWM_MOTOR_MAX;
	else if(Motor2<PWM_MOTOR_MIN) Motor2 = PWM_MOTOR_MIN;
				
	if(Motor3>PWM_MOTOR_MAX)      Motor3 = PWM_MOTOR_MAX;
	else if(Motor3<PWM_MOTOR_MIN) Motor3 = PWM_MOTOR_MIN;
						
	if(Motor4>PWM_MOTOR_MAX)      Motor4 = PWM_MOTOR_MAX;
	else if(Motor4<PWM_MOTOR_MIN) Motor4 = PWM_MOTOR_MIN;
								
	PWM_Motor1 = Motor1;
	PWM_Motor2 = Motor2;
	PWM_Motor3 = Motor3;
	PWM_Motor4 = Motor4;
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
   RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOD , ENABLE );
   RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );
}

/**
  * @brief  configure the PD12~15 to Timers
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure); // Reset init structure
 
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
      

    // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15; 
	//PD12->LED3 PD13->LED4 PD14->LED5 PD15->LED6
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init( GPIOD, &GPIO_InitStructure );  
}

/**
  * @brief  configure the TIM4 for PWM mode
  * @param  None
  * @retval None
  */
void TIM_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    // Let PWM frequency equal 100Hz.
    // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
    // Solving for prescaler gives 240.
    TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_TimeBaseInitStruct.TIM_Period = 3360 - 1;   
    TIM_TimeBaseInitStruct.TIM_Prescaler = 500 - 1; 
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;    
    TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStruct );
    
    TIM_OCStructInit( &TIM_OCInitStruct );
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    
    // Initial duty cycle equals 0%. Value can range from zero to 65535.
    //TIM_Pulse = TIM4_CCR1 register (16 bits)
    TIM_OCInitStruct.TIM_Pulse = 0; //(0=Always Off, 65535=Always On)
 
    TIM_OC1Init( TIM4, &TIM_OCInitStruct ); // Channel 1  LED
    TIM_OC2Init( TIM4, &TIM_OCInitStruct ); // Channel 2  LED
    TIM_OC3Init( TIM4, &TIM_OCInitStruct ); // Channel 3  LED
    TIM_OC4Init( TIM4, &TIM_OCInitStruct ); // Channel 4  LED
 
    TIM_Cmd( TIM4, ENABLE );
}
/*********************************************************************************************************/
/* Private functions ---------------------------------------------------------*/

/* Task functions declarations */
static void vLEDTask( void *pvParameters );
static void vSWITCHTask( void *pvParameters );
static void vMEMSTask(void *pvParameters);
static void vBALANCETask(void *pvParameters);

/* handlers to tasks to better control them */
xTaskHandle xLED_Tasks[4];
xTaskHandle xMEMS_Task, xBALANCE_Task;

/* variables used by tasks */
volatile int32_t ITM_RxBuffer;
/* initial arguments for vLEDTask task (which LED and what is the delay) */
static const int LEDS[4][2] = {{LED3,DELAY*1},
							   {LED4,DELAY*2},
							   {LED5,DELAY*3},
							   {LED6,DELAY*4}};

/* semaphores, queues declarations */
xSemaphoreHandle xSemaphoreSW  = NULL;
xQueueHandle xQueue;

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{ 
	/* create a pipe for MEMS->TIM4 data exchange */
	xQueue=xQueueCreate(1,queueSIZE*sizeof(uint8_t));

	/* create semaphores... */
	vSemaphoreCreateBinary( xSemaphoreSW );

	/* ...and clean them up */
	if(xSemaphoreTake(xSemaphoreSW, ( portTickType ) 0) == pdTRUE);

	/* initialize hardware... */
	prvSetupHardware();

	/* Start the tasks defined within this file/specific to this demo. */
	//xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED3", configMINIMAL_STACK_SIZE, (void *)LEDS[0],tskIDLE_PRIORITY, &xLED_Tasks[0] );
	//xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED4", configMINIMAL_STACK_SIZE, (void *)LEDS[1],tskIDLE_PRIORITY, &xLED_Tasks[1] );
	//xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED5", configMINIMAL_STACK_SIZE, (void *)LEDS[2],tskIDLE_PRIORITY, &xLED_Tasks[2] );
	//xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED6", configMINIMAL_STACK_SIZE, (void *)LEDS[3],tskIDLE_PRIORITY, &xLED_Tasks[3] );
	//xTaskCreate( vSWITCHTask, ( signed portCHAR * ) "SWITCH", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, NULL );
	xTaskCreate( vMEMSTask, ( signed portCHAR * ) "MEMS", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, &xMEMS_Task );
	//xTaskCreate( vBALANCETask, ( signed portCHAR * ) "BALANCE", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, &xBALANCE_Task );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;  
}

/*-----------------------------------------------------------*/

void vMEMSTask(void *pvParameters)
{
	/* queue for MEMS data length */
    volatile int *LED;
    LED = (int *) pvParameters;


	uint8_t Buffer_x[1];
	uint8_t Buffer_y[1];
	uint8_t counter  = 0;
	__IO uint32_t TimingDelay = 0;
	__IO int8_t XOffset;
	__IO int8_t YOffset;


	int8_t temp1 = 0;
	int8_t temp2 = 0;

  	uint8_t TempAcceleration = 0;   

	/* reset offset */


  	LIS302DL_Read(Buffer_x, LIS302DL_OUT_X_ADDR, 1);
	LIS302DL_Read(Buffer_y, LIS302DL_OUT_Y_ADDR, 1);
            
  	XOffset = Buffer_x[0];
  	YOffset = Buffer_y[0];
	/* reset */



for( ;; )
{
	counter++;
	if (counter == 10)
	{

  	LIS302DL_Read(Buffer_x, LIS302DL_OUT_X_ADDR, 1);
	LIS302DL_Read(Buffer_y, LIS302DL_OUT_Y_ADDR, 1);

      /* Remove the offsets values from data */
      Buffer_x[0] -= XOffset;
      Buffer_y[0] -= YOffset;

      /* Update autoreload and capture compare registers value*/
      temp1 = ABS((int8_t)(Buffer_x[0]));
      temp2 = ABS((int8_t)(Buffer_y[0]));
      TempAcceleration = MAX(temp1, temp2);
	
	if(TempAcceleration != 0)
      {
	
        if ((int8_t)Buffer_x[0] < -G)
        {
				STM_EVAL_LEDOn(LED4);


                if ((int8_t)Buffer_x[0] <= G)
                {
                        STM_EVAL_LEDOff(LED3);

                }

                if ((int8_t)Buffer_y[0] <= G)
                {
                       STM_EVAL_LEDOff(LED6);
                }

                if ((int8_t)Buffer_y[0] >= -G)
                {
                        STM_EVAL_LEDOff(LED5);
                }

        }
        if ((int8_t)Buffer_x[0] > G)
        {
				STM_EVAL_LEDOn(LED5);

                                if ((int8_t)Buffer_y[0] <= G)
                                {
                                STM_EVAL_LEDOff(LED4);
                                }

                                if ((int8_t)Buffer_y[0] >= -G)
                                {
                        		STM_EVAL_LEDOff(LED3);
                                }

                                if ((int8_t)Buffer_x[0] >= -G)
                                {
		                        STM_EVAL_LEDOff(LED6);
                                }

        }
        if ((int8_t)Buffer_y[0] > G)
        {

				STM_EVAL_LEDOn(LED3);

                                if ((int8_t)Buffer_x[0] <= G)
                                {
                                        STM_EVAL_LEDOff(LED4);
                                }

                                if ((int8_t)Buffer_y[0] >= -G)
                                {
                                        STM_EVAL_LEDOff(LED5);
                                }

                                if ((int8_t)Buffer_x[0] >= -G)
                                {
                                        STM_EVAL_LEDOff(LED6);
                                }

        }
        if ((int8_t)Buffer_y[0] < -G)
        {

			STM_EVAL_LEDOn(LED6);

                                if ((int8_t)Buffer_x[0] <= G)
                                {
                                        STM_EVAL_LEDOff(LED3);
                                }

                                if ((int8_t)Buffer_y[0] <= G)
                                {
                               STM_EVAL_LEDOff(LED4);
                                }

                                if ((int8_t)Buffer_x[0] >= -G)
                                {
                                STM_EVAL_LEDOff(LED5);
                                }
        }
		counter = 0x00;

    }
  }
}
}

/*-----------------------------------------------------------*/

void vBALANCETask(void *pvParameters)
{
	uint8_t temp1, temp2 = 0;
	__IO uint8_t TempAcceleration = 0;
	uint8_t xBuffer_receive[queueSIZE];
	for( ;; )
	{
	 if(xQueueReceive(xQueue,xBuffer_receive,0)==pdPASS)
		{
		/* Disable All TIM4 Capture Compare Channels */
		TIM_CCxCmd(TIM4, TIM_Channel_1, DISABLE);
		TIM_CCxCmd(TIM4, TIM_Channel_2, DISABLE);
		TIM_CCxCmd(TIM4, TIM_Channel_3, DISABLE);
		TIM_CCxCmd(TIM4, TIM_Channel_4, DISABLE);

		/* Update autoreload and capture compare registers value*/
		temp1=((int8_t)(xBuffer_receive[0])<0)?(int8_t)(xBuffer_receive[0])*(-1):(int8_t)(xBuffer_receive[0]); //ABS
		temp2=((int8_t)(xBuffer_receive[2])<0)?(int8_t)(xBuffer_receive[2])*(-1):(int8_t)(xBuffer_receive[2]); //ABS
		TempAcceleration = (temp1<temp2)?temp2:temp1; //MAX(temp1,temp2)

		if(TempAcceleration != 0)
		{
			if ((int8_t)xBuffer_receive[0] < -2)
			{
				/* Enable TIM4 Capture Compare Channel 4 */
				TIM_CCxCmd(TIM4, TIM_Channel_4, ENABLE);
				/* Sets the TIM4 Capture Compare4 Register value */
				TIM_SetCompare4(TIM4, TIM_CCR/TempAcceleration);
			}
			if ((int8_t)xBuffer_receive[0] > 2)
			{
				/* Enable TIM4 Capture Compare Channel 2 */
				TIM_CCxCmd(TIM4, TIM_Channel_2, ENABLE);
				/* Sets the TIM4 Capture Compare2 Register value */
				TIM_SetCompare2(TIM4, TIM_CCR/TempAcceleration);
			}
			if ((int8_t)xBuffer_receive[2] > 2)
			{
				/* Enable TIM4 Capture Compare Channel 1 */
				TIM_CCxCmd(TIM4, TIM_Channel_1, ENABLE);
				/* Sets the TIM4 Capture Compare1 Register value */
				TIM_SetCompare1(TIM4, TIM_CCR/TempAcceleration);
			}
			if ((int8_t)xBuffer_receive[2] < -2)
			{
				/* Enable TIM4 Capture Compare Channel 3 */
				TIM_CCxCmd(TIM4, TIM_Channel_3, ENABLE);
				/* Sets the TIM4 Capture Compare3 Register value */
				TIM_SetCompare3(TIM4, TIM_CCR/TempAcceleration);
			}

			/* Time base configuration */
			TIM_SetAutoreload(TIM4,  TIM_ARR/TempAcceleration);
		}
	 }
	taskYIELD(); 	//task is going to ready state to allow next one to run
	}
}

/*-----------------------------------------------------------*/

void vLEDTask( void *pvParameters )
{
    volatile int *LED;
    LED = (int *) pvParameters;

	for( ;; )
	{
		STM_EVAL_LEDToggle((Led_TypeDef)LED[0]);
	    vTaskDelay(LED[1]/portTICK_RATE_MS);
	}
}

/*-----------------------------------------------------------*/

void vSWITCHTask( void *pvParameters )
{
	static int i=0;
	for( ;; )
	{
		if(xSemaphoreTake(xSemaphoreSW,( portTickType ) 0) == pdTRUE)
		{
			i^=1;		//just switch the state if semaphore was given

			if(i==0)	//LED3..LD6 tasks ready, BALANCE, MEMS suspended
			{
				vTaskSuspend(xBALANCE_Task);
				TIM_Cmd(TIM4, DISABLE);
				vTaskSuspend(xMEMS_Task);
				prvLED_Config(GPIO);
				vTaskResume(xLED_Tasks[0]);
				vTaskResume(xLED_Tasks[1]);
				vTaskResume(xLED_Tasks[2]);
				vTaskResume(xLED_Tasks[3]);
			}
			else		//MEMS and BALANCE ready, LED tasks suspended
			{
				vTaskSuspend(xLED_Tasks[0]);
				vTaskSuspend(xLED_Tasks[1]);
				vTaskSuspend(xLED_Tasks[2]);
				vTaskSuspend(xLED_Tasks[3]);
				prvLED_Config(TIMER);
				TIM_Cmd(TIM4, ENABLE);
				vTaskResume(xBALANCE_Task);
				vTaskResume(xMEMS_Task);
			}
		}
		taskYIELD(); 	//task is going to ready state to allow next one to run
	}
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amout of FreeRTOS heap that 
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	for( ;; );
}
/*-----------------------------------------------------------*/

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


