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

#include "hw_config.h"  //all hardware configuration was setted here
#include "main.h"

#include "string-util.c"

/* variable parameter function*/
#include <stdarg.h>

/** @addtogroup STM32F4-Discovery_Demo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DELAY 125     /* msec */
#define queueSIZE	6

/*brushless motor PWM max and min duty cycle*/
#define PWM_MOTOR_MIN 100
#define PWM_MOTOR_MAX 1000
#define TEST 200

/* angle */
#define G 2

/* Private macro -------------------------------------------------------------*/
/*PWM signal to drive brushless motor*/
#define PWM_Motor1 TIM4->CCR1   
#define PWM_Motor2 TIM4->CCR2   
#define PWM_Motor3 TIM4->CCR3   
#define PWM_Motor4 TIM4->CCR4   

/* Private variables ---------------------------------------------------------*/
/* Queue structure used for passing messages. */
typedef struct {
	char str[100];
} serial_str_msg;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void pwm(void)
{
  const portTickType xDelay = 1000; // portTICK_RATE_MS;

 //Delay_1ms(50);

  Motor_Control(PWM_MOTOR_MAX, PWM_MOTOR_MAX, PWM_MOTOR_MAX, PWM_MOTOR_MAX);

  Delay_1ms(100);

  Motor_Control(PWM_MOTOR_MIN, PWM_MOTOR_MIN, PWM_MOTOR_MIN, PWM_MOTOR_MIN);
  
  Delay_1ms(100);

  while(1)  // Do not exit
  {
   Motor_Control(TEST, TEST, TEST, TEST);
   
   vTaskDelay( xDelay );

   Motor_Control(TEST+50, TEST+50, TEST+50, TEST+50);

   vTaskDelay( xDelay );

  }
 
  return(0); // System will implode
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


/*********************************************************************************************************/
/* Private functions ---------------------------------------------------------*/

/* Task functions declarations */
static void vMEMSTask(void *pvParameters);
static void UsartTask(void *pvParameters);

/* semaphores, queues declarations */
xQueueHandle xQueueUARTSend;

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{ 
<<<<<<< HEAD
	/* create a pipe for MEMS->TIM4 data exchange */
	xQueue=xQueueCreate(1,queueSIZE*sizeof(uint8_t));

	/* create semaphores... */
	vSemaphoreCreateBinary( xSemaphoreSW );

	/* ...and clean them up */
	if(xSemaphoreTake(xSemaphoreSW, ( portTickType ) 0) == pdTRUE);
	
=======
	
	/*a queue for tansfer the senddate to USART task*/
	xQueueUARTSend = xQueueCreate(15, sizeof(serial_str_msg));

>>>>>>> ac1e70594edb89011fb1fe40d9d7d2eb9ff85178
	/* initialize hardware... */
	prvSetupHardware();
	RCC_Configuration();
	TIM_Configuration();
	GPIO_Configuration();

	/* Start the tasks defined within this file/specific to this demo. */
	xTaskCreate(vMEMSTask, ( signed portCHAR * ) "MEMS", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, NULL );
	xTaskCreate(UsartTask, ( signed portCHAR * ) "USART", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;  
}



/* Task functions ------------------------------------------------- */

//Task For Sending Data Via USART
static void UsartTask(void *pvParameters)
{
	//Variable to store received data	
	uint32_t Data;
	uint8_t curr_char;	

	while(1) {

	
		serial_str_msg msg;
		//Wait for character
		// while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET) {
  //           if (USART_GetFlagStatus(USART2, (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)))
		// 		USART_ReceiveData(USART2); // Clear Error
		// }

		//Collect the caracter
		//Data = USART_ReceiveData(USART2);

		while (!xQueueReceive(xQueueUARTSend , &msg, portMAX_DELAY));

		/* Write each character of the message to the RS232 port. */
		curr_char = 0;
		while (msg.str[curr_char] != '\0') {
			//Wait till the flag resets
			while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
			//Send the data
			USART_SendData(USART2, msg.str[curr_char]); // Send Char from queue
			curr_char++;
		}
	}

	while(1);
}



void vMEMSTask(void *pvParameters)
{
	/* queue for MEMS data length */
    volatile int *LED;
    LED = (int *) pvParameters;

	uint8_t Buffer_x[1];
	uint8_t Buffer_y[1];
	uint8_t Buffer_z[1];
	uint8_t counter  = 0;
	__IO uint32_t TimingDelay = 0;
	__IO int8_t XOffset;
	__IO int8_t YOffset;
	__IO int8_t ZOffset;


	int8_t temp1 = 0;
	int8_t temp2 = 0;

  	uint8_t TempAcceleration = 0;   

	/* reset offset */
  	LIS302DL_Read(Buffer_x, LIS302DL_OUT_X_ADDR, 1);
	LIS302DL_Read(Buffer_y, LIS302DL_OUT_Y_ADDR, 1);
	LIS302DL_Read(Buffer_z, LIS302DL_OUT_Z_ADDR, 1);
            
  	XOffset = (int8_t)Buffer_x[0];
  	YOffset = (int8_t)Buffer_y[0];
  	ZOffset = (int8_t)Buffer_z[0];
 
	/* reset */
<<<<<<< HEAD
	
=======

	for( ;; )
	{
		counter++;
		if (counter == 10)
		{
>>>>>>> ac1e70594edb89011fb1fe40d9d7d2eb9ff85178

  		LIS302DL_Read(Buffer_x, LIS302DL_OUT_X_ADDR, 1);
		LIS302DL_Read(Buffer_y, LIS302DL_OUT_Y_ADDR, 1);
		LIS302DL_Read(Buffer_z, LIS302DL_OUT_Z_ADDR, 1);

	    /* Remove the offsets values from data */
	    //Buffer_x[0] -= XOffset;
	    //Buffer_y[0] -= YOffset;
	    //Buffer_z[0] -= ZOffset;


<<<<<<< HEAD
      /* Remove the offsets values from data */
      Buffer_x[0] -= XOffset;
      Buffer_y[0] -= YOffset;
		if ((int8_t)Buffer_x[0] > 2 && (int8_t)Buffer_y[0] > 2)
		{
		PWM_Motor1 = 0*(int8_t)Buffer_x[0];//left
		PWM_Motor2 = 10*(int8_t)Buffer_y[0];//up
		PWM_Motor3 = 10*(int8_t)Buffer_x[0];//right
		PWM_Motor4 = 0*(int8_t)Buffer_y[0];//dowm
		}
		else if((int8_t)Buffer_x[0] < -2 && (int8_t)Buffer_y[0] < -2)
		{
		PWM_Motor1 = -10*(int8_t)Buffer_x[0];//left
		PWM_Motor2 = 0;//up
		PWM_Motor3 = 0;//right
		PWM_Motor4 = -10*(int8_t)Buffer_y[0];//dowm		
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
=======
		temp1 = (uint8_t)((int8_t)(Buffer_x[0]) + 23);
	    temp2 = (uint8_t)((int8_t)(Buffer_y[0]) + 23);
	    //qprintf(xQueueUARTSend, "x: %d, y: %d\n\r", temp1, temp2);

	    /* Update autoreload and capture compare registers value*/
	    temp1 = ABS((int8_t)(Buffer_x[0]));
	    temp2 = ABS((int8_t)(Buffer_y[0]));
    	TempAcceleration = MAX(temp1, temp2);
		  
		//qprintf(xQueueUARTSend, "abcdefghijklmn1234567890\n\r");  
		qprintf(xQueueUARTSend, "x: %d, y: %d, z: %d\n\r", (int8_t)Buffer_x[0], (int8_t)Buffer_y[0], (int8_t)Buffer_z[0]);
		//qprintf(xQueueUARTSend, "x: %d, y: %d\n\r", temp1, temp2);
		
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
>>>>>>> ac1e70594edb89011fb1fe40d9d7d2eb9ff85178
	}
}

/*-----------------------------------------------------------*/



// void vSWITCHTask( void *pvParameters )
// {
// 	static int i=0;
// 	for( ;; )
// 	{
// 		if(xSemaphoreTake(xSemaphoreSW,( portTickType ) 0) == pdTRUE)
// 		{
// 			i^=1;		//just switch the state if semaphore was given

// 			if(i==0)	//LED3..LD6 tasks ready, BALANCE, MEMS suspended
// 			{
// 				vTaskSuspend(xBALANCE_Task);
// 				TIM_Cmd(TIM4, DISABLE);
// 				vTaskSuspend(xMEMS_Task);
// 				prvLED_Config(GPIO);
// 				vTaskResume(xLED_Tasks[0]);
// 				vTaskResume(xLED_Tasks[1]);
// 				vTaskResume(xLED_Tasks[2]);
// 				vTaskResume(xLED_Tasks[3]);
// 			}
// 			else		//MEMS and BALANCE ready, LED tasks suspended
// 			{
// 				vTaskSuspend(xLED_Tasks[0]);
// 				vTaskSuspend(xLED_Tasks[1]);
// 				vTaskSuspend(xLED_Tasks[2]);
// 				vTaskSuspend(xLED_Tasks[3]);
// 				prvLED_Config(TIMER);
// 				TIM_Cmd(TIM4, ENABLE);
// 				vTaskResume(xBALANCE_Task);
// 				vTaskResume(xMEMS_Task);
// 			}
// 		}
// 		taskYIELD(); 	//task is going to ready state to allow next one to run
// 	}
// }

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


