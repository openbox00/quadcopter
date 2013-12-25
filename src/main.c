#include "stm32f4xx.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "hw_config.h"  //all hardware configuration was setted here
#include "main.h"
#include "shell.h"
#include "I2C.h"
#include "stm32f4_discovery_l3g4200d.h"

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
#define PWM_MOTOR_INIT_MIN 100
#define PWM_MOTOR_INIT_MAX 1000


#define PWM_MOTOR_MIN 120
#define PWM_MOTOR_MAX 300

/*acc sensitivity*/
#define Sensitivity_2G	0.06  	
#define Sensitivity_4G	0.12  
#define Sensitivity_6G	0.18  
#define Sensitivity_8G	0.24  
#define Sensitivity_16G	0.72   

/*gyro sensitivity*/
#define Sensitivity_250		8.75  	
#define Sensitivity_500		17.5  
#define Sensitivity_2000	70  
  	

/* Private macro -------------------------------------------------------------*/
/*PWM signal to drive brushless motor*/
#define PWM_Motor1 TIM4->CCR1   
#define PWM_Motor2 TIM4->CCR2   
#define PWM_Motor3 TIM4->CCR3   
#define PWM_Motor4 TIM4->CCR4   

/* Task functions declarations */
static void vPWMctrlTask(void *pvParameters);
static void vBalanceTask(void *pvParameters);

static void vUsartSendTask(void *pvParameters);
static void vUsartReciveTask(void *pvParameters);

int pwm_flag;

/* semaphores, queues declarations */
xQueueHandle xQueueUARTSend;
xQueueHandle xQueueUARTRecvie;
xQueueHandle xQueueShell2PWM;
xQueueHandle xQueuePWMdirection;

/* software Timers */
xTimerHandle xTimerNoSignal;
xTimerHandle xTimerSampleRate;

/*Task Handler */
xTaskHandle xBalanceHandle;

/* Private variables ---------------------------------------------------------*/
/* Queue structure used for passing messages. */
typedef struct {
	char str[50];
} serial_str_msg;

typedef struct {
        char ch;
} serial_ch_msg;

int throttle[4];

typedef struct {
	float PitchP; //Pitch Proportional Gain Coefficient
	float PitchD; //Pitch Derivative Gain Coefficient

	float RollP;  //Roll Proportional Gain Coefficient
	float RollD;  //Roll Pitch Derivative Gain Coefficient

	float Pitch_desire; //Desired Pich angle
	float Roll_desire;  //Desired Roll angle

	float Pitch_err;  //Pitch error
	float Roll_err;   //Roll error

	float Pitch, Roll;  	//present Pitch and Roll
	float Pitch_v, Roll_v;  //present Pitch and Roll velosity
} PID;

char receive_byte()
{
        serial_ch_msg msg;

        /* Wait for a byte to be queued by the receive interrupts handler. */
        while (!xQueueReceive(xQueueUARTRecvie, &msg, portMAX_DELAY));

        return msg.ch;
}

/* for fg ref zzz0072*/
int receive_byte_noblock(char *ch)
{
    serial_ch_msg msg;
    int rval = xQueueReceive(xQueueUARTRecvie, &msg, 10);
    if ( rval == 1) {
        *ch = msg.ch;
    }
    return rval;
}

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

static void vPWMctrlTask(void *pvParameters)
{

  char pwm_speed_char[4];

  char pwm_direction[4];

  int pwm_speed_int = 100;
	
  char direction;
	
  int pwm_speed_w = 0;
  int pwm_speed_a = 0;
  int pwm_speed_s = 0;
  int pwm_speed_d = 0;

  while(1)  // Do not exit
  {

  while (!xQueueReceive(xQueuePWMdirection , pwm_direction, portMAX_DELAY));

  direction = pwm_direction[0];

  while (!xQueueReceive(xQueueShell2PWM , pwm_speed_char, portMAX_DELAY));

  pwm_speed_int = atoi(pwm_speed_char);	

  	if (pwm_speed_int >300) {
		pwm_speed_int = 300;
	}else if (pwm_speed_int < 0){
		pwm_speed_int = 0;
	}else{
		pwm_speed_int = pwm_speed_int;
	}

	if (direction == 'w'){
		pwm_speed_w = pwm_speed_w + pwm_speed_int; 
		pwm_speed_a = pwm_speed_a; 
		pwm_speed_s = pwm_speed_s;
		pwm_speed_d	= pwm_speed_d;
	}else if (direction == 'a'){
		pwm_speed_w = pwm_speed_w; 
		pwm_speed_a = pwm_speed_a + pwm_speed_int; 
		pwm_speed_s = pwm_speed_s;
		pwm_speed_d	= pwm_speed_d;
	}else if (direction == 's'){
		pwm_speed_w = pwm_speed_w; 
		pwm_speed_a = pwm_speed_a; 
		pwm_speed_s = pwm_speed_s + pwm_speed_int;
		pwm_speed_d	= pwm_speed_d;
	}else if (direction == 'd'){
		pwm_speed_w = pwm_speed_w; 
		pwm_speed_a = pwm_speed_a; 
		pwm_speed_s = pwm_speed_s;
		pwm_speed_d	= pwm_speed_d + pwm_speed_int;
	}else{	
		pwm_speed_w = pwm_speed_int;
		pwm_speed_a = pwm_speed_int;
		pwm_speed_s = pwm_speed_int;
		pwm_speed_d = pwm_speed_int;
	}

	throttle[0] = pwm_speed_a; //PWM_Motor1 = LED4
	throttle[1] = pwm_speed_w; //PWM_Motor2 = LED3
	throttle[2] = pwm_speed_d; //PWM_Motor3 = LED5 
	throttle[3] = pwm_speed_s; //PWM_Motor4 = LED6

	qprintf(xQueueUARTSend, "set w: %d , a: %d , s: %d , d: %d\n\r",
							pwm_speed_w, pwm_speed_a, pwm_speed_s, pwm_speed_d);	

 	//Motor_Control(pwm_speed_w, pwm_speed_a, pwm_speed_s, pwm_speed_d);
 	qprintf(xQueueUARTSend, "LD5: %d,LD4: %d,LD6: %d,LD3: %d\r\n", PWM_Motor3, PWM_Motor1, PWM_Motor4, PWM_Motor2);
 	
  }
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
								
	PWM_Motor1 = Motor1;	// 12 	18 + 2.4=20.4
	PWM_Motor2 = Motor2;	// 13	18  	
	PWM_Motor3 = Motor3;	// + 2;	// 14	18 - 0.2 = 17.8
	PWM_Motor4 = Motor4;	// 15	18 + 1 = 19
}


void vTimerSystemIdle( xTimerHandle pxTimer ){
	pwm_flag = 0;
	Motor_Control(120, 120, 120, 120);
	qprintf(xQueueUARTSend, "30 sec idle time pass ... trun off motor\n\r");
}

uint8_t Buffer_Hx[1];
uint8_t Buffer_Hy[1];
uint8_t Buffer_Hz[1];

uint8_t Buffer_Lx[1];
uint8_t Buffer_Ly[1];
uint8_t Buffer_Lz[1];	

__IO int16_t XOffset;
__IO int16_t YOffset;
__IO int16_t ZOffset;

float x_acc;
float y_acc;
float z_acc;

uint8_t Buffer_GHx[1];
uint8_t Buffer_GHy[1];
uint8_t Buffer_GHz[1];

uint8_t Buffer_GLx[1];
uint8_t Buffer_GLy[1];
uint8_t Buffer_GLz[1];

__IO int16_t GXOffset;
__IO int16_t GYOffset;
__IO int16_t GZOffset;

float x_gyro;
float y_gyro;
float z_gyro;

float angle_x;
float angle_y;
float angle_z;

// int16_t x_1, y_1;
// uint8_t x_2, y_2;

// int16_t gx_1, gy_1, gz_1;
// uint8_t gx_2, gy_2, gz_2;

u16 Motor1, Motor2, Motor3, Motor4;        

PID argv;

void vTimerSample(xTimerHandle pxTimer){
 	LIS3DSH_Read(Buffer_Hx, LIS3DSH_OUT_X_H_REG_ADDR, 1);
	LIS3DSH_Read(Buffer_Hy, LIS3DSH_OUT_Y_H_REG_ADDR, 1);
	// LIS3DSH_Read(Buffer_Hz, LIS3DSH_OUT_Z_H_REG_ADDR, 1);


	LIS3DSH_Read(Buffer_Lx, LIS3DSH_OUT_X_L_REG_ADDR, 1);
	LIS3DSH_Read(Buffer_Ly, LIS3DSH_OUT_Y_L_REG_ADDR, 1);
	// LIS3DSH_Read(Buffer_Lz, LIS3DSH_OUT_Z_L_REG_ADDR, 1);	


	x_acc = (float)((int16_t)(Buffer_Hx[0] << 8 | Buffer_Lx[0]) - XOffset) * Sensitivity_2G / 1000 * 180 / 3.14159;
	y_acc = (float)((int16_t)(Buffer_Hy[0] << 8 | Buffer_Ly[0]) - YOffset) * Sensitivity_2G / 1000 * 180 / 3.14159;
	// z_acc = (float)((int16_t)(Buffer_Hz[0] << 8 | Buffer_Lz[0]) - ZOffset) * Sensitivity_2G / 1000 * 180 / 3.14159;

	// x_1 = (int16_t)x_acc;
	// x_2 = (x_acc - x_1) * 1000;
	// y_1 = (int16_t)y_acc;
	// y_2 = (y_acc - y_1) * 1000;
		

    Buffer_GHx[0] = I2C_readreg(L3G4200D_ADDR,OUT_X_H);
    Buffer_GHy[0] = I2C_readreg(L3G4200D_ADDR,OUT_Y_H);
    // Buffer_GHz[0] = I2C_readreg(L3G4200D_ADDR,OUT_Z_H);

    Buffer_GLx[0] = I2C_readreg(L3G4200D_ADDR,OUT_X_L);
    Buffer_GLy[0] = I2C_readreg(L3G4200D_ADDR,OUT_Y_L);
    // Buffer_GLz[0] = I2C_readreg(L3G4200D_ADDR,OUT_Z_L);


	x_gyro = (float)((int16_t)(Buffer_GHx[0] << 8 | Buffer_GLx[0]) - GXOffset) * Sensitivity_250 / 1000;
	y_gyro = (float)((int16_t)(Buffer_GHy[0] << 8 | Buffer_GLy[0]) - GYOffset) * Sensitivity_250 / 1000;
	// z_gyro = (float)((int16_t)(Buffer_GHz[0] << 8 | Buffer_GLz[0]) - GZOffset) * Sensitivity_250 / 1000;

	// gx_1 = (int16_t)x_gyro;
	// gx_2 = (x_gyro - gx_1) * 1000;
	// gy_1 = (int16_t)y_gyro;
	// gy_2 = (y_gyro - gy_1) * 1000;
	// gz_1 = (int16_t)z_gyro;
	// gz_2 = (z_gyro - gz_1) * 1000;		


	angle_x = (0.93) * (angle_x + y_gyro * 0.01) - (0.07) * (x_acc);  		
	angle_y = (0.93) * (angle_y + x_gyro * 0.01) + (0.07) * (y_acc); 
	//angle_z = (0.966) * (angle_z + z_gyro * 0.001) + (0.034) * (z_acc); 
	

	argv.Pitch = angle_y;    //pitch degree
	argv.Roll = angle_x;     //roll degree
	argv.Pitch_v = x_gyro;   //pitch velocity
	argv.Roll_v = y_gyro;    //Roll velocity

	argv.Pitch_err = argv.Pitch_desire - argv.Pitch;
	argv.Roll_err  = argv.Roll_desire - argv.Roll;

	if (throttle[0] != 0 || throttle[1] !=0 || throttle[2] !=0 || throttle[3] != 0)
	{
		PWM_Motor1 = throttle[0];
		PWM_Motor2 = throttle[1];
		PWM_Motor3 = throttle[2];
		PWM_Motor4 = throttle[3];
		throttle[0] = 0;
		throttle[1] = 0;
		throttle[2] = 0;
		throttle[3] = 0;

		pwm_flag = 1;
	}


	if(pwm_flag == 0){

	}else{

		Motor1 = PWM_Motor1 + (int)( argv.PitchP * argv.Pitch_err - argv.PitchD * argv.Pitch_v	) - (int)( argv.RollP  * argv.Roll_err  - argv.RollD  * argv.Roll_v); 	//LD4	
		Motor2 = PWM_Motor2 + (int)( argv.PitchP * argv.Pitch_err - argv.PitchD * argv.Pitch_v	) + (int)( argv.RollP  * argv.Roll_err  - argv.RollD  * argv.Roll_v); 	//LD3			
		Motor3 = PWM_Motor3 - (int)( argv.PitchP * argv.Pitch_err - argv.PitchD * argv.Pitch_v	) + (int)( argv.RollP  * argv.Roll_err  - argv.RollD  * argv.Roll_v); 	//LD5
		Motor4 = PWM_Motor4 - (int)( argv.PitchP * argv.Pitch_err - argv.PitchD * argv.Pitch_v	) - (int)( argv.RollP  * argv.Roll_err  - argv.RollD  * argv.Roll_v); 	//LD6
		
		Motor_Control(Motor1, Motor2, Motor3, Motor4);
	}
}
/*********************************************************************************************************/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{ 
	int timerID = 1;
	int timerID1 = 2;
	/*A Timer used to count how long there is no signal come in*/
	xTimerNoSignal = xTimerCreate("TurnOffTime", 30000 / portTICK_RATE_MS, pdFALSE,  (void *) timerID, vTimerSystemIdle);

	xTimerSampleRate = xTimerCreate("SensorSampleRate", 10 / portTICK_RATE_MS, pdTRUE,  (void *) timerID1, vTimerSample);


	/*a queue for tansfer the senddate to USART task*/
	xQueueUARTSend = xQueueCreate(15, sizeof(serial_str_msg));
   	xQueueUARTRecvie = xQueueCreate(15, sizeof(serial_ch_msg));
   	xQueueShell2PWM = xQueueCreate(1, sizeof(int));
   	xQueuePWMdirection = xQueueCreate(1, sizeof(int));

	/* initialize hardware... */
	prvSetupHardware();
	init_I2C1();
	//L3G4200D_Init();
	xTimerStart(xTimerNoSignal, 0);


	/* Start the tasks defined within this file/specific to this demo. */
	xTaskCreate(vPWMctrlTask, ( signed portCHAR * ) "pwmctrl", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, NULL );
	xTaskCreate(vUsartSendTask, ( signed portCHAR * ) "USART", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, NULL);
	xTaskCreate(vUsartReciveTask, ( signed portCHAR * ) "Usartrecive", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, NULL);
	xTaskCreate(shell, ( signed portCHAR * ) "shell", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY + 5, NULL);
	xTaskCreate(vBalanceTask, ( signed portCHAR * ) "Balance", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, xBalanceHandle);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;  
}



/* Task functions ------------------------------------------------- */

//Task For Sending Data Via USART
static void vUsartSendTask(void *pvParameters)
{
	//Variable to store received data	
	uint32_t Data;
	uint8_t curr_char;	

	while(1) {
		serial_str_msg msg;

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

//Task For Sending Data Via USART
static void vUsartReciveTask(void *pvParameters)
{
	//Variable to store received data	
	uint32_t Data;
	uint8_t curr_char;	

	while(1) {
		
		//Wait for character
		 while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET) {
           if (USART_GetFlagStatus(USART2, (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)))
		 		USART_ReceiveData(USART2); // Clear Error
		 		
		 }
		
		xTimerReset(xTimerNoSignal, 10);
		//Collect the character
		Data = USART_ReceiveData(USART2);
		while (!xQueueSendToBack(xQueueUARTRecvie, &Data, portMAX_DELAY));
		//qprintf(xQueueUARTRecvie, "%c", Data); 

	}

}


void vBalanceTask(void *pvParameters)
{
	const portTickType ms100 = 100;  	
	const portTickType ms10 = 10;  
	const portTickType sec1 = 1000; 	

	const portTickType xDelay = 6000; 
   	PWM_Motor1 = PWM_MOTOR_INIT_MAX;
   	PWM_Motor2 = PWM_MOTOR_INIT_MAX;
	PWM_Motor3 = PWM_MOTOR_INIT_MAX;
   	PWM_Motor4 = PWM_MOTOR_INIT_MAX;   	
  	vTaskDelay( xDelay );  //6S
  	PWM_Motor1 = PWM_MOTOR_INIT_MIN;
   	PWM_Motor2 = PWM_MOTOR_INIT_MIN;
	PWM_Motor3 = PWM_MOTOR_INIT_MIN;
   	PWM_Motor4 = PWM_MOTOR_INIT_MIN;   	

	/*inital Offset value of Gryo. and Acce.*/

  	LIS3DSH_Read(Buffer_Hx, LIS3DSH_OUT_X_H_REG_ADDR, 1);
	LIS3DSH_Read(Buffer_Hy, LIS3DSH_OUT_Y_H_REG_ADDR, 1);
	// LIS3DSH_Read(Buffer_Hz, LIS3DSH_OUT_Z_H_REG_ADDR, 1);

	LIS3DSH_Read(Buffer_Lx, LIS3DSH_OUT_X_L_REG_ADDR, 1);
	LIS3DSH_Read(Buffer_Ly, LIS3DSH_OUT_Y_L_REG_ADDR, 1);
	// LIS3DSH_Read(Buffer_Lz, LIS3DSH_OUT_Z_L_REG_ADDR, 1);	

  	XOffset = (int16_t)(Buffer_Hx[0] << 8 | Buffer_Lx[0]);
 	YOffset = (int16_t)(Buffer_Hy[0] << 8 | Buffer_Ly[0]);
 	// ZOffset = (int16_t)(Buffer_Hz[0] << 8 | Buffer_Lz[0]);

	x_acc = (float)((int16_t)(Buffer_Hx[0] << 8 | Buffer_Lx[0]) - XOffset) * Sensitivity_2G / 1000 * 180 / 3.14159;
	y_acc = (float)((int16_t)(Buffer_Hy[0] << 8 | Buffer_Ly[0]) - YOffset) * Sensitivity_2G / 1000 * 180 / 3.14159;
	// z_acc = (float)((int16_t)(Buffer_Hz[0] << 8 | Buffer_Lz[0]) - ZOffset) * Sensitivity_2G / 1000 * 180 / 3.14159;

	/* reset gyro offset */	
    Buffer_GHx[0]=I2C_readreg(L3G4200D_ADDR,OUT_X_H);
    Buffer_GHy[0]=I2C_readreg(L3G4200D_ADDR,OUT_Y_H);
    // Buffer_GHz[0]=I2C_readreg(L3G4200D_ADDR,OUT_Z_H);

    Buffer_GLx[0]=I2C_readreg(L3G4200D_ADDR,OUT_X_L);
    Buffer_GLy[0]=I2C_readreg(L3G4200D_ADDR,OUT_Y_L);
    // Buffer_GLz[0]=I2C_readreg(L3G4200D_ADDR,OUT_Z_L);

  	GXOffset = (int16_t)(Buffer_GHx[0] << 8 | Buffer_GLx[0]);
 	GYOffset = (int16_t)(Buffer_GHy[0] << 8 | Buffer_GLy[0]);
 	// GZOffset = (int16_t)(Buffer_GHz[0] << 8 | Buffer_GLz[0]);

	x_gyro = (float)((int16_t)(Buffer_GHx[0] << 8 | Buffer_GLx[0]) - GXOffset) * Sensitivity_250 / 1000;
	y_gyro = (float)((int16_t)(Buffer_GHy[0] << 8 | Buffer_GLy[0]) - GYOffset) * Sensitivity_250 / 1000;
	// z_gyro = (float)((int16_t)(Buffer_GHz[0] << 8 | Buffer_GLz[0]) - GZOffset) * Sensitivity_250 / 1000;

	angle_x = 0;
	angle_y = 0;
	// angle_z = 0;


	pwm_flag = 0;

    argv.PitchP = 0.4;        
    argv.PitchD = 0.003;
    argv.RollP = 0.4;
    argv.RollD = 0.003;

    argv.Pitch_desire = 0; //Desire angle of Pitch
    argv.Roll_desire = 0; //Desire angle of Roll

	xTimerStart(xTimerSampleRate, 0);	



	while(1){

		qprintf(xQueueUARTSend, "Motor1(P12): %d	,Motor2(P13): %d	,Motor3(P14):	%d	,Motor4(P15):	%d\n\r", PWM_Motor1, PWM_Motor2, PWM_Motor3, PWM_Motor4);			
		//vTaskDelay(ms100);
		//qprintf(xQueueUARTSend, "x_acc :	%d	, y_acc :	%d \n\r", (int)x_acc, (int)y_acc);		
		//qprintf(xQueueUARTSend, "x_gyro :	%d	, y_gyro :	%d \n\r", (int)x_gyro, (int)y_gyro);		
		//qprintf(xQueueUARTSend, "angle_x :	%d	, angle_y :	%d \n\r", (int)angle_x, (int)angle_y);	
		//qprintf(xQueueUARTSend, "Pitch: %d, Roll: %d\r\n", Pitch, Roll);
		//vTaskDelay(ms100); //Setting rate is 100Hz	
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


