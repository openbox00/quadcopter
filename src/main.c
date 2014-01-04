#include "stm32f4xx.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "queue.h"

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


#define DELAY 125     /* msec */
#define queueSIZE	6

/*brushless motor PWM max and min duty cycle*/
#define PWM_MOTOR_INIT_MIN 800
#define PWM_MOTOR_INIT_MAX 2000

#define PWM_MOTOR_MIN 810
#define PWM_MOTOR_MAX 1800

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
xTimerHandle xTimerPidRate;

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

unsigned int thrust[4];

typedef struct {
	float PitchP; //Pitch Proportional Gain Coefficient
	float PitchD; //Pitch Derivative Gain Coefficient

	float RollP;  //Roll Proportional Gain Coefficient
	float RollD;  //Roll Pitch Derivative Gain Coefficient

	float YawD;

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

unsigned int PWM_Motor1_tmp = 0;
unsigned int PWM_Motor2_tmp = 0;
unsigned int PWM_Motor3_tmp = 0;
unsigned int PWM_Motor4_tmp = 0;

static void vPWMctrlTask(void *pvParameters)
{

  char pwm_speed_char[4];

  char pwm_direction[4];

  unsigned int pwm_speed_int = 100;
	
  char direction;

  unsigned int pwm_speed_1 = 0;
  unsigned int pwm_speed_2 = 0;
  unsigned int pwm_speed_3 = 0;
  unsigned int pwm_speed_4 = 0;

  while(1)  // Do not exit
  {

  while (!xQueueReceive(xQueueShell2PWM , pwm_speed_char, portMAX_DELAY));

  pwm_speed_int = atoi(pwm_speed_char);	

  	if (pwm_speed_int > PWM_MOTOR_MAX) {
		pwm_speed_int = PWM_MOTOR_MAX;
	}else if (pwm_speed_int < 0){
		pwm_speed_int = 0;
	}else{
		pwm_speed_int = pwm_speed_int;
	}

	thrust[0] = pwm_speed_int; //PWM_Motor1 = LED4
	thrust[1] = pwm_speed_int; //PWM_Motor2 = LED3
	thrust[2] = pwm_speed_int; //PWM_Motor3 = LED5 
	thrust[3] = pwm_speed_int; //PWM_Motor4 = LED6

	if (thrust[0] != 0 || thrust[1] !=0 || thrust[2] !=0 || thrust[3] != 0)
	{
		PWM_Motor1_tmp = 0;
		PWM_Motor2_tmp = 0;
		PWM_Motor3_tmp = 0;
		PWM_Motor4_tmp = 0;
		PWM_Motor1_tmp = thrust[0];
		PWM_Motor2_tmp = thrust[1];
		PWM_Motor3_tmp = thrust[2];
		PWM_Motor4_tmp = thrust[3];

		thrust[0] = 0;
		thrust[1] = 0;
		thrust[2] = 0;
		thrust[3] = 0;

		pwm_flag = 1;
	}

  }
} 

void Motor_Control(unsigned int Motor1, unsigned int Motor2, unsigned int Motor3, unsigned int Motor4)
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


void vTimerSystemIdle( xTimerHandle pxTimer ){
	pwm_flag = 0;
	Motor_Control(PWM_MOTOR_MIN, PWM_MOTOR_MIN, PWM_MOTOR_MIN, PWM_MOTOR_MIN);
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

int testx;
int testy;

int PITCH, ROLL, YAW;

unsigned int  Motor1, Motor2, Motor3, Motor4;  

PID argv;



void vTimerSample(xTimerHandle pxTimer)
{
 	LIS3DSH_Read(Buffer_Hx, LIS3DSH_OUT_X_H_REG_ADDR, 1);
	LIS3DSH_Read(Buffer_Hy, LIS3DSH_OUT_Y_H_REG_ADDR, 1);


	LIS3DSH_Read(Buffer_Lx, LIS3DSH_OUT_X_L_REG_ADDR, 1);
	LIS3DSH_Read(Buffer_Ly, LIS3DSH_OUT_Y_L_REG_ADDR, 1);


	x_acc = (float)((int16_t)(Buffer_Hx[0] << 8 | Buffer_Lx[0]) - XOffset) * Sensitivity_2G / 1000 * 180 / 3.14159f;
	y_acc = (float)((int16_t)(Buffer_Hy[0] << 8 | Buffer_Ly[0]) - YOffset) * Sensitivity_2G / 1000 * 180 / 3.14159f;
	

    Buffer_GHx[0] = I2C_readreg(L3G4200D_ADDR,OUT_X_H);
    Buffer_GHy[0] = I2C_readreg(L3G4200D_ADDR,OUT_Y_H);

    Buffer_GLx[0] = I2C_readreg(L3G4200D_ADDR,OUT_X_L);
    Buffer_GLy[0] = I2C_readreg(L3G4200D_ADDR,OUT_Y_L);


	x_gyro = (float)((int16_t)(Buffer_GHx[0] << 8 | (Buffer_GLx[0] & 0xF0)) - GXOffset) * Sensitivity_250 / 1000;
	y_gyro = (float)((int16_t)(Buffer_GHy[0] << 8 | (Buffer_GLy[0] & 0xF0)) - GYOffset) * Sensitivity_250 / 1000;

	angle_x = (0.985f) * (angle_x + y_gyro * 0.004f) - (0.015f) * (x_acc);  		
	angle_y = (0.985f) * (angle_y + x_gyro * 0.004f) + (0.015f) * (y_acc); 

	testx = angle_x;
	testy = angle_y;	

	argv.Pitch = angle_y;    //pitch degree
	argv.Roll = angle_x;     //roll degree
	argv.Pitch_v = x_gyro;   //pitch velocity
	argv.Roll_v = y_gyro;    //Roll velocity

	argv.Pitch_err = argv.Pitch_desire - argv.Pitch;
	argv.Roll_err  = argv.Roll_desire - argv.Roll;

	PITCH = (int)(argv.PitchP * argv.Pitch_err - argv.PitchD * argv.Pitch_v);
	ROLL  =	(int)(argv.RollP  * argv.Roll_err  - argv.RollD  * argv.Roll_v);	
	YAW   = (int)(argv.YawD * z_gyro);

		if(pwm_flag == 0){

		}else{

			Motor1 = PWM_Motor1_tmp + PITCH - ROLL - YAW; 	//LD4	
			Motor2 = PWM_Motor2_tmp + PITCH + ROLL + YAW; 	//LD3			
			Motor3 = PWM_Motor3_tmp - PITCH + ROLL - YAW; 	//LD5
			Motor4 = PWM_Motor4_tmp - PITCH - ROLL + YAW; 	//LD6

			Motor_Control(Motor1, Motor2, Motor3, Motor4);
		}

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

	const portTickType twosecDelay = 2000; 
	const portTickType xDelay = 3000; 
	vTaskDelay( twosecDelay  );
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

	LIS3DSH_Read(Buffer_Lx, LIS3DSH_OUT_X_L_REG_ADDR, 1);
	LIS3DSH_Read(Buffer_Ly, LIS3DSH_OUT_Y_L_REG_ADDR, 1);

  	XOffset = (int16_t)(Buffer_Hx[0] << 8 | Buffer_Lx[0]);
 	YOffset = (int16_t)(Buffer_Hy[0] << 8 | Buffer_Ly[0]);


	/* reset gyro offset */	
    Buffer_GHx[0]=I2C_readreg(L3G4200D_ADDR,OUT_X_H);
    Buffer_GHy[0]=I2C_readreg(L3G4200D_ADDR,OUT_Y_H);
    Buffer_GHz[0]=I2C_readreg(L3G4200D_ADDR,OUT_Z_H);

    Buffer_GLx[0]=I2C_readreg(L3G4200D_ADDR,OUT_X_L);
    Buffer_GLy[0]=I2C_readreg(L3G4200D_ADDR,OUT_Y_L);
    Buffer_GLz[0]=I2C_readreg(L3G4200D_ADDR,OUT_Z_L);

  	GXOffset = (int16_t)(Buffer_GHx[0] << 8 | Buffer_GLx[0]);
 	GYOffset = (int16_t)(Buffer_GHy[0] << 8 | Buffer_GLy[0]);
 	GZOffset = (int16_t)(Buffer_GHz[0] << 8 | Buffer_GLz[0]);
	int i;

	for(i = 1;i<128;i++){
    Buffer_GHx[0]=I2C_readreg(L3G4200D_ADDR,OUT_X_H);
    Buffer_GHy[0]=I2C_readreg(L3G4200D_ADDR,OUT_Y_H);
    Buffer_GHz[0]=I2C_readreg(L3G4200D_ADDR,OUT_Z_H);

    Buffer_GLx[0]=I2C_readreg(L3G4200D_ADDR,OUT_X_L);
    Buffer_GLy[0]=I2C_readreg(L3G4200D_ADDR,OUT_Y_L);
    Buffer_GLz[0]=I2C_readreg(L3G4200D_ADDR,OUT_Z_L);

  	GXOffset = (int16_t)(Buffer_GHx[0] << 8 | Buffer_GLx[0]);
 	GYOffset = (int16_t)(Buffer_GHy[0] << 8 | Buffer_GLy[0]);
 	GZOffset = (int16_t)(Buffer_GHz[0] << 8 | Buffer_GLz[0]);	
	}
	
	GXOffset = GXOffset / 128;
	GYOffset = GYOffset / 128;
	GZOffset = GZOffset / 128;


	angle_x = 0;
	angle_y = 0;

	pwm_flag = 0;

    argv.PitchP = 3;//2.5f; //2 //1.5f 
    argv.PitchD = 1; //1
    argv.RollP = 3;//2.5f;	
    argv.RollD = 1;

	argv.YawD = 0;

    argv.Pitch_desire = 0; //Desire angle of Pitch
    argv.Roll_desire = 0; //Desire angle of Roll

	xTimerStart(xTimerSampleRate, 0);
	
	//xTimerStart(xTimerPidRate, 0);		

	while(1){

		qprintf(xQueueUARTSend, "Motor1(P12):%d	,Motor2(P13):%d	,Motor3(P14):%d	,Motor4(P15):%d\n\r", PWM_Motor1, PWM_Motor2, PWM_Motor3, PWM_Motor4);			
		qprintf(xQueueUARTSend, "angle_x: %d	,angle_y: %d\n\r", testx, testy);
	}
}


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{ 
	int timerID = 1;
	int timerID1 = 2;
	int timerID2 = 3;
	/*A Timer used to count how long there is no signal come in*/
	xTimerNoSignal = xTimerCreate("TurnOffTime", 40000 / portTICK_RATE_MS, pdFALSE,  (void *) timerID, vTimerSystemIdle);

	xTimerSampleRate = xTimerCreate("SensorSampleRate", 4 / portTICK_RATE_MS, pdTRUE,  (void *) timerID1, vTimerSample);

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


