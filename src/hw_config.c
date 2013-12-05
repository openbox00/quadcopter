/**
  ******************************************************************************
  * @file    STM32F4-Discovery FreeRTOS demo\hw_config.c
  * @author  T.O.M.A.S. Team
  * @version V1.0.0
  * @date    05-October-2011
  * @brief   Hardware initialization
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

/* Library includes. */
#include "hw_config.h"


/*-----------------------------------------------------------*/
void prvSetupHardware( void )
{
	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure LED IOs as output push-pull */
    /* Initialize LEDs on STM32F4_Discovery board */
	prvLED_Config(GPIO);
	/* Configure User button pin (PA0) as external interrupt -> modes switching */
	STM_EVAL_PBInit(BUTTON_USER,BUTTON_MODE_EXTI);

	/* Configuration of Timer4 to control LEDs based on MEMS data */
	//prvTIM4_Config();

	/* Configure LIS302 in order to produce data used for TIM4 reconfiguration and LED control */
	prvMEMS_Config();

  RCC_Configuration();
  GPIO_Configuration();
  TIM_Configuration();
  USART_Configuration();
}

void prvLED_Config(char state)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  /* Configure PD12, PD13, PD14 and PD15 in output push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  if(state==GPIO)
  {
	  /* standard output pin */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
  }
  else
  {
	  /*-------------------------- GPIO Configuration ----------------------------*/
	  /* GPIOD Configuration: Pins 12, 13, 14 and 15 in output push-pull - alternative mode */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  /* Connect TIM4 pins to AF2 */
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
  }
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

  //Enable GPIO Clocks For USART2
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  //Enable Clocks for USART2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);    
}

/**
  * @brief  configure the PD12~15 to Timers
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //GPIO Configuration for TIM4
  //   GPIO_StructInit(&GPIO_InitStructure); // Reset init structure
 
  //   GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  //   GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
  //   GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  //   GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
      

  //   // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
  //   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15; 
  // //PD12->LED3 PD13->LED4 PD14->LED5 PD15->LED6
  // //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
  // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
  //   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  //   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  //   GPIO_Init( GPIOD, &GPIO_InitStructure );  

    /*----------------------------------------------------------------------*/

  //GPIO Configuration for USART - PA2, PA3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //Connect USART pins to AF
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

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

/**
  * @brief  configure the USART
  * @param  None
  * @retval None
  */
void USART_Configuration(void)
{
  //Structure With Data For USART Configuration
  USART_InitTypeDef USART_InitStructure;
  
  //USART Parameters
  USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;

  //Configuring And Enabling USART2
  USART_Init(USART2, &USART_InitStructure);
  USART_Cmd(USART2, ENABLE);


}

/*------------below is original code------------*/

void prvMEMS_Config(void)
{
	uint8_t ctrl = 0;
	uint32_t i=0;		//simple index for software delay

	LIS302DL_InitTypeDef  LIS302DL_InitStruct;
	LIS302DL_InterruptConfigTypeDef LIS302DL_InterruptStruct;

	/* Set configuration of LIS302DL*/
	LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
	LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
	LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE | LIS302DL_Z_ENABLE;
	LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
	LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
	LIS302DL_Init(&LIS302DL_InitStruct);

	/* Set configuration of Internal High Pass Filter of LIS302DL*/
	LIS302DL_InterruptStruct.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
	LIS302DL_InterruptStruct.SingleClick_Axes = LIS302DL_CLICKINTERRUPT_Z_ENABLE;
	LIS302DL_InterruptStruct.DoubleClick_Axes = LIS302DL_DOUBLECLICKINTERRUPT_Z_ENABLE;
	LIS302DL_InterruptConfig(&LIS302DL_InterruptStruct);

	/* Required delay for the MEMS Accelerometer: Turn-on time = 3/Output data Rate
	                                                            = 3/100 = 30ms */
	for(i=0;i<0x1FFFF;i++);

	/* Configure Interrupt control register: enable Click interrupt1 */
	ctrl = 0x07;
	LIS302DL_Write(&ctrl, LIS302DL_CTRL_REG3_ADDR, 1);

	/* Enable Interrupt generation on click/double click on Z axis */
	ctrl = 0x70;
	LIS302DL_Write(&ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);

	/* Configure Click Threshold on X/Y axis (10 x 0.5g) */
	ctrl = 0xAA;
	LIS302DL_Write(&ctrl, LIS302DL_CLICK_THSY_X_REG_ADDR, 1);

	/* Configure Click Threshold on Z axis (10 x 0.5g) */
	ctrl = 0x0A;
	LIS302DL_Write(&ctrl, LIS302DL_CLICK_THSZ_REG_ADDR, 1);

	/* Configure Time Limit */
	ctrl = 0x03;
	LIS302DL_Write(&ctrl, LIS302DL_CLICK_TIMELIMIT_REG_ADDR, 1);

	/* Configure Latency */
	ctrl = 0x7F;
	LIS302DL_Write(&ctrl, LIS302DL_CLICK_LATENCY_REG_ADDR, 1);

	/* Configure Click Window */
	ctrl = 0x7F;
	LIS302DL_Write(&ctrl, LIS302DL_CLICK_WINDOW_REG_ADDR, 1);
}

/**
  * @brief  MEMS accelerometer management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LIS302DL_TIMEOUT_UserCallback(void)
{
  /* MEMS Accelerometer Timeout error has occured */
  while (1)
  {
  }
}

void prvTIM4_Config(void)
{
  uint16_t PrescalerValue = 0;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* -----------------------------------------------------------------------
    TIM4 Configuration: Output Compare Timing Mode:

    In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1),
    since APB1 prescaler is different from 1 (APB1 Prescaler = 4, see system_stm32f4xx.c file).
      TIM4CLK = 2 * PCLK1
      PCLK1 = HCLK / 4
      => TIM4CLK = 2*(HCLK / 4) = HCLK/2 = SystemCoreClock/2

    To get TIM4 counter clock at 2 KHz, the prescaler is computed as follows:
       Prescaler = (TIM4CLK / TIM1 counter clock) - 1
       Prescaler = (168 MHz/(2 * 2 KHz)) - 1 = 41999

    To get TIM4 output clock at 1 Hz, the period (ARR)) is computed as follows:
       ARR = (TIM4 counter clock / TIM4 output clock) - 1
           = 1999

    TIM4 Channel1 duty cycle = (TIM4_CCR1/ TIM4_ARR)* 100 = 50%
    TIM4 Channel2 duty cycle = (TIM4_CCR2/ TIM4_ARR)* 100 = 50%
    TIM4 Channel3 duty cycle = (TIM4_CCR3/ TIM4_ARR)* 100 = 50%
    TIM4 Channel4 duty cycle = (TIM4_CCR4/ TIM4_ARR)* 100 = 50%

    ==> TIM4_CCRx = TIM4_ARR/2 = 1000  (where x = 1, 2, 3 and 4).

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.
  ----------------------------------------------------------------------- */

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 2000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = TIM_ARR;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = TIM_CCR;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 */
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_CCxCmd(TIM4, TIM_Channel_1, DISABLE);

  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* Output Compare PWM1 Mode configuration: Channel2 */
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_CCxCmd(TIM4, TIM_Channel_2, DISABLE);

  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* Output Compare PWM1 Mode configuration: Channel3 */
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_CCxCmd(TIM4, TIM_Channel_3, DISABLE);

  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* Output Compare PWM1 Mode configuration: Channel4 */
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_CCxCmd(TIM4, TIM_Channel_4, DISABLE);

  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
