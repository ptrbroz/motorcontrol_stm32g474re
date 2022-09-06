/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


/// high-bandwidth 3-phase motor control for robots
/// Written by Ben Katz, with much inspiration from Bayley Wang, Nick Kirkby, Shane Colton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "structs.h"
#include <stdio.h>
#include <string.h>

//#include "stm32g4xx_flash.h"
//#include "flash_writer.h"
#include "position_sensor.h"
//#include "preference_writer.h"
#include "hw_config.h"
#include "user_config.h"
#include "fsm.h"
#include "foc.h"
#include "math_ops.h"
#include "calibration.h"
#include "flash_access.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define VERSION_NUM 2.1f


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Flash Registers */
float __float_reg[64];
int __int_reg[256];
//PreferenceWriter prefs;

int count = 0;

/* Structs for control, etc */

ControllerStruct controller;
ObserverStruct observer;
COMStruct com;
FSMStruct state;
EncoderStruct comm_encoder;
DRVStruct drv;
CalStruct comm_encoder_cal;
CANTxMessage can_tx;
CANRxMessage can_rx;

/* init but don't allocate calibration arrays */
int *error_array = NULL;
int *lut_array = NULL;

uint8_t Serial2RxBuffer[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_FDCAN2_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */


  /* Load settings from flash */
    load_from_flash();

  /* Sanitize configs in case flash is empty*/
  if(E_ZERO==-1){E_ZERO = 0;}
  if(M_ZERO==-1){M_ZERO = 0;}
  if(isnan(I_BW) || I_BW==-1){I_BW = 1000;}
  if(isnan(I_MAX) || I_MAX ==-1){I_MAX=40;}
  if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=0;}
  if(CAN_ID==-1){CAN_ID = 1;}
  if(CAN_MASTER==-1){CAN_MASTER = 0;}
  if(CAN_TIMEOUT==-1){CAN_TIMEOUT = 1000;}
  if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}
  if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 125.0f;}
  if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
  if(isnan(I_CAL)||I_CAL==-1){I_CAL = 5.0f;}
  if(isnan(PPAIRS) || PPAIRS==-1){PPAIRS = 21.0f;}
  if(isnan(GR) || GR==-1){GR = 1.0f;}
  if(isnan(KT) || KT==-1){KT = 1.0f;}
  if(isnan(KP_MAX) || KP_MAX==-1){KP_MAX = 500.0f;}
  if(isnan(KD_MAX) || KD_MAX==-1){KD_MAX = 5.0f;}
  if(isnan(P_MAX)){P_MAX = 12.5f;}
  if(isnan(P_MIN)){P_MIN = -12.5f;}
  if(isnan(V_MAX)){V_MAX = 65.0f;}
  if(isnan(V_MIN)){V_MIN = -65.0f;}


  printf("\r\nFirmware Version Number: %.2f\r\n", VERSION_NUM);

  /* Controller Setup */
  if(PHASE_ORDER){							// Timer channel to phase mapping

  }
  else{

  }

if(COMMUTATE_OVERRIDE){
		printf("BEWARE, debug mode with overriden q, d axis voltages in commutate()!\r\n");
	 }


#define PHASE_ORDER_OVERRIDE 0
  if(PHASE_ORDER_OVERRIDE){
	  PHASE_ORDER = 0;
	  printf("BEWARE, debug mode setting PHASE_ORDER to %d!!\n\r", PHASE_ORDER);
  }

  init_controller_params(&controller);

  /* calibration "encoder" zeroing */
  memset(&comm_encoder_cal.cal_position, 0, sizeof(EncoderStruct));

  /* commutation encoder setup */
  comm_encoder.m_zero = M_ZERO;
  comm_encoder.e_zero = E_ZERO;
  comm_encoder.ppairs = PPAIRS;
  ps_warmup(&comm_encoder, 100);			// clear the noisy data when the encoder first turns on

  if(EN_ENC_LINEARIZATION){memcpy(&comm_encoder.offset_lut, &ENCODER_LUT, sizeof(comm_encoder.offset_lut));}	// Copy the linearization lookup table
  else{memset(&comm_encoder.offset_lut, 0, sizeof(comm_encoder.offset_lut));}
  //for(int i = 0; i<128; i++){printf("%d\r\n", comm_encoder.offset_lut[i]);}

  /* Turn on ADCs */
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc3);

  /* DRV8353 setup */
  HAL_GPIO_WritePin(DRV_CS, GPIO_PIN_SET ); 	// CS high
  HAL_GPIO_WritePin(ENABLE_PIN, GPIO_PIN_SET );
  HAL_Delay(1);
  //drv_calibrate(drv);
  HAL_Delay(1);
  drv_write_DCR(drv, 0x0, DIS_GDF_EN, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
  HAL_Delay(1);
  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x1, 0x1, 0x1, SEN_LVL_1_0);
  HAL_Delay(1);
  zero_current(&controller);
  HAL_Delay(1);
  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x1, 0x0, 0x0, 0x0, SEN_LVL_1_0);
  HAL_Delay(1);
  drv_write_OCPCR(drv, TRETRY_50US, DEADTIME_50NS, OCP_DEG_8US, OCP_DEG_8US, VDS_LVL_1_50);
  HAL_Delay(1);
  drv_disable_gd(drv);
  HAL_Delay(1);
  //drv_enable_gd(drv);   */
  printf("ADC A OFFSET: %d     ADC B OFFSET: %d\r\n", controller.adc_a_offset, controller.adc_b_offset);

  // TODO REMOVE - turning off / on driver for debug purposes.
  #define DRV_DISABLED 0
  #define DRV_NOPRINT 0
  if(DRV_DISABLED){
	  HAL_GPIO_WritePin(ENABLE_PIN, GPIO_PIN_RESET);
	  printf("BEWARE, debug mode with driver disabled!!\n\r");
  	  }
  if(DRV_NOPRINT){
	  printf("BEWARE, debug mode with driver error printing disabled!\n\r");
  	  }

  #define EZERO_OVERRIDE 0
  if(EZERO_OVERRIDE){
  	  printf("BEWARE, debug mode with hardcoded e-zero!\n\r");
  	  E_ZERO = 0;
    	  }

  /*
  while(1){
	  uint16_t retVal = drv_spi_write(&drv, (LSR<<11)|(val&mask));
	  printf("sent %d to %d, gotten %d\n\r", val, (LSR<<12), retVal);
	  HAL_Delay(1);

	  retVal = drv_spi_write(&drv, (1<<16)|(LSR<<11)|(val&mask));
	  printf("reading from %d, gotten %d\n\r", (LSR<<12), retVal);
	  HAL_Delay(1);
	  //val++;s
  }
  */




  HAL_GPIO_WritePin(LED1, 1 );

  /* Turn on PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  /* CAN setup */
  /*
  can_rx_init(&can_rx);
  can_tx_init(&can_tx);
  HAL_CAN_Start(&CAN_H); start CAN
  __HAL_CAN_ENABLE_IT(&CAN_H, CAN_IT_RX_FIFO0_MSG_PENDING);  Start can interrupt
	*/

  can_rx_init(&can_rx);
  can_tx_init(&can_tx);


  HAL_FDCAN_Start(&CAN_H);
  HAL_FDCAN_ActivateNotification(&CAN_H, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  //__HAL_FDCAN_ENABLE_IT(&CAN_H, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); DOES NOT WORK!



  /* Set Interrupt Priorities */
  /*
  NVIC_SetPriority(PWM_ISR, 1); // commutation > communication
  NVIC_SetPriority(CAN_ISR, 3);
  */
  //ben bugfix 12.6.22:
  HAL_NVIC_SetPriority(PWM_ISR, 0x0,0x0);
  HAL_NVIC_SetPriority(CAN_ISR, 0x01, 0x01);

  /* Start the FSM */
  state.state = MENU_MODE;
  state.next_state = MENU_MODE;
  state.ready = 1;


  /* Turn on interrupts */
  HAL_UART_Receive_IT(&huart2, (uint8_t *)Serial2RxBuffer, 1);
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_ADC_Start_IT(&ADC_CH_VBUS);


  HAL_GPIO_WritePin(LED1, 0 );


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(100);

	  if(! (DRV_DISABLED||DRV_NOPRINT) ) drv_print_faults(drv);



	  //controller.dtc_u = 0.3;
	  //set_dtc(&controller);
	  //printf("%f %f %f \n\r", controller.i_a, controller.i_b, controller.i_c);


	  //printf("ppairs %f \n\r", PPAIRS);
	  //printf("vbus %f \n\r",controller.v_bus);
	  //printf("Ria,b %d %d \n\r", controller.adc_a_raw, controller.adc_b_raw);
	  //printf("ia,b %f %f | %d %d \n\r", controller.i_a, controller.i_b, controller.adc_a_raw-controller.adc_a_offset, controller.adc_b_raw-controller.adc_b_offset);
	  //pack_reply(&can_tx, CAN_ID,  0.0, 0.0, 0.0);	// Pack response
	  //HAL_FDCAN_AddMessageToTxFifoQ(&CAN_H, &can_tx.tx_header, can_tx.data); //replacement for above line
	  //printf("sent\n");
	  if(state.state==MOTOR_MODE){
	  	  //printf("%.2f %.2f %.2f %.2f %.2f %.2f\r\n", controller.i_a, controller.i_b, controller.i_d, controller.i_q, controller.dtheta_elec, controller.dtheta_mech);
		  //printf("%f.2 ", controller.p_des);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_ADC345|RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
