/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <lvgl.h>
#include <screen_driver.h>
#include <touch_sensor_driver.h>
#include "usbd_cdc_if.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_FULL_ASSERT
/**
 * @brief  FMC SDRAM Mode definition register defines
 *
 * @brief  defines copied from the STM32F469I-Disco Board support package
 */
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

/**
 * @brief  SDRAM refresh counter (90 MHz SD clock)
 *
 * @brief  defines copied from the STM32F469I-Disco Board support package
 */
#define REFRESH_COUNT       ((uint32_t)0x0569)
#define SDRAM_TIMEOUT       ((uint32_t)0xFFFF)

#define ENC_PPR 			1600
#define ENC_PULSE_RAD 		(M_TWOPI/ENC_PPR)
#define ENC_MENSURE_TIME_S	0.1
#define ENC_D				(ENC_PPR * ENC_MENSURE_TIME_S)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DMA2D_HandleTypeDef hdma2d;

DSI_HandleTypeDef hdsi;

I2C_HandleTypeDef hi2c1;

LTDC_HandleTypeDef hltdc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

static volatile uint32_t enc_cnt_cw_total = 0;
static volatile uint32_t enc_cnt_ccw_total = 0;
static volatile uint16_t enc_pulses_old = 0;
static volatile int16_t enc_pulses_diff = 0;
static volatile double AngularVelocity = 0.0;
static volatile double Frequency = 0.0;
static volatile double RPM = 0.0;

static volatile double kP = 1.0;
static volatile double kI = 0.0;
static volatile double kD = 0.0;
static volatile double P = 0.0;
static volatile double I = 0.0;
static volatile double D = 0.0;
static volatile double PID = 0.0;

static double setPoint 	= 0.0;
static volatile double error 	= 0.0;
static volatile double lastRPM	= 0.0;

static volatile char string[32];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DMA2D_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_LTDC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void create_gui(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Função de interrupção do TIMER 5
 * @details Ocerre a cada 100 milisegundos
 */
void TIM5_IRQHandler(void)
{
	if (TIM5->SR & (TIM_SR_UIF))
	{
		// Limpa a flag de interrupção
		TIM5->SR &= ~(TIM_SR_UIF);
		// Pega a quantidade de pulsos do encder
		uint16_t counter_value = TIM3->CNT;
		// Calcula a diferença entre o valor atual e o último valor lido
		enc_pulses_diff = counter_value - enc_pulses_old;
		// Atualiza o último lido
		enc_pulses_old = counter_value;
		// Calcula a velocidade angular, frequencia e RPM
		AngularVelocity = (M_TWOPI * enc_pulses_diff) / ENC_D;
		Frequency = AngularVelocity / M_TWOPI;
		RPM = Frequency * 60.0;
		/* Implementação do controle PID */
		// Calcula o erro
		error = setPoint - RPM;
		// Calcula a proporcional
		P = error * kP;
		// Calcula a itegral
		I += error * kI * ( /* 100ms */0.1) ;
		// Calcula a derivada
		D = (lastRPM - RPM) * kD * ( /* 100ms */0.1);
		// Soma os parâmetros
		PID = P + I + D;
		// Seta o duty cycle do PWM com o valor calculado do PID
		TIM14->CCR1 = (uint16_t)PID;
		/* Cria uma linha de texto contendo "tempo em segundos, rotação em RPM" */
		size_t l = sprintf(string, "$%0.3f %0.3f;", RPM, setPoint);
		/* Transmite via usb a linha gerada para o supervisório */
		CDC_Transmit_FS((uint8_t*)string, l);
	}
	NVIC_ClearPendingIRQ(TIM5_IRQn);
}

uint32_t ADC_DATA[2];
float mA1, mA2;

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DMA2D_Init();
  MX_DSIHOST_DSI_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_LTDC_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_TIM14_Init();
  MX_TIM5_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	lv_init();
	screen_driver_init();
	touch_sensor_driver_init();
	create_gui();

	HAL_ADC_Start_DMA(&hadc1, ADC_DATA, 2);

//	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
//	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
//
//	HAL_Delay(2000);

	/* Configura o timer 5 para gerar interrupção a cada 5 ms.
	 * Na interrupção será calculado o RPM. */
//	TIM5->PSC   = 90-1; 		//
//	TIM5->ARR   = 100000-1;
//	TIM5->RCR   =  0;
//	TIM5->DIER  = TIM_DIER_UIE;
//	NVIC_EnableIRQ(TIM5_IRQn);
//	TIM5->CR1  |= TIM_CR1_CEN; Is = (Vout x 1k) / (RS x RL)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		lv_timer_handler();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48|RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 7;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
static void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */

  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_VidCfgTypeDef VidCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 4;
  hdsi.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
  PLLInit.PLLNDIV = 125;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV2;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV1;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 35;
  PhyTimings.ClockLaneLP2HSTime = 35;
  PhyTimings.DataLaneHS2LPTime = 35;
  PhyTimings.DataLaneLP2HSTime = 35;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 10;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  VidCfg.VirtualChannelID = 0;
  VidCfg.ColorCoding = DSI_RGB565;
  VidCfg.LooselyPacked = DSI_LOOSELY_PACKED_DISABLE;
  VidCfg.Mode = DSI_VID_MODE_BURST;
  VidCfg.PacketSize = 800;
  VidCfg.NumberOfChunks = 0;
  VidCfg.NullPacketSize = 0;
  VidCfg.HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
  VidCfg.VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
  VidCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  VidCfg.HorizontalSyncActive = 5;
  VidCfg.HorizontalBackPorch = 77;
  VidCfg.HorizontalLine = 1982;
  VidCfg.VerticalSyncActive = 120;
  VidCfg.VerticalBackPorch = 150;
  VidCfg.VerticalFrontPorch = 150;
  VidCfg.VerticalActive = 480;
  VidCfg.LPCommandEnable = DSI_LP_COMMAND_ENABLE;
  VidCfg.LPLargestPacketSize = 16;
  VidCfg.LPVACTLargestPacketSize = 0;
  VidCfg.LPHorizontalFrontPorchEnable = DSI_LP_HFP_ENABLE;
  VidCfg.LPHorizontalBackPorchEnable = DSI_LP_HBP_ENABLE;
  VidCfg.LPVerticalActiveEnable = DSI_LP_VACT_ENABLE;
  VidCfg.LPVerticalFrontPorchEnable = DSI_LP_VFP_ENABLE;
  VidCfg.LPVerticalBackPorchEnable = DSI_LP_VBP_ENABLE;
  VidCfg.LPVerticalSyncActiveEnable = DSI_LP_VSYNC_ENABLE;
  VidCfg.FrameBTAAcknowledgeEnable = DSI_FBTAA_DISABLE;
  if (HAL_DSI_ConfigVideoMode(&hdsi, &VidCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */

  /* USER CODE END DSIHOST_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AH;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AH;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 1;
  hltdc.Init.VerticalSync = 119;
  hltdc.Init.AccumulatedHBP = 35;
  hltdc.Init.AccumulatedVBP = 269;
  hltdc.Init.AccumulatedActiveW = 835;
  hltdc.Init.AccumulatedActiveH = 749;
  hltdc.Init.TotalWidth = 869;
  hltdc.Init.TotalHeigh = 899;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 800;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 480;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xC0000000;
  pLayerCfg.ImageWidth = 800;
  pLayerCfg.ImageHeight = 480;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 99;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 4096;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 99;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 4069;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_LOW;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */
	static FMC_SDRAM_CommandTypeDef Command;
  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_32;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

	/*
	 * The following code is copied from the STM32F469I-Disco Board Support Package
	 */
	__IO uint32_t tmpmrd = 0;

	/* Step 1: Configure a clock configuration enable command */
	Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber      = 1;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

	/* Step 2: Insert 100 us minimum delay */
	/* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
	HAL_Delay(1);

	/* Step 3: Configure a PALL (precharge all) command */
	Command.CommandMode            = FMC_SDRAM_CMD_PALL;
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber      = 1;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

	/* Step 4: Configure an Auto Refresh command */
	Command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber      = 8;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

	/* Step 5: Program the external memory mode register */
	tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |\
			SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |\
			SDRAM_MODEREG_CAS_LATENCY_3           |\
			SDRAM_MODEREG_OPERATING_MODE_STANDARD |\
			SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

	Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber      = 1;
	Command.ModeRegisterDefinition = tmpmrd;

	/* Send the command */
	HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

	/* Step 6: Set the refresh rate counter */
	/* Set the device refresh rate */
	HAL_SDRAM_ProgramRefreshRate(&hsdram1, REFRESH_COUNT);

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED3_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OTG_FS1_PowerSwitchOn_Pin|EXT_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED3_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS1_PowerSwitchOn_Pin EXT_RESET_Pin */
  GPIO_InitStruct.Pin = OTG_FS1_PowerSwitchOn_Pin|EXT_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PH7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart3, (const uint8_t *)ptr, len, 100);
	return len;
}

//static lv_obj_t * btn1;
//static lv_obj_t * btn2;

static lv_obj_t * chart;
static lv_chart_series_t * ser_rpm;
static lv_chart_series_t * ser_setPoint;
//static lv_obj_t * kb;
//static lv_obj_t * ta_setPoint;
//static lv_obj_t * ta_kP;
//static lv_obj_t * ta_kI;
//static lv_obj_t * ta_kD;

//static lv_obj_t * l_error;
static lv_obj_t * l_duty;
//static lv_obj_t * l_P;
//static lv_obj_t * l_I;
//static lv_obj_t * l_D;
static lv_obj_t * l_RPM;

//static char str_error[10] 	= {"0"};
static char str_duty[16] 	= {"0"};
//static char str_P[10] 		= {"0"};
//static char str_I[10] 		= {"0"};
//static char str_D[10] 		= {"0"};
static char str_RPM[16] 	= {"0"};

typedef union
{
    unsigned int Val;
    struct
    {
        unsigned Flag:1;        // flag of filling of summ
        unsigned Index:9;       // buffer index
        unsigned Filter_sum:23; // summ of filter's value
    } Reg;
} FILTER_REG;

#define COUNT_FILTER	4

uint16_t  filter_average(uint16_t ADC_val, uint16_t* buf, FILTER_REG* filter_reg){

    if (filter_reg -> Reg.Flag){
        filter_reg -> Reg.Filter_sum -= buf[filter_reg -> Reg.Index];
        filter_reg -> Reg.Filter_sum += ADC_val;
        buf[filter_reg -> Reg.Index] = ADC_val;
        if (filter_reg -> Reg.Index >= COUNT_FILTER - 1){
            filter_reg -> Reg.Index = 0;
        }
        else{
            filter_reg -> Reg.Index++;
        }
    }
    else{
        filter_reg -> Reg.Filter_sum += ADC_val;
        buf[filter_reg -> Reg.Index] = ADC_val;
        if (filter_reg -> Reg.Index >= COUNT_FILTER - 1){
            filter_reg -> Reg.Index = 0;
            filter_reg -> Reg.Flag = 1;
        }
        else{
            filter_reg -> Reg.Index++;
        }
    }
    return (filter_reg -> Reg.Filter_sum / COUNT_FILTER);
}

uint16_t buffADC1[COUNT_FILTER], buffADC2[COUNT_FILTER];
FILTER_REG filterADC1, filterADC2;
static uint16_t ADC1_voltage, ADC2_voltage;

void chartUpdate(lv_timer_t * tmr)
{

	ADC1_voltage = filter_average((uint16_t)(ADC_DATA[0]), buffADC1, &filterADC1);
	ADC2_voltage = filter_average((uint16_t)(ADC_DATA[1]), buffADC2, &filterADC2);


	mA1 = ((ADC1_voltage * 3.3)/4096)/(100*0.1)*1000.0;
	mA2 = ((ADC2_voltage * 3.3)/4096)/(100*0.1)*1000.0;

	lv_chart_set_next_value(chart, ser_rpm,  (lv_coord_t)mA1);
	lv_chart_set_next_value(chart, ser_setPoint, (lv_coord_t)mA2);

//	static int i = 0;
//	if(i++ >= 5)
//	{
		sprintf(str_RPM, "%.3f mA", mA1);
		sprintf(str_duty, "%.3f mA", mA2);

		size_t l = sprintf(string, "$%0.3f %0.3f;", mA1, mA2);
		/* Transmite via usb a linha gerada para o supervisório */
		CDC_Transmit_FS((uint8_t*)string, l);

//		sprintf(str_error, "%.2f", error);
//		sprintf(str_P, "%.2f", P);
//		sprintf(str_I, "%.2f", I);
//		sprintf(str_D, "%.2f", D);
//
		lv_label_set_text_static(l_RPM, str_RPM);
		lv_label_set_text_static(l_duty, str_duty);
//		lv_label_set_text_static(l_error, str_error);
//		lv_label_set_text_static(l_P, str_P);
//		lv_label_set_text_static(l_I, str_I);
//		lv_label_set_text_static(l_D, str_D);
//	}
}

//static void event_handler(lv_event_t * e)
//{
//	lv_event_code_t code = lv_event_get_code(e);
//	lv_obj_t * bt = lv_event_get_target(e);
//
//	if(code == LV_EVENT_CLICKED)
//	{
//		if(bt == btn1)
//		{
//			setPoint = atof(lv_textarea_get_text(ta_setPoint));
//			kP = atof(lv_textarea_get_text(ta_kP));
//			kI = atof(lv_textarea_get_text(ta_kI));
//			kD = atof(lv_textarea_get_text(ta_kD));
//		}
//		else
//		{
//			P = 0.0;
//			I = 0.0;
//			D = 0.0;
//			PID = 0.0;
//			setPoint 	= 0.0;
//			error 	= 0.0;
//			lastRPM	= 0.0;
//			RPM = 0;
//			TIM14->CCR1 = 0;
//		}
//	}
//}

//static void ta_event_cb(lv_event_t * e)
//{
//	lv_obj_t * ta = lv_event_get_target(e);
//
//	if(e->code == LV_EVENT_FOCUSED)
//	{
//		if(lv_obj_has_flag(kb, LV_OBJ_FLAG_HIDDEN))
//		{
//			lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
//		}
//		lv_keyboard_set_textarea(kb, ta);
//	}
//	else if(e->code == LV_EVENT_DEFOCUSED)
//	{
//		lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
//	}
//}

void create_gui(void)
{
	//lv_obj_t * label;
//	btn1 = lv_btn_create(lv_scr_act());
//	lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
//	lv_obj_set_pos(btn1, 636, 25);
//	lv_obj_set_size(btn1, 100, 60);
//
//	label = lv_label_create(btn1);
//	lv_label_set_text(label, "Start");
//	lv_obj_center(label);
//
//	btn2 = lv_btn_create(lv_scr_act());
//	lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, NULL);
//	lv_obj_set_pos(btn2, 636, 125);
//	lv_obj_set_size(btn2, 100, 60);
//
//	label = lv_label_create(btn2);
//	lv_label_set_text(label, "Reset");
//	lv_obj_center(label);

	/*Create a chart*/
	chart = lv_chart_create(lv_scr_act());
	lv_obj_set_size(chart, 720, 400);
	lv_obj_align(chart, LV_ALIGN_BOTTOM_MID, 20, -30);

	lv_chart_set_point_count(chart, 200);
	lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -10, 50);
	lv_chart_set_range(chart, LV_CHART_AXIS_SECONDARY_Y, 0, 200);

	lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_Y, 10, 5, 7, 5, true, 40);
	lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_X, 5, 5, 20, 5, true, 40);

	lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_CIRCULAR);

	lv_obj_refresh_ext_draw_size(chart);

	ser_rpm = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_X);
	ser_setPoint = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_X);

	lv_chart_set_zoom_x(chart, 250);

	lv_timer_create(chartUpdate, 80, NULL);

//	lv_obj_t * l = lv_label_create(lv_scr_act());
//	lv_label_set_text(l, "SetPoint:");
//	lv_obj_set_pos(l, 8, 18);
//
//	ta_setPoint = lv_textarea_create(lv_scr_act());
//	lv_textarea_set_text(ta_setPoint, "0.0");
//	lv_obj_set_pos(ta_setPoint, 80, 5);
//	lv_obj_set_size(ta_setPoint, 120, 42);
//	lv_obj_add_event_cb(ta_setPoint, ta_event_cb, LV_EVENT_ALL, NULL);
//
//	l = lv_label_create(lv_scr_act());
//	lv_label_set_text(l, "kP:");
//	lv_obj_set_pos(l, 8, 63 );
//
//	ta_kP = lv_textarea_create(lv_scr_act());
//	lv_textarea_set_text(ta_kP, "1.0");
//	lv_obj_set_pos(ta_kP, 80, 52);
//	lv_obj_set_size(ta_kP, 120, 42);
//	lv_obj_add_event_cb(ta_kP, ta_event_cb, LV_EVENT_ALL, NULL);
//
//	l = lv_label_create(lv_scr_act());
//	lv_label_set_text(l, "kI:");
//	lv_obj_set_pos(l, 8, 110 );
//
//	ta_kI = lv_textarea_create(lv_scr_act());
//	lv_textarea_set_text(ta_kI, "1.0");
//
//	lv_obj_set_pos(ta_kI, 80, 99);
//	lv_obj_set_size(ta_kI, 120, 42);
//	lv_obj_add_event_cb(ta_kI, ta_event_cb, LV_EVENT_ALL, NULL);
//
//	l = lv_label_create(lv_scr_act());
//	lv_label_set_text(l, "kD:");
//	lv_obj_set_pos(l, 8, 152 );
//
//	ta_kD = lv_textarea_create(lv_scr_act());
//	lv_textarea_set_text(ta_kD, "0.0");
//	lv_obj_set_pos(ta_kD, 80, 146);
//	lv_obj_set_size(ta_kD, 120, 42);
//	lv_obj_add_event_cb(ta_kD, ta_event_cb, LV_EVENT_ALL, NULL);
//
//	/*Create a keyboard*/
//	kb = lv_keyboard_create(lv_scr_act());
//	lv_obj_set_size(kb,  LV_HOR_RES, LV_VER_RES / 2);
//	lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_NUMBER);
//
//	lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);

	lv_obj_t * l = lv_label_create(lv_scr_act());
	lv_label_set_text(l, "Corrente 1:");
	lv_obj_set_pos(l, 100, 20 );

	l = lv_label_create(lv_scr_act());
	lv_label_set_text(l, "Corrente 2:");
	lv_obj_set_pos(l, 300, 20 );

//	l = lv_label_create(lv_scr_act());
//	lv_label_set_text(l, "Erro:");
//	lv_obj_set_pos(l, 360, 81 );
//
//	l = lv_label_create(lv_scr_act());
//	lv_label_set_text(l, "P:");
//	lv_obj_set_pos(l, 360, 112 );
//
//	l = lv_label_create(lv_scr_act());
//	lv_label_set_text(l, "I:");
//	lv_obj_set_pos(l, 360, 142 );
//
//	l = lv_label_create(lv_scr_act());
//	lv_label_set_text(l, "D:");
//	lv_obj_set_pos(l, 360, 172 );
//
	l_RPM = lv_label_create(lv_scr_act());
	lv_label_set_text_static(l_RPM, str_RPM);
	lv_obj_set_pos(l_RPM, 190, 20 );

	l_duty = lv_label_create(lv_scr_act());
	lv_label_set_text_static(l_duty, str_duty);
	lv_obj_set_pos(l_duty, 390, 20 );
//
//	l_error = lv_label_create(lv_scr_act());
//	lv_label_set_text_static(l_error, str_error);
//	lv_obj_set_pos(l_error, 403, 81 );
//
//	l_P = lv_label_create(lv_scr_act());
//	lv_label_set_text_static(l_P, str_P);
//	lv_obj_set_pos(l_P, 403, 112 );
//
//	l_I = lv_label_create(lv_scr_act());
//	lv_label_set_text_static(l_I, str_I);
//	lv_obj_set_pos(l_I, 403, 142 );
//
//	l_D = lv_label_create(lv_scr_act());
//	lv_label_set_text_static(l_D, str_D);
//	lv_obj_set_pos(l_D, 403, 172 );
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
