/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "FPI-0H00084.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 4
#define SIN_STEPS 1024
#define EXECUTION_PERIOD_MS 1
#define LAMP_PWM_PERIOD 3350
#define LAMP_VOLTAGE_MV 10000
#define VPLUS_DIVIDER 6.0
#define VFPI_DIVIDER 12.5
#define FPI_PERIOD 849.0
#define FPI_DITHER 15.0
#define FPI_PULSE_DMAX 700.0

#define FPI_USER_LIMIT 38300
#define FPI_ERROR_THRESHOLD 250

#define TEMP_CONVERSION 2.8
#define KELVIN_TO_CELSIUS 273.15
#define CONSTANT_A 0.001093
#define CONSTANT_B 0.0002401
#define CONSTANT_C 0.00000006288

#define ADC_CONVERSION_CONSTANT 3300.0 / 65536.0

#define FPI_VOLTAGE_MEASUREMENT_CORRECTION 15650.0 / 15000.0

#define PLUS_VOLTAGE_MEASUREMENT_CORRECTION 12373.0 / 11683.0

#define FPI_VOLTAGE_GAIN 0.9598
#define FPI_VOLTAGE_OFFSET 640.0

#define VOLTAGE_ERROR_MEASURE 10.0 	//mv
#define VOLTAGE_ERROR_AVG_COUNT 10
#define MEASURE_RANGE_MIN 1550		//nm
#define MEASURE_RANGE_MAX 1850		//nm
#define MEASURE_STEP_SIZE 1		//nm
#define STEP_TIME 5				//ms

#define PHOTO_VALUE_SIZE (int)( (float)(MEASURE_RANGE_MAX - MEASURE_RANGE_MIN) / (float)MEASURE_STEP_SIZE )
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC1Buf[2];
uint16_t ADC2Buf[4];

uint32_t lastCycleTick = 0;

int nLAMP_CURRENT_ALERT = 0;
int lampCurrentRAW = 0;
float lampI = 0;
float sinLookup[SIN_STEPS];
float waveFormCounter = 0;
float lampPulse = 0;

int vfpiRAW = 0;
int vPlusRAW = 0;
float vPlus = 0;
float vFPI = 0;

float vFPISetpoint = 15000;
float vFPIError = 0;
const float vFPIProportionalGain = 0.005;
const float vFPIIntegralGain = 0.0002;
float vFPIIntegral = 0;
float vFPIPulse = 0;
int ditherPulse = 0;

int vTempRAW = 0;
float vTemp = 0;
float vThermistor = 0;
float thermistorR = 0;
float tempDegC = 0.0;

int vPhotoPRAW = 0;
int vPhotoNRAW = 0;
float vPhotoP = 0;
float vPhotoN = 0;
float vPhotoDiff = 0;

int wavelengthSetpoint = MEASURE_RANGE_MAX;
int lastTime = 0;
float averageError = 0.0;
uint8_t averageErrorWriteIndex = 0;
float errorAccumulation[VOLTAGE_ERROR_AVG_COUNT];

uint16_t photoValueIndex = 0;
float photoValue[PHOTO_VALUE_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void loop(void);
void initializePeripherals(void);
void measureSlow(void);
void measureFast(void);
void boostConverter(void);

float tempCalc(void);
void lampPulsing(void);
void lampOn(void);

uint16_t wavelengthToVoltage(int wavelength, float temperature);

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
  lastCycleTick = HAL_GetTick();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_OPAMP1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  initializePeripherals();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* LOOP function called every 50us by TIM8 */
void loop(void){
	/* called every 50us */
	measureFast();
	boostConverter();

	uint32_t tick = HAL_GetTick();

	/* called every 1 ms */
	if(tick >= lastCycleTick + EXECUTION_PERIOD_MS)
	{
		lastCycleTick = tick;
		measureSlow();
		lampOn();
		//lampPulsing();

		/* called when the voltage has settled and at least 10ms have passed*/
		if (abs(averageError) < VOLTAGE_ERROR_MEASURE && tick >= lastTime + STEP_TIME)
		{
			uint8_t data[512];
			memset(data, 0, 512);
			sprintf((char *)data, "DATA:$%d$%.2f$%.2f$%.2f$%.5f\n", wavelengthSetpoint, tempDegC, vFPISetpoint, vFPI, vPhotoDiff);
			/* transmit the measurement value */
			HAL_SPI_Transmit_DMA(&hspi1, data, strlen((char *)data));


			lastTime = tick;
			if(wavelengthSetpoint > MEASURE_RANGE_MIN){
				wavelengthSetpoint = wavelengthSetpoint - MEASURE_STEP_SIZE;
			} else {
				wavelengthSetpoint = MEASURE_RANGE_MAX;
				// memset(data, 0, 512);
				// sprintf((char *)data, "NEW MEASUREMENT SERIES:\n");
				// /* transmit the measurement value */
				// HAL_SPI_Transmit_DMA(&hspi1, data, strlen((char *)data));
			}

			tempDegC = tempCalc();
			vFPISetpoint = wavelengthToVoltage(wavelengthSetpoint, tempDegC);

		}
	}
}

void initializePeripherals(void)
{

	/* Calculate SIN Lookup */
	for( int i = 0; i < SIN_STEPS; i ++)
	{
		sinLookup[i] = sin( ( (float) i / (float) SIN_STEPS ) * 2 * M_PI );
	}

	/* read initial values of exti pins */
	nLAMP_CURRENT_ALERT = HAL_GPIO_ReadPin(nI_LAMP_ALERT_GPIO_Port, nI_LAMP_ALERT_Pin);

	/* ADCs */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1Buf, 2);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2Buf, 4);

	/* OPAMP1 for Temp amplification */
	/* The OPAMP uses inverting input for temp sensor measurement and +1.65V reference on non inverting input  */
	/* This way a thermistor resistance value of 20k will produce a output of 0mV. */
	/* PGA Values bewtween 2 and 16 should be automatically used for range detection */

	/* TODO: expand the usage to more then 16x amplification with correction */

	HAL_OPAMP_Start(&hopamp1);

	/* OPAMP2 and OPAMP3 for Photodiode amplification */
	/* With PGA gain of 8 the OPAMPS saturates when the light is directly in front of the MEMS */

	/* TODO: expand the software to dynamically adapt the PGA gain */

	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);


	/* Timers */
	//Lamp
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	//Boost Converter
	//LM5106 EN
	HAL_GPIO_WritePin(EN_PWM_GPIO_Port, EN_PWM_Pin, GPIO_PIN_SET);
	//LM5106 IN
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	/* Start timer for loop triggering */
	HAL_TIM_Base_Start_IT(&htim8);

}

void measureSlow(void)
{
	lampCurrentRAW = ADC1Buf[0];
	vPhotoNRAW = ADC2Buf[2];
	vPhotoPRAW = ADC2Buf[3];
	lampI = 	( (float)lampCurrentRAW 	* ADC_CONVERSION_CONSTANT ) / 200.0 / 0.1;
	vPhotoN = 	( (float)vPhotoNRAW 		* ADC_CONVERSION_CONSTANT );
	vPhotoP = 	( (float)vPhotoPRAW 		* ADC_CONVERSION_CONSTANT );
	vPhotoDiff = vPhotoP - vPhotoN;

	for (uint8_t i = 0; i < VOLTAGE_ERROR_AVG_COUNT; i++)
	{
		averageError += errorAccumulation[i];
	}
	averageError = averageError / (float)VOLTAGE_ERROR_AVG_COUNT;

}

void measureFast(void)
{

	vfpiRAW = ADC1Buf[1];
	vPlusRAW = ADC2Buf[0];
	vPlus = 	( (float)vPlusRAW			* ADC_CONVERSION_CONSTANT ) * VPLUS_DIVIDER * PLUS_VOLTAGE_MEASUREMENT_CORRECTION;
	vFPI = 		( (float)vfpiRAW			* ADC_CONVERSION_CONSTANT ) * VFPI_DIVIDER * FPI_VOLTAGE_MEASUREMENT_CORRECTION;
	vFPI = vFPI * FPI_VOLTAGE_GAIN + FPI_VOLTAGE_OFFSET;

}

float tempCalc(void)
{
	vTempRAW = ADC2Buf[1];
	vTemp = 	( (float)vTempRAW 			* ADC_CONVERSION_CONSTANT );

	vThermistor = 4271.8 - 1.638 * vTemp;
	thermistorR = 0.94 * 20000.0 * (1.0 / ((3300.0/vThermistor) - 1.0));
	float oneByT = CONSTANT_A + CONSTANT_B * log(thermistorR) + CONSTANT_C * pow(log(thermistorR), 3);
	tempDegC = 1/oneByT - KELVIN_TO_CELSIUS;
	//tempDegC = 25.0;
	return tempDegC;
}

void boostConverter(void)
{
	if( vFPI > VFPI_MAX + FPI_ERROR_THRESHOLD)
	{
		vFPISetpoint = FPI_USER_LIMIT + FPI_ERROR_THRESHOLD;
	}
	if( vFPISetpoint > FPI_USER_LIMIT )
		vFPISetpoint = FPI_USER_LIMIT;
	if(vFPISetpoint < vPlus)
		vFPISetpoint = vPlus + FPI_ERROR_THRESHOLD * 2;

	vFPIError = vFPISetpoint - vFPI;
	vFPIIntegral += vFPIError;
	vFPIPulse = vFPIError * vFPIProportionalGain + vFPIIntegral * vFPIIntegralGain;

	errorAccumulation[averageErrorWriteIndex] = vFPIError;
	averageErrorWriteIndex ++;
	if(averageErrorWriteIndex >= VOLTAGE_ERROR_AVG_COUNT)
		averageErrorWriteIndex = 0;

	if(vFPIPulse > FPI_PULSE_DMAX)
		vFPIPulse = FPI_PULSE_DMAX;

	if(vFPIPulse < 0)
		vFPIPulse = 0;


	HAL_GPIO_WritePin(EN_PWM_GPIO_Port, EN_PWM_Pin, GPIO_PIN_SET);
	ditherPulse = ((int)vFPIPulse << 4) + (int)(FPI_DITHER * (float)(vFPIPulse - (int)vFPIPulse));
	htim2.Instance->CCR3 = (int)ditherPulse;

	//htim1.Instance->CCR1 = 0;
	//htim2.Instance->CCR3 = 0;

}

void lampPulsing(void)
{
	if( lampI > 30)
	{
		htim1.Instance->CCR3 = 0;
		return;

	}

	lampPulse = (sinLookup[(int)waveFormCounter] * 0.25 + 0.5) * (float)LAMP_PWM_PERIOD * LAMP_VOLTAGE_MV / vPlus;

	htim1.Instance->CCR3 = (int)lampPulse;

	waveFormCounter += 0.0625;
	if( waveFormCounter >= SIN_STEPS)
		waveFormCounter = 0;
	if( waveFormCounter < 0)
			waveFormCounter = 0;
}

void lampOn(void)
{
	if( lampI > 30)
	{
		htim1.Instance->CCR3 = 0;
		return;

	}

	lampPulse = (float)LAMP_PWM_PERIOD * ((float)LAMP_VOLTAGE_MV / vPlus);

	htim1.Instance->CCR3 = (unsigned int)lampPulse;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch ( GPIO_Pin )
	{
		case nI_LAMP_ALERT_Pin:
			nLAMP_CURRENT_ALERT = HAL_GPIO_ReadPin(nI_LAMP_ALERT_GPIO_Port, nI_LAMP_ALERT_Pin);
			break;
	}

}

uint16_t wavelengthToVoltage(int wavelength, float temperature)
{
	uint16_t voltage = 0;
	// check if wavelength value is in wavelength range
	if (wavelength > WAVELENGTH_MAX)
		return 0;
	if (wavelength < WAVELENGTH_MIN)
		return 0;

	// check if temperature is in operable range
	if (temperature > tempLookup[TEMP_LOOKUP_LEN - 1])
		return 0;
	if (temperature < tempLookup[0])
		return 0;


	uint16_t 	wavelengthIndex = wavelength - WAVELENGTH_MIN;
	uint16_t 	voltageArrayAtTemp[TEMP_LOOKUP_LEN];
	uint8_t 	tempIndexLower = 0;
	uint8_t 	tempIndexUpper = 1;

	for( int i = 0; i < TEMP_LOOKUP_LEN; i ++)
	{
		// copy the values from the lookup table
		voltageArrayAtTemp[i] = fpiLookup[wavelengthIndex][i];

		// get the temperature range
		if( temperature >= tempLookup[i])
			tempIndexLower = i;

		if( temperature < tempLookup[i])
			tempIndexUpper = i;
	}

	uint16_t voltageLower = voltageArrayAtTemp[tempIndexLower];
	uint16_t voltageUpper = voltageArrayAtTemp[tempIndexUpper];
	float tempLower = (float)tempLookup[tempIndexLower];
	float tempUpper = (float)tempLookup[tempIndexUpper];

	float 		tempRange = tempUpper - tempLower;
	uint16_t 	voltageRange = voltageUpper - voltageLower;
	float 		tempOffsetFactor = (tempDegC - tempLower) / tempRange;

	voltage = voltageLower + (int)(tempOffsetFactor * (float)voltageRange);
	if(voltage > VFPI_MAX)
		voltage = VFPI_MAX;
	if(voltage < VFPI_MIN)
		voltage = VFPI_MIN;

	if(voltage > FPI_USER_LIMIT)
		voltage = FPI_USER_LIMIT;

	return voltage;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM8) {
	  loop();
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	htim1.Instance->CCR1 = 0;
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
