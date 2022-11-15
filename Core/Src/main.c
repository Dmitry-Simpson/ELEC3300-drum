/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "MPU9250.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void mpu9250_read_reg(uint8_t reg, uint8_t *data, uint8_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t timer_flag = 0;
int runned = 0;
uint8_t xMSB;
uint8_t xLSB;
uint8_t yMSB;
uint8_t yLSB;
uint8_t HMC5883L_Addr = 0x1E;
uint8_t CRA = 0x70;
uint8_t CRB = 0xA0;
uint8_t CR_C = 0x00;
uint8_t request = 0x00;
uint8_t qua = 0;
int xycoor = 0;
int lefting = 0;
int righting = 0;

int hit = 0;
int record = 1;
int hitcount = 0;

char buff[31];
char buff1[31];
char buff2[31];

float acc[3];
float gyro[3];
float mag[3];
char counter[9];
char _hitcount[9];
//char str[100];
int16_t AccData[3], GyroData[3], MagData[3], Gyro_offset[3];


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
  MX_I2C2_Init();
  MX_FSMC_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  LCD_INIT();
  MPU9250_Init();
  MPU9250_SetAccelRange(ACCEL_RANGE_8G);
  MPU9250_SetGyroRange(GYRO_RANGE_1000DPS);
  imu_calibrateGyro_noclass(Gyro_offset);
//  HAL_I2C_Mem_Write(&hi2c2,HMC5883L_Addr<<1,0x00,1,&CRA,1,100);
//  HAL_I2C_Mem_Write(&hi2c2,HMC5883L_Addr<<1,0x01,1,&CRB,1,100);
//  int16_t accel_data;
//  uint8_t imu_data[14];

//  uint16_t _Combine(uint8_t b1, uint8_t b2) {
//	  uint16_t combined = b1 << 8 | b2;
//	  return combined;
//  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (!runned) {
//		  LCD_DrawString(20,20,"Chan Lik Yeung");
//		  LCD_DrawDot(120,160, 0xF800);
//		  LCD_DrawEllipse(120, 160, 25, 75, BLACK) ;
		  LCD_Clear(0,0, 239,319, WHITE);
//		  LCD_DrawString(120,20, "YValue");
//		  LCD_DrawString(180,20, "XValue");
//		  LCD_DrawString(20,40, "Value : ");
//		  LCD_DrawString(20,60, "Angle : ");

		  runned = 1;
	  }

	  MPU9250_GetData(AccData, MagData, GyroData);
	  imu_normalizeGyro(GyroData,Gyro_offset);



//	  printf("%08d;%08d;%08d;%08d;%08d;%08d;%08d;%08d;%08d\n",
//	    (int16_t)AccData[0], (int16_t)AccData[1], (int16_t)AccData[2],
//	    (int16_t)GyroData[0], (int16_t)GyroData[1], (int16_t)GyroData[2],
//	    (int16_t)MagData[0], (int16_t)MagData[1], (int16_t)MagData[2]);

	  for (int i=0;i<3;i++) {
		  acc[i] = AccData[i]/2048.0;
		  gyro[i] = imu_normalizeGyro(GyroData[i], Gyro_offset[i]) ;
		  mag[i] = MagData[i];

	  }

//	  if (acc[0]>3 && (acc[0] != 8 && acc[0] != 9 && acc[0] != 10 )) {
	  if (acc[0] > 2.3 && (acc[2] == 8 ||acc[2] == 9 ||acc[2] == 10)) {
		  lefting = 1;
		  righting = 0;
		  HAL_Delay(20);
	  } else if ( acc[0] < -2.3 && (acc[2] == 8 || acc[2] == 9 ||acc[2] == 10)) {
		  righting = 1;
		  lefting = 0;
		  HAL_Delay(20);

	  } else {
	  		  lefting=0;
	  		  righting=0;
	  }

	  if (acc[2]< -8 && record == 1) {
		  hit = 1; record = 0;
		  HAL_Delay(10);
	  } else {
		  hit = 0;
	  }

	  if (lefting != 0) {xycoor -= lefting; LCD_DrawString(0, 100, "lefting");}
	  if (righting != 0) {xycoor+= righting;LCD_DrawString(0, 100, "righting");}
	  if (hit == 1 && record == 0) {hitcount++; record = 1; }


	  sprintf(buff, "acc : %6.2f,%6.2f,%6.2f", acc[0], acc[1], acc[2]);
	  LCD_DrawString(0, 40, buff);
	  sprintf(buff, "gyro: %6.0f,%6.0f,%6.0f", gyro[0], gyro[1], gyro[2]);
	  LCD_DrawString(0, 60, buff);
	  sprintf(buff, "off: %6d,%6d,%6d", Gyro_offset[0], Gyro_offset[1], Gyro_offset[2]);
	  LCD_DrawString(0, 80, buff);
	  sprintf(buff, "mag : %6.0f,%6.0f,%6.0f", mag[0], mag[1], mag[2]);
	  LCD_DrawString(0, 100, buff);
//	  sprintf(gyro, "gyro: %4d,%4d,%4d", (int16_t)GyroData[0], (int16_t)GyroData[1], (int16_t)GyroData[2]);
//	  sprintf(counter, "xycoor: %8d", (int16_t)xycoor);
//	  LCD_DrawString(0,60, counter);
	  sprintf(_hitcount, "hitcount: %8d", (int16_t)hitcount);
	  LCD_DrawString(0,180, _hitcount);
//	  sprintf(acc1, "%8d", (int16_t)accg1);
//	  sprintf(acc2, "%8d", (int16_t)accg2);
//	  sprintf(acc3, "%8d", (int16_t)accg3);
//	  sprintf(gyro1, "%8d", (int16_t)GyroData[0]);
//	  sprintf(gyro2, "%8d", (int16_t)GyroData[1]);
//	  sprintf(gyro3, "%8d", (int16_t)GyroData[2]);
//	  sprintf(MagData1, "%8d", (int16_t)MagData[0]);
//	  sprintf(MagData2, "%8d", (int16_t)MagData[1]);
//	  sprintf(MagData3, "%8d", (int16_t)MagData[2]);
//	  sprintf(str,"%08d;%08d;%08d;%08d;%08d;%08d;%08d;%08d;%08d",
//			    (int16_t)AccData[0], (int16_t)AccData[1], (int16_t)AccData[2],
//			    (int16_t)GyroData[0], (int16_t)GyroData[1], (int16_t)GyroData[2],
//			    (int16_t)MagData[0], (int16_t)MagData[1], (int16_t)MagData[2]);
//	  sprintf(buff, "acc : %6.2f,%6.2f,%6.2f", AccData[0], AccData[1], AccData[2]);
//	  LCD_DrawString(0, 40, buff);


//	  LCD_DrawString(120,40, acc1);
//	  LCD_DrawString(120,60, acc2);
//	  LCD_DrawString(120,80, acc3);
//	  LCD_DrawString(120,100, gyro1);
//	  LCD_DrawString(120,120, gyro2);
//	  LCD_DrawString(120,140, gyro3);
//	  LCD_DrawString(120,160, MagData1);
//	  LCD_DrawString(120,180, MagData2);
//	  LCD_DrawString(120,200, MagData3);

//	  if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
//
//		  HAL_ADC_Start(&hadc1);
//		  HAL_ADC_PollForConversion(&hadc1, 1000);
//		  char str[10];
//		  uint32_t s_value = HAL_ADC_GetValue(&hadc1);
//		  sprintf(str,"%04lu   %04x" ,s_value, s_value);
//		  LCD_DrawString(120,40, str);
//
//		  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

#define numCalPoints_noclass 100
void imu_calibrateGyro_noclass (int16_t Gyro_offset[3]) {
	//Init
	int32_t sum[3] = {0, 0, 0};
	int16_t AccData[3], GyroData[3], MagData[3];

	// Save specified number of points, add them to sum (x,y,z)
	for (uint16_t i = 0; i < numCalPoints_noclass; i++) {
		HAL_Delay(3);
	    MPU9250_GetData(AccData, MagData, GyroData);
	    for (int j = 0; j < 3; j++) sum[j] += GyroData[j];
	}

	// Average the saved data points to find the gyroscope offset
	for (int j = 0; j < 3; j++) Gyro_offset[j] = (float) sum[j] / numCalPoints_noclass;

}

void imu_normalizeGyro (int16_t GyroData, int16_t Gyro_offset) {
		GyroData = (GyroData - Gyro_offset)/ 32.8;
//		if (GyroData[j] <= 0) {
//			GyroData[j] = 0;
//		}

}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim == &htim2)
//	{
//		timer_flag = 1;
//	}
//}
//
//void mpu9250_read_reg(uint8_t reg, uint8_t *data, uint8_t len)
//{
//	uint8_t temp_data = 0x80|reg;
//	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi2, &temp_data, 1, 100);
//	HAL_SPI_Receive(&hspi2,data, len, 100);
//	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
//}
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
