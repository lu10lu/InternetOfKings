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
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int Coordinaat[] = { 0, 0};
int direction[] = {0, 1};
int CoordinaatX[3] ={5,-6,2};
int CoordinaatY[3]= {3,5,-1};
extern UART_HandleTypeDef huart1;
extern volatile uint32_t bufferIndex;
extern char receivedBuffer[100];
extern float receivedTemperature;  // Variable to store the received temperature
uint8_t Rx_data[18];
#define SHT40_MEAS_HIGH_PRECISION 0xFD
#define SHT40_I2C_ADDRESS 0x44 << 1 // SHT40 I2C address
#define SHT40_MEASURE_DELAY_MS 10 // Measurement delay for high precision
#define LTR329_I2C_ADDRESS 0x29 << 1  // Shifted for 8-bit format
#define MAX_BLE_MESSAGE_SIZE 100

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void myCAllback(UART_HandleTypeDef *huart);

#define LTR329_ALS_CONTR 0x80
#define LTR329_ALS_MEAS_RATE 0x85
#define LTR329_PART_ID 0x86
#define LTR329_MANUFAC_ID 0x87
#define LTR329_ALS_DATA_CH1_0 0x88
#define LTR329_ALS_DATA_CH1_1 0x89
#define LTR329_ALS_DATA_CH0_0 0x8A
#define LTR329_ALS_DATA_CH0_1 0x8B

#define LTR329_ACTIVE_MODE 0x01
#define LTR329_MEAS_RATE 0x03  // Example: 100ms integration time
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
volatile uint8_t justWokenUpFromStop2 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void StartDefaultTask(void *argument);

void SendUARTMessage(const char* message) {
  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}
void BLEUARTMessage(const char* message){
  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

void Forward() {
	  HAL_Init();

	char message[100]= {};

    int distance = 10  ;  // Create a character array to hold the message
    int speed =  200;
    // Format the message with the current LED value
    sprintf(message, "drvstr %d %d\r\n", distance, speed);

    // Send the message over UART
    SendUARTMessage(message);


}
void Right(){

	char message[100]= {};

    int angle = 45  ;  // Create a character array to hold the message
    int speed =  200;
    // Format the message with the current LED value
    sprintf(message, "rotctr %d %d\r\n", angle, speed);

    // Send the message over UART
    SendUARTMessage(message);


}
void Left(){

	char message[100]= {};

    int angle = -45  ;  // Create a character array to hold the message
    int speed =  200;
    // Format the message with the current LED value
    sprintf(message, "rotctr %d %d\r\n", angle, speed);

    // Send the message over UART
    SendUARTMessage(message);


}

void Navigation(int destinationX, int destinationY){
	char message[100]= {};
    //sprintf(message, "Inside %u %u\r\n", destinationX, destinationY);

    //SendUARTMessage(message);
	int deltaX = destinationX - Coordinaat[0];
	int deltaY = destinationY - Coordinaat[1];
	while( deltaX != 0){
	if (deltaX > 0){
		if(direction[0] == 0){
			if(direction[1] == 1){
				Right();
				direction[0] = 1;
				direction[1] = 0;
			    //sprintf(message, "0 draai rechts\r\n");

			    // Send the message over UART
			    //SendUARTMessage(message);

			}else {
				Left();
				direction[0] = 1;
				direction[1] = 0;
			    //sprintf(message, "0 draai links\r\n");

			    // Send the message over UART
			    //SendUARTMessage(message);
			}

		}else{
			if(direction[1] ==-1 ){
				Left();
				Left();
				direction[0] = 1;
				direction[1] = 0;
			    //sprintf(message, "0 2x links\r\n");

			    // Send the message over UART
			    //SendUARTMessage(message);
			}
		}
	}
	else if (deltaX < 0){
		if(direction[0] == 0){
			if(direction[1] == 1){
				Left();
				direction[0] = -1;
				direction[1] = 0;
			    //sprintf(message, "1 draai links\r\n");
			    //SendUARTMessage(message);


			}else {
				Right();
				direction[0] = -1;
				direction[1] = 0;
			    //sprintf(message, "1 draai rechts\r\n");
			    //SendUARTMessage(message);

			}

		}else{
			if(direction[1] ==1 ){
				Left();
				Left();
				direction[0] = -1;
				direction[1] = 0;
			    //sprintf(message, "1 2xlinks\r\n");
			    //SendUARTMessage(message);

			}
		}
	}

	for(int i = 0; i < abs(deltaX); i++){
		Forward();
		Coordinaat[0] += direction[0];
	    //sprintf(message, "forward X %d %d\r\n", Coordinaat[0], Coordinaat[1]);

	    // Send the message over UART
	    //SendUARTMessage(message);
	}

	deltaX = destinationX - Coordinaat[0];
	}
	while(deltaY != 0){
		if(deltaY > 0){
			if (direction[0] == 1){
				Left();
				direction[0] = 0;
				direction[1] = 1;
				//sprintf(message, "Y draai links\r\n");
				//SendUARTMessage(message);

			} else if (direction[0] == -1){
				Right();
				direction[0] = 0;
				direction[1] = 1;
				//sprintf(message, "Y draai rechts\r\n");
				//SendUARTMessage(message);

			}else{
				if (direction[1] == -1){
					Right();
					Right();
					direction[0] = 0;
					direction[1] = 1;
					//sprintf(message, "Y 2xrechts\r\n");
					//SendUARTMessage(message);
				}
			}

		}
		if(deltaY < 0){
			if (direction[0] == 1){
				Right();
				direction[0] = 0;
				direction[1] = -1;
				//sprintf(message, "Y draai links\r\n");
				//SendUARTMessage(message);

			} else if (direction[0] == -1){
				Left();
				direction[0] = 0;
				direction[1] = -1;
				//sprintf(message, "Y draai rechts\r\n");
				//SendUARTMessage(message);

			}else{
				if (direction[1] == 1){
					Right();
					Right();
					direction[0] = 0;
					direction[1] = -1;
					//sprintf(message, "Y 2xrechts\r\n");
					//SendUARTMessage(message);
				}
			}

		}
		for(int i = 0; i < abs(deltaY); i++){
			Forward();
			Coordinaat[1] += direction[1];
			//sprintf(message, "forward Y %d %d\r\n", Coordinaat[0], Coordinaat[1]);

			// Send the message over UART
			//SendUARTMessage(message);
		}
		deltaY = destinationY - Coordinaat[1];

	}
}
void SHT40_Init()
{
	HAL_I2C_Init(&hi2c3); // Initialize and configure the I2C peripheral
}
void SHT40_Read(float *t, float *rh, uint8_t mode)
{
	uint8_t data[2] = {mode, 0x00};
	uint8_t buffer[6];

    HAL_I2C_Master_Transmit(&hi2c3, SHT40_I2C_ADDRESS, data, 2, HAL_MAX_DELAY);
    HAL_Delay(10);
    HAL_I2C_Master_Receive(&hi2c3, SHT40_I2C_ADDRESS, buffer, 6, HAL_MAX_DELAY);

    uint16_t t_ticks = buffer[0]*256 + buffer[1];
    uint16_t checksum_t = buffer[2];
    uint16_t rh_ticks = buffer[3]*256 + buffer[4];
    uint16_t checksum_rh = buffer[5];

    float t_degC = -45 + 175.0 * t_ticks/65535;
    float rh_pRH = -6 + 125.0 * rh_ticks/65535;

    if (rh_pRH > 100){
    	rh_pRH = 100; }
    if (rh_pRH < 0){
    	rh_pRH = 0; }

    *t = t_degC;
    *rh = rh_pRH;
}


void LTR329_Init()
{
    uint8_t data;

    // Activate the sensor
    data = LTR329_ACTIVE_MODE;
    HAL_I2C_Mem_Write(&hi2c3, LTR329_I2C_ADDRESS, LTR329_ALS_CONTR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    // Set measurement rate
    data = LTR329_MEAS_RATE;
    HAL_I2C_Mem_Write(&hi2c3, LTR329_I2C_ADDRESS, LTR329_ALS_MEAS_RATE, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}
void LTR329_Read_Light(uint16_t *ch0, uint16_t *ch1)
{
    uint8_t data[4];

    // Read 4 bytes of data starting from LTR329_ALS_DATA_CH1_0
    HAL_I2C_Mem_Read(&hi2c3, LTR329_I2C_ADDRESS, LTR329_ALS_DATA_CH1_0, I2C_MEMADD_SIZE_8BIT, data, 4, 100);

    // Combine bytes to get the light data for each channel
    *ch1 = (uint16_t)(data[1] << 8) | data[0];
    *ch0 = (uint16_t)(data[3] << 8) | data[2];
}




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCplCallback(UART_HandleTypeDef *huart){
    char message[] = "Received data from BLE module!\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

    // Restart reception for continuous data reception
    HAL_UART_Receive_IT(&huart1, Rx_data, sizeof(Rx_data));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


	  MX_GPIO_Init();
	  MX_USART2_UART_Init();
	  float t,rh;
	  uint16_t light_ch0, light_ch1;
	  char buffer[50];  // A buffer for formatting the data


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();

  /* USER CODE BEGIN Init */
	  HAL_Init();

	  SystemClock_Config();  /* USER CODE END Init */

  /* Configure the system clock */
  //SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  /* Enable USART1 interrupt */
  //__enable_irq();
  //HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  //HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_UART_Receive_IT(&huart1, Rx_data, 18);
  LTR329_Init();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  char message[100]= {};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	for(int j = 0; j<3; j++){
	//interrupt for rover
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	//HAL_Delay(5);

    Navigation(CoordinaatX[j], CoordinaatY[j]);
	//SendUARTMessage("sleep");
  	char message[100];

	  for(int i = 0; i <3; i++){
	  SHT40_Read(&t, &rh, SHT40_MEAS_HIGH_PRECISION);
	  LTR329_Read_Light(&light_ch0, &light_ch1);
      HAL_Delay(10000);  // Delay for a second

      //SLEEPmode
     //HAL_SuspendTick();
     //HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x4E20, RTC_WAKEUPCLOCK_RTCCLK_DIV16, RTC_WAKEUPCLOCK_RTCCLK_DIV16); // value is 2000 decimal to hexa AND 2000 = ((32000/16)*10sec)
     // Enter STOP 2 mode
     //HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI); // entering stop 2 mode
     //HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);     // deactivating wake-up timer
	 //SystemClock_Config();
	 //HAL_ResumeTick();

      //VisibleLight[i] = light_ch0;
      //Ir[i] = light_ch1;
	  sprintf(message,"[%0.2f; %0.2f; %u; %u]\n ",t,rh, light_ch0, light_ch1);

	  BLEUARTMessage(message);
	  SendUARTMessage(message);
	  memset(message, 0, sizeof(message));

	  }

  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00000E14;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_UART_Receive_IT(&huart1, Rx_data, sizeof(Rx_data));

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
