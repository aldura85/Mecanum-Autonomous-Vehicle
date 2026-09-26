/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
#include "NRF24_conf.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint16_t readValueX=0;
uint16_t readValueY=0;
uint16_t readValueX2=0;
uint16_t readValueY2=0;

uint8_t flag = 0;
uint8_t flag1 = 0;
uint8_t flag2 = 0;

uint8_t check=0;
uint8_t watch;
uint8_t receive_counter = 0;

uint32_t last_ticks = 0;

float pid_freq = 0;

uint32_t last_interrupt_time = 0;
uint32_t last_interrupt_time1 = 0;
uint32_t last_interrupt_time2 = 0;

#define ADC_BUFFER_SIZE 4
uint16_t adc_buffer[ADC_BUFFER_SIZE];  // X in [0], Y in [1]
uint16_t adc2_buffer[ADC_BUFFER_SIZE];

int i =0  ;
int y = 10;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define PLD_S 12
uint8_t rx_ack_pld[PLD_S] = {"OK"};
uint8_t tx_addr[5] = {0x45, 0x55, 0x67, 0x10, 0x21};

uint16_t data = 0;
uint8_t dataR[PLD_S];
uint8_t dataT[PLD_S];
volatile uint8_t irq = 0;

void read_joystick_values(void) {

	  readValueX = (int16_t)adc_buffer[0] ;  // Center: -2048 to +2047
		 	  readValueY = (int16_t)adc_buffer[1] ;
		 	  readValueX2=(int16_t)adc_buffer[2] ;
		 	 readValueY2=(int16_t)adc_buffer[3] ;
}

void prepare_data_packet(uint8_t toggle) {
    dataT[0] = toggle;
    dataT[1] = (readValueY >> 8) & 0xFF;  // High byte of Y
    dataT[2] = (readValueY & 0xFF);       // Low byte of Y
    dataT[3] = (readValueX >> 8) & 0xFF;  // High byte of X
    dataT[4] = (readValueX & 0xFF);       // Low byte of X
    dataT[5]= (readValueY2 >> 8) & 0xFF;  // High byte of X
    dataT[6]= (readValueY2 & 0xFF);       // Low byte of X
    dataT[7]= (readValueX2 >> 8) & 0xFF;
    dataT[8]= (readValueX2 & 0xFF);
    dataT[9]= flag;
    dataT[10]= flag1;
    dataT[11]= flag2;
}

void send_data(void) {
    nrf24_transmit(dataT, sizeof(dataT));
}

void transmitter(void){
	 nrf24_stop_listen();
	 nrf24_en_ack_pld(disable);
}
void nrf24_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2];
    data[0] = reg;  // Register address
    data[1] = value;  // Value to write

    // Start SPI communication
    HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
}


void reciver(void){

	nrf24_listen();
	nrf24_en_ack_pld(enable);
	nrf24_en_dyn_ack(disable);

}

void handle_irq(void) {

	if(nrf24_data_available()){
			nrf24_receive(dataR, sizeof(dataR));
		}

}


void conf(void){
	csn_high();
	  ce_high();

	  HAL_Delay(5);

	  ce_low();

	  nrf24_init();

	  nrf24_stop_listen();

	  nrf24_auto_ack_all(auto_ack);
	  nrf24_en_ack_pld(disable);
	  nrf24_dpl(disable);

	  nrf24_set_crc(no_crc, _1byte);

	  nrf24_tx_pwr(_0dbm);
	  nrf24_data_rate(_250kbps);
	  nrf24_set_channel(83);
	  nrf24_set_addr_width(5);

	  nrf24_set_rx_dpl(0, disable);
	  nrf24_set_rx_dpl(1, disable);
	  nrf24_set_rx_dpl(2, disable);
	  nrf24_set_rx_dpl(3, disable);
	  nrf24_set_rx_dpl(4, disable);
	  nrf24_set_rx_dpl(5, disable);

	  nrf24_pipe_pld_size(0, PLD_S);

	  nrf24_auto_retr_delay(4);
	  nrf24_auto_retr_limit(10);

	  nrf24_open_tx_pipe(tx_addr);
	  nrf24_open_rx_pipe(0, tx_addr);
	  ce_high();
}

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT access
    DWT->CYCCNT = 0;                                // Reset the counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Start the counter
}

float Measure_LoopRate(void) {
    static uint32_t last_cycle = 0;
    static uint32_t count = 0;
    static float rate = 0.0f;

    count++;
    uint32_t now = DWT->CYCCNT;

    // Calculate once every second
    if ((now - last_cycle) >= SystemCoreClock) {
        rate = (float)count * ((float)SystemCoreClock / (now - last_cycle));
        count = 0;
        last_cycle = now;
    }

    return rate;
}

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  conf();
      uint8_t toggle = 0; // Toggle variable

      DWT_Init();



      if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE) != HAL_OK) {
               Error_Handler();
           }
      /*if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, ADC_BUFFER_SIZE) != HAL_OK) {
                  Error_Handler();
              }*/

      transmitter();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
 i++;
 y+=10;
 pid_freq = Measure_LoopRate();


	 read_joystick_values();
	      prepare_data_packet(toggle);
	      send_data();
	      toggle = !toggle;
	      check=flag;
	      if (i>200){
	    	  i=0;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, CSN_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CSN_Pin CE_Pin */
  GPIO_InitStruct.Pin = CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0){
			irq = 1;
		}
	if (GPIO_Pin == GPIO_PIN_15) // Check if the interrup t was from PB13
	    {
	    	uint32_t current_time = HAL_GetTick(); // milliseconds
	    	        if (current_time - last_interrupt_time > 200) // 50 ms debounce time
	    	        {
	    	            flag1 = !flag1;
	    	            last_interrupt_time = current_time;
	    	        }
	    }

if (GPIO_Pin == GPIO_PIN_14)
    {
    	uint32_t current_time1 = HAL_GetTick(); // milliseconds
    	        if (current_time1 - last_interrupt_time2 > 200) // 50 ms debounce time
    	        {
    	            flag2 = !flag2;
    	            last_interrupt_time2 = current_time1;
    	        }
    }


    if (GPIO_Pin == GPIO_PIN_13)
    {
    	uint32_t current_time2 = HAL_GetTick(); // milliseconds
    	        if (current_time2 - last_interrupt_time1 > 200) // 50 ms debounce time
    	        {
    	            flag = !flag;
    	            last_interrupt_time1 = current_time2;
    	        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
		if (hadc->Instance == ADC1) {
			// Optional: Flag for processing, debounce, etc.
			// Buffer is fresh!
		}
		if (hadc->Instance == ADC2) {
					// Optional: Flag for processing, debounce, etc.
					// Buffer is fresh!
				}
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
