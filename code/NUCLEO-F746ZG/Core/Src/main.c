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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "fatfs_sd.h"
#include "wavlib.h"

#define ARM_MATH_CM7
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_FULL_ASSERT
#define ADC_BUFFER_SIZE 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 2000 Hz

* 0 Hz - 400 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 4.1393894966071585 dB

* 500 Hz - 1000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.07355419274887 dB

*/

#define FILTER_TAP_NUM	29
#define BLOCK_SIZE		1

static float32_t filter_taps[FILTER_TAP_NUM] = {

		-0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f,


	    +0.0085302217f, -0.0000000000f, -0.0173976984f, -0.0341458607f, -0.0333591565f,


	    +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f,


	    +0.2229246956f, +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f,


	    -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f, +0.0080754303f,


	    +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f};

// FIR DECLARATIONS
arm_fir_instance_f32 FilterSettings;
static float32_t firState[BLOCK_SIZE + FILTER_TAP_NUM - 1];
uint32_t blockSize = BLOCK_SIZE;

float32_t signal_in = 0;
float32_t signal_out = 0;

// DMA AND SD FILE SYSTEM DECLARATIONS
volatile uint16_t adc_buff[ADC_BUFFER_SIZE];

uint16_t raw;

FATFS fs;
FIL fil;
FRESULT fresult;
UINT br, bw;

FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

int32_t fc = 5 * SAMPLE_RATE;
int32_t dc = 0;

wavfile_header_t header;
PCM16_stereo_t sample;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void initialise_monitor_handles(void);

uint16_t to_u16(uint16_t value) {
	return ((value << 4) & (uint16_t)0xFFF0) | ((value >> 8) & (uint16_t)0x000F);
}

void closeFile()
{
  fresult = f_close(&fil);
  if (fresult != FR_OK)
    printf("Error Closing File | Code: %d\n", fresult);
  else
    printf("┗━ WAV File Write Success.\n");
  return;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  initialise_monitor_handles();
  printf("┏━ Boot Complete...\n");
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_I2S2_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  printf("┣━ Firmware Init Complete...\n");

//  FIR INIT
  arm_fir_init_f32(&FilterSettings, FILTER_TAP_NUM, (float32_t *)&filter_taps[0], (float32_t *)&firState[0], blockSize);
  printf("┣━ FIR Low Pass Filter Init Complete...\n");

  // Mount SD Card
  fresult = f_mount(&fs, "", 1);
  if (fresult != FR_OK)
    printf("██ Error Mounting SD Card | Code: %d\n", fresult);
  else
    printf("┣━ SD Card Mounted Successfully...\n");

  f_getfree("", &fre_clust, &pfs);
  total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
  free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
  printf("┃  ┣━ Total Size: %lu MiB\n", total/1024);
  printf("┃  ┗━ Free Space: %lu MiB\n", free_space/1024);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2S
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sClockSelection = RCC_I2SCLKSOURCE_PLLI2S;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void StartDefaultTask(void *argument) {
	for(;;) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
		osDelay(500);
	}
}

void StartAudioInputTask(void *argument) {
	printf("┣━ Microphone ADC-DMA Channel Setup Complete...\n");
	printf("┃  ┣━ Starting WAV Write...\r\n");
	f_open(&fil, "htim8k.wav", FA_OPEN_ALWAYS | FA_WRITE);

	header = get_PCM16_stereo_header(SAMPLE_RATE, fc);
	fresult = f_write(&fil, &header, sizeof(wavfile_header_t), &bw);
	if (fresult != FR_OK)
	printf("Error Writing Header to SD Card | Code: %d\n", fresult);
	else
	printf("┃  ┃  ┗━ Wrote WAV Header to File\n");
	f_sync(&fil);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buff, ADC_BUFFER_SIZE);
	for(;;) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		osDelay(500);
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
//  if (dc < fc)
//  {
//    for (int i = 0; i < ADC_BUFFER_SIZE / 2; i++)
//    {
		signal_in = (float32_t)to_u16(adc_buff[0]);
		arm_fir_f32(&FilterSettings, &signal_in, &signal_out, blockSize);

      sample.left = (uint16_t)signal_out;
      sample.right = sample.left;
//      dc++;
//      fresult = f_write(&fil, &sample, sizeof(PCM16_stereo_t), &bw);
//      if (fresult != FR_OK)
//        printf("Error Writing to SD Card | Code: %d\n", fresult);
//    }
//  }
//  else
//  {
//    HAL_ADC_Stop_DMA(&hadc1);
//    printf("┃  ┗━ Write Complete. ADC Halted.\n");
//    closeFile();
//  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
//  if (dc < fc)
//  {
//    for (int i = ADC_BUFFER_SIZE / 2; i < ADC_BUFFER_SIZE; i++)
//    {
		signal_in = (float32_t)to_u16(adc_buff[1]);
		arm_fir_f32(&FilterSettings, &signal_in, &signal_out, blockSize);

	  sample.left = (uint16_t)signal_out;
	  sample.right = sample.left;
//      dc++;
//      fresult = f_write(&fil, &sample, sizeof(PCM16_stereo_t), &bw);
//      if (fresult != FR_OK)
//        printf("Error Writing to SD Card | Code: %d\n", fresult);
//    }
//  }
//  else
//  {
//    HAL_ADC_Stop_DMA(&hadc1);
//    printf("┃  ┗━ Write Complete. ADC Halted.\n");
//    closeFile();
//  }
}
/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM7) {
	  if (dc < fc) {
		  dc++;
		  fresult = f_write(&fil, &sample, sizeof(PCM16_stereo_t), &bw);
		  if (fresult != FR_OK)
			  printf("Error Writing to SD Card | Code: %d\n", fresult);
	  } else {
		  	  HAL_TIM_Base_Stop_IT(&htim7);
		      HAL_ADC_Stop_DMA(&hadc1);
		      printf("┃  ┗━ Write Complete. ADC Halted.\n");
		      closeFile();
	  }
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
	printf("Error: file %s on line %lu\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
