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
extern "C" {
#include "fatfs_sd.h"
#include "wavlib.h"
}

#define ARM_MATH_CM7
#include "arm_math.h"

#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "speech.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_FULL_ASSERT
#define ADC_BUFFER_SIZE 1
#define I2S_BUFFER_SIZE 4
#define BLOCK_SIZE		1
#define NUM_SECONDS 3
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

* 0 Hz - 100 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.114878445068044 dB

* 200 Hz - 600 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 4.118996003728048 dB

* 700 Hz - 1000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.114878445068044 dB

*/

#define FILTER_TAP_NUM 25

static double filter_taps[FILTER_TAP_NUM] = {
  -0.011345656581760597,
  -0.0032228001375654166,
  -0.002785122852201937,
  -0.0009610245471522801,
  0.06507058198859063,
  0.08411301595113971,
  -0.04228657823330367,
  -0.07378384638021954,
  0.012887112852679112,
  -0.1314165755227072,
  -0.2564496623495336,
  0.13020553667522836,
  0.46981865035105913,
  0.13020553667522836,
  -0.2564496623495336,
  -0.1314165755227072,
  0.012887112852679112,
  -0.07378384638021954,
  -0.04228657823330367,
  0.08411301595113971,
  0.06507058198859063,
  -0.0009610245471522801,
  -0.002785122852201937,
  -0.0032228001375654166,
  -0.011345656581760597
};


// TENSORFLOW DECLARATIONS
// TFLite globals
namespace {
tflite::ErrorReporter *error_reporter = nullptr;
const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *model_input = nullptr;
TfLiteTensor *model_output = nullptr;

static tflite::MicroErrorReporter micro_error_reporter;

static tflite::MicroMutableOpResolver<5> micro_op_resolver;
// Create an area of memory to use for input, output, and other TensorFlow
// arrays. You'll need to adjust this by compiling, running, and looking
// for errors.
constexpr int kTensorArenaSize = 70 * 1024;
__attribute__((aligned(16))) uint8_t tensor_arena[kTensorArenaSize];
}

TfLiteStatus tflite_status;
uint32_t num_elements;
uint32_t timestamp;
int8_t y_val;

// FIR DECLARATIONS
arm_fir_instance_f32 FilterSettings;
static float32_t firState[BLOCK_SIZE + FILTER_TAP_NUM - 1];
uint32_t blockSize = BLOCK_SIZE;

float32_t signal_in = 0;
float32_t signal_out = 0;

// FFT DECLARATIONS
arm_rfft_fast_instance_f32 FFTSettings;
float32_t fft_buffer[512];
float32_t fft_out[512];
arm_status status;

// FATFS_SD FILE SYSTEM DECLARATIONS
uint16_t raw;

FATFS fs;
FIL fil;
FRESULT fresult;
UINT br, bw;

FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

wavfile_header_t header;
PCM16_stereo_t sample;

char filename[10];
int file_count = 0;

// DMA ADC

int IS_RECORDING = 0;

int32_t fc = NUM_SECONDS * SAMPLE_RATE;
int32_t dc = -1;

uint16_t buffer[NUM_SECONDS * SAMPLE_RATE];
volatile uint16_t adc_buff[ADC_BUFFER_SIZE];

// I2S DECLARATIONS
uint16_t i2s_buff[NUM_SECONDS * SAMPLE_RATE];
int32_t tx_frame_count = 0;

extern "C" {
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
	void SystemClock_Config(void);
	void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
}

void setupTFLM();
void runInference();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern "C" {
	void initialise_monitor_handles(void);
}

uint16_t to_u16(uint16_t value);
void closeFile();
void startRecording();
void record();
void play_feedback();
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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

	// Mount SD Card
	fresult = f_mount(&fs, "", 1);
	if (fresult != FR_OK)
		printf("██ Error Mounting SD Card | Code: %d\n", fresult);
	else
		printf("┣━ SD Card Mounted Successfully...\n");

	f_getfree("", &fre_clust, &pfs);
	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	printf("┃  ┣━ Total Size: %lu MiB\n", total / 1024);
	printf("┃  ┗━ Free Space: %lu MiB\n", free_space / 1024);

	setupTFLM();

	//  FIR INIT
//	arm_fir_init_f32(&FilterSettings, FILTER_TAP_NUM,
//			(float32_t*) &filter_taps[0], (float32_t*) &firState[0], blockSize);
//	printf("┣━ FIR Band Pass Filter Initialised...\n");
//	status = arm_cfft_radix2_init_f32(&FFTSettings, 512, 0, 1);
	for (int i = 0; i < 512; i++) {
		fft_buffer[i] = 0;
	}
//	if (status == ARM_MATH_SUCCESS)
//		printf("┣━ FFT Init Complete...\n");
//	else
//		printf("██ FFT Init Error | Code: %d\n", status);

//	setupTFLM();
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize(); /* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3
			| RCC_PERIPHCLK_I2S | RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
	PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
	PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
	PeriphClkInitStruct.PLLI2SDivQ = 1;
	PeriphClkInitStruct.I2sClockSelection = RCC_I2SCLKSOURCE_PLLI2S;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void StartDefaultTask(void *argument){
	for(;;) {
		osDelay(3000);
	}
}

void StartAudioInputTask(void *argument){
	for(;;) {
		osDelay(3000);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (dc < 0) {
		dc++;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == GPIO_PIN_13 && IS_RECORDING == 0) // If The INT Source Is EXTI Line9 (A9 Pin)
			{
		startRecording();
	}
}

//void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
//	if (tx_frame_count < NUM_SECONDS * SAMPLE_RATE * 2) {
//		i2s_buff[0] = buffer[tx_frame_count];
//		i2s_buff[1] = buffer[tx_frame_count];
//		tx_frame_count++;
//	}
//}

//void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
//	if (tx_frame_count < NUM_SECONDS * SAMPLE_RATE * 2) {
//		i2s_buff[2] = buffer[tx_frame_count];
//		i2s_buff[3] = buffer[tx_frame_count];
//		tx_frame_count++;
//	} else {
//		HAL_I2S_DMAStop(&hi2s2);
//		printf("┣━ Feedback Playback Stop\n");
//	}
//}

void startRecording() {
	printf("┣━ Start Record Interrupt Received...\r\n");
//	printf("┃  ┣━ Starting WAV Write...\r\n");
//	sprintf(filename, "rec%d.wav", file_count);
//	f_open(&fil, filename, FA_OPEN_ALWAYS | FA_WRITE);
//	header = get_PCM16_stereo_header(SAMPLE_RATE, fc);
//	fresult = f_write(&fil, &header, sizeof(wavfile_header_t), &bw);
//	if (fresult != FR_OK)
//		printf("Error Writing Header to SD Card | Code: %d\n", fresult);
//	else
//		printf("┃  ┣━ Wrote WAV Header to File\n");
//	f_sync(&fil);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	IS_RECORDING = 1;
	printf("┃  ┣━ ADC Started. Recording...\r\n");
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buff, ADC_BUFFER_SIZE);
}

void record() {
	if (dc > -1 && IS_RECORDING == 1) {
		if (dc < fc) {
//			signal_in = (float32_t)(adc_buff[0] < 1975 || adc_buff[0] > 2125) ?
//					to_u16(adc_buff[0]) : 0;
//			arm_fir_f32(&FilterSettings, &signal_in, &signal_out, blockSize);
			buffer[dc - 1] =
					(adc_buff[0] < 1975 || adc_buff[0] > 2125) ?
							to_u16(adc_buff[0]) : 0;
			dc++;
		} else {
			HAL_TIM_Base_Stop_IT(&htim7);
			HAL_ADC_Stop_DMA(&hadc1);
			printf("┃  ┣━ ADC Halted.\n");
//			play_feedback();
//			printf("┃  ┣━ Start buffer dump to SD Card.\n");
//			for (int i = 0; i < fc; i++) {
//				sample.left = buffer[i];
//				sample.right = sample.left;
//				fresult = f_write(&fil, &sample, sizeof(PCM16_stereo_t), &bw);
//				if (fresult != FR_OK)
//					printf("Error Writing to SD Card | Code: %d\n", fresult);
//			}
//			printf("┃  ┣━ %s Dump Write Complete.\n", filename);
//			closeFile();
			runInference();
			dc = 0;
//			file_count++;
			IS_RECORDING = 0;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		}
	}
}

uint16_t to_u16(uint16_t value) {
//	return ((value << 4) & (uint16_t)0xFFF0) | ((value >> 8) & (uint16_t)0x000F);
	return value << 4;
}

void closeFile() {
	fresult = f_close(&fil);
	if (fresult != FR_OK)
		printf("Error Closing File | Code: %d\n", fresult);
	else
		printf("┃  ┗━ WAV File Write Success.\n");
	return;
}

void play_feedback() {
	for (int i = 0; i < NUM_SECONDS * SAMPLE_RATE/2; i++) {
		i2s_buff[i*2] = buffer[i];
		i2s_buff[i*2+1] = buffer[i];
//		tx_frame_count++;
	}
	printf("┣━ Feedback Playback Start\n");
	if (HAL_I2S_Transmit_DMA(&hi2s2, i2s_buff, NUM_SECONDS * SAMPLE_RATE) != HAL_OK) {
		printf("I2S Tx Error\n");
		Error_Handler();
	}
}

void setupTFLM() {
	error_reporter = &micro_error_reporter;

	// Map the model into a usable data structure
	model = tflite::GetModel(speech_model);
	if (model->version() != TFLITE_SCHEMA_VERSION) {
		printf("┣━ [TFLM] Model version does not match Schema\n");
	} else {
		printf("┣━ [TFLM] Model Loaded Successfully\n");
	}

	// Add neural network layer operations
	tflite_status = micro_op_resolver.AddResizeBilinear();
	if (tflite_status != kTfLiteOk) {
		printf("┣━ [TFLM] Could not add RESIZE_BILINEAR layer\n");
	}
	tflite_status = micro_op_resolver.AddReshape();
	if (tflite_status != kTfLiteOk) {
		printf("┣━ [TFLM] Could not add RESHAPE layer\n");
	}
	tflite_status = micro_op_resolver.AddConv2D();
	if (tflite_status != kTfLiteOk) {
		printf("┣━ [TFLM] Could not add CONV_2D layer\n");
	}
	tflite_status = micro_op_resolver.AddMaxPool2D();
	if (tflite_status != kTfLiteOk) {
		printf("┣━ [TFLM] Could not add MAX_POOL_2D layer\n");
	}
	tflite_status = micro_op_resolver.AddFullyConnected();
	if (tflite_status != kTfLiteOk) {
		printf("┣━ [TFLM] Could not add FULLY_CONNECTED layer\n");
	}

	static tflite::MicroInterpreter static_interpreter(model, micro_op_resolver,
			tensor_arena, kTensorArenaSize, error_reporter);
	interpreter = &static_interpreter;

	// Allocate memory from the tensor_arena for the model's tensors.
	tflite_status = interpreter->AllocateTensors();
	if (tflite_status != kTfLiteOk) {
		printf("┣━ [TFLM] AllocateTensors() failed");
	}

	// Assign model input and output buffers (tensors) to pointers
	model_input = interpreter->input(0);
	model_output = interpreter->output(0);

	// Get number of elements in input tensor
	num_elements = model_input->bytes / sizeof(uint8_t);
	printf("┣━ [TFLM] Number of input elements: %lu\r\n", num_elements);
	printf("┣━ [TFLM] Setup Complete\n");
}

void runInference() {
	printf("┃  ┣━ Starting Inference...\n");
	// Fill input buffer (use test value)
	for (uint32_t i = 0; i < 100; i++) {
		if (i == 0) {
			for (int j = 0; j < 480; j++) {
				fft_buffer[j] = buffer[j];
			}
		} else {
			for (uint16_t j = i*480-160, k = 0; k < 480; j++, k++) {
				fft_buffer[k] = buffer[j];
			}
		}
		status = arm_rfft_fast_init_f32(&FFTSettings, 512);
		if (status != ARM_MATH_SUCCESS)
			printf("██ FFT Init Error | Code: %d\n", status);
		arm_rfft_fast_f32(&FFTSettings, fft_buffer, fft_out, 0);
		for(int j = 0; j < 80; j++) {
			int8_t sum = 0;
			for(int k = 0; k < 6; k++) {
				sum += fft_out[j*6+k];
			}
			model_input->data.int8[j*80+i] = sum/6;
		}
	}
	printf("┃  ┣━ Spectrogram Generated\n");

	// Run inference
	tflite_status = interpreter->Invoke();
	if (tflite_status != kTfLiteOk) {
		error_reporter->Report("Invoke failed");
	}

	y_val = model_output->data.int8[0] >= model_output->data.int8[1] ? 0 : 1;

	// Print output of neural network along with inference time (microseconds)
	printf("┃  ┗━ [TFLM] Output: %s, Confidence [Off/On]:%d/%d\r\n", y_val == 0 ? "off" : "on", model_output->data.int8[0], model_output->data.int8[1]);
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM7) {
		record();
	}
	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	printf("Error: file %s on line %lu\r\n", file, line);
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
