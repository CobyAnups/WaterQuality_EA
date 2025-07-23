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
	  *
	  *
	  ******************************************************************************
	  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "FreeRTOS.h"
	#include "task.h"
	#include "event_groups.h"
	#include "queue.h"
	#include "semphr.h"
	#include "timers.h"

	#include "ds18b20.h"
	#include "File_Handling_RTOS.h"

	/* C Standard Libraries */
	#include <stdio.h>
	#include <string.h>
	#include <stdbool.h>
	#include <stdint.h>
	#include <inttypes.h>
	#include <math.h>
	#include <time.h>
	#include <stdlib.h>

	extern ADC_HandleTypeDef hadc1;
	extern UART_HandleTypeDef huart2;
	extern UART_HandleTypeDef huart1;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	#define UART_BUFFER_SIZE 64  // Adjust based on expected UART message length
	#define DEFAULT_SLEEP_TIME 30  // seconds — TODO: change to 1 hour (3600) before deployment

	#define PUMP1        			GPIOC, GPIO_PIN_3
	#define PUMP3       			GPIOA, GPIO_PIN_15
	#define PUMP2         			GPIOC, GPIO_PIN_0
	#define pH_POWER     			GPIOC, GPIO_PIN_4
	#define Temp_POWER      		GPIOC, GPIO_PIN_5

	#define EA_RELAY_GPIO           GPIOC, GPIO_PIN_12

	#define PUMP1_DURATION     		45000
	#define SENSOR_ON_DURATION_MS   3000
	#define PUMP2_DURATION     		45000
	#define PUMP3_DURATION        	5000

	#define BKP_REG_SYNC_FLAG RTC_BKP_DR1
	#define RTC_SYNC_MAGIC    0x32F2  // Arbitrary unique value

	#define MAX_RETRIES 5
	#define ACK_TIMEOUT_MS 60000  // 1 minute
	#define ACK_BUFFER_SIZE 100

	// EA Threshold Levels (can be tuned or loaded from config later)
	#define LEVEL_0 1000
	#define LEVEL_1 2000
	#define LEVEL_2 3000
	#define LEVEL_3 4000
	#define LEVEL_4 5000
	#define LEVEL_5 6000



	#define ACK_TIMEOUT_MS 60000  // 1 minute

	#define MAX_READINGS 3
	typedef struct {

	    char timestamp[25];   // ISO string
	    float temp;
	    float ph;
	    float do_val;
	    int batt;
	    uint8_t index;
	} SensorReading;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
	/* RTOS Task Handles */
	TaskHandle_t xMainTask;
	TaskHandle_t RcvTask;
	TaskHandle_t xBoot;
	TaskHandle_t xTX_Task;
	/* RTOS Synchronization Objects */

	SemaphoreHandle_t xLowPowerSemaphore;
	SemaphoreHandle_t xRtcUpdateSemaphore;
	SemaphoreHandle_t xAckSemaphore;
	SemaphoreHandle_t xMainTaskSemaphore;
	/* Sensor Data (Raw + Processed) */
	uint32_t adc_raw_ph = 0, adc_raw_do = 0;
	uint32_t ph_v = 0, ph_cv = 0;
	uint32_t do_v = 0, do_cv = 0;

	float temp = 0.0f;
	float ph_scaled = 0.0f, do_scaled = 0.0f;

	/* Battery Monitoring */
	uint16_t Batt_raw = 0;

	/* Sleep Time (EA behavior control) */
	uint16_t sleep_time = DEFAULT_SLEEP_TIME;

	/* EA Threshold Levels (can be tuned or loaded from config later) */
	int16_t Level_0 = 1000, Level_1 = 2000, Level_2 = 3000;
	int16_t Level_3 = 4000, Level_4 = 5000, Level_5 = 6000;

	/* UART RX Variables */
	char gps_buffer[256] = {0};  // Buffer for GPS data
	char uart_rx_buffer[UART_BUFFER_SIZE] = {0};
	char uart_rx_char = 0;
	uint16_t uart_rx_index = 0;
	uint8_t newline_count = 0;
	char uart_buf[128] = {0};  // General TX buffer (printf, debug)
	volatile bool uartWakeupFlag = false;

	uint32_t Read_ADC_Channel(uint32_t channel);
	bool ack_received = false;

	int sleep_lvl = 0; // Sleep level for EA behavior control

	SensorReading readings[6] = {0};  // Array to hold sensor readings
	uint16_t reading_ids[6];  // Parallel array to track global_id per reading
	uint8_t current_index = 0;  // increments after each set
	uint16_t global_id = 0;
	uint8_t resend_indexes[6] = {0};

	volatile bool resend_requested = false;
	volatile uint8_t tx_type_override = 0;  // 0 = type:0 (normal), 3 = type:3 (resend)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
	static void ParseGPSAndSetRTC(void);
	static void MainTask_handle(void*parameters);
	static void Receive_Task(void* parameters);
	static void BootTask(void *params);
	static void get_timestamp(char* buffer);
	static void PrintGPSTimeFormatted(void);
//	static void WaitForACKOrRetry(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	void MainTask_handle(void*parameters)
	{
		while(1)
		{
			if (xSemaphoreTake(xMainTaskSemaphore, portMAX_DELAY) == pdTRUE)
			{
//				// Reset the semaphore for next use
//				xSemaphoreTake(xMainTaskSemaphore, 0);
//
//				// Initialize variables
//				uint32_t Batt_raw = 0;
//				uint16_t sleep_time = DEFAULT_SLEEP_TIME;
//
//				// Initialize readings array
//				memset(readings, 0, sizeof(readings));
//				memset(reading_ids, 0, sizeof(reading_ids));
//				current_index = 0;
//				global_id = 0;

					// Print start message
				printf("Program Start \n");


				//Turn Pin LOW PC12  | EA GPIO RELAY MODULE
				HAL_GPIO_WritePin(EA_RELAY_GPIO , GPIO_PIN_RESET);  // PC12: LOW
				vTaskDelay(pdMS_TO_TICKS(200));


				//READ BATTERY LEVEL
				//Batt_raw = Read_ADC_Channel(ADC_CHANNEL_0); //TODO CHANGE
				Batt_raw = 2000;
				HAL_GPIO_WritePin(EA_RELAY_GPIO , GPIO_PIN_SET);


				//INSERT TABLE BELOW
				if (Batt_raw < Level_0 )// 0-10%
				{
					printf("Battery Level: 0-10%%\n");
					//todo UART Transmit Low battery %d Batt_raw
					sleep_time = 30; // 30 seconds in seconds

					//HAL_UART_Transmit(&huart6, (uint8_t *)"Low Battery Detected, Entering Low Power Mode\n", 44, HAL_MAX_DELAY);
					xSemaphoreGive(xLowPowerSemaphore); // Signal low power mode
					vTaskDelay(pdMS_TO_TICKS(10000)); // Delay to allow Task Block to low power mode.
				}
				else {


					if (Batt_raw < Level_1) {// 10-20%
						sleep_time = 60; // 6 hours in seconds
						printf("Battery Level: 10-20%%\n");
						//todo SET SLEEP_TIME to 6 hours
					}else if (Batt_raw < Level_2) {
						//set wakeup timer to Every 4 hours
						sleep_time = 14400; // 4 hours in seconds
						printf("Battery Level: 20-30%%\n");

					}else if (Batt_raw < Level_3) {
						//set wakeup timer to Every 2 hours
						sleep_time = 7200; // 2 hours in seconds
						printf("Battery Level: 30-40%%\n");

					}else if (Batt_raw < Level_4) {
						//set wakeup timer to Every 1 hour
						sleep_time = 3600; // 1 hour in seconds
						printf("Battery Level: 40-50%%\n");

					}

					// Turn everything ON
					printf("PUMP ON\n");
					HAL_GPIO_WritePin(PUMP1, GPIO_PIN_RESET);  // PC3: Pump ON

					//vTaskDelay(pdMS_TO_TICKS(PUMP1_DURATION ));  // 90 seconds delay while ON //todo time
					vTaskDelay(pdMS_TO_TICKS(1000 ));

					// Turn Pump OFF
					printf("PUMP OFF, SENSORS ON\n");
					HAL_GPIO_WritePin(PUMP1, GPIO_PIN_SET);  // PC3 Pump OFF


					HAL_GPIO_WritePin(pH_POWER, GPIO_PIN_SET);   // PC4: Sensor ON
					HAL_GPIO_WritePin(Temp_POWER, GPIO_PIN_SET);   // PC5: Sensor ON
					vTaskDelay(pdMS_TO_TICKS(1000));  // 60 seconds delay while pump is OFF //todo time


					// Read Sensors
					printf("GETTING DATA\n");

					for (int i = 0; i < MAX_READINGS; i++) {
					    SensorReading r;

					    get_timestamp(r.timestamp); // Your timestamp function
					    r.temp = Temperature_Read();
					    r.ph = Read_ADC_Channel(ADC_CHANNEL_8)/100; //todo
					    r.do_val = Read_ADC_Channel(ADC_CHANNEL_1)/100; //todo
					    r.batt = 80; //todo
					    r.index = i + 1; // Index starts from 1

					    readings[i] = r;
					    reading_ids[i] = global_id++;

					    snprintf(uart_buf, sizeof(uart_buf),
					             "type:0,id:%04d,time:%s,temp:%.2f,ph:%.2f,do:%05.2f,batt:%02d,ind:%d\r\n",
					             reading_ids[i], r.timestamp, r.temp, r.ph, r.do_val, r.batt, r.index);

					    Mount_SD("/");
					    Update_File("LOGS.TXT", uart_buf);
					    Unmount_SD("/");

					    vTaskDelay(pdMS_TO_TICKS(1000));
					}
					xTaskNotifyGive(xTX_Task);

					printf("SENSORS OFF\n");
					HAL_GPIO_WritePin(pH_POWER, GPIO_PIN_RESET);   // PC4: Sensor OFF
					HAL_GPIO_WritePin(Temp_POWER, GPIO_PIN_RESET);   // PC5: Sensor OFF



					printf("Dispose \n ");
					HAL_GPIO_WritePin(PUMP2, GPIO_PIN_RESET);  // PC0: Pump ON
					vTaskDelay(pdMS_TO_TICKS(PUMP2_DURATION));//todo time

					printf("StartClean \n");
					HAL_GPIO_WritePin(PUMP3, GPIO_PIN_RESET);  // PA15: Pump ON
					vTaskDelay(pdMS_TO_TICKS(2000)); //todo time

					printf("StopClean\n");
					HAL_GPIO_WritePin(PUMP3, GPIO_PIN_SET);  // PA15: Pump OFF
					vTaskDelay(pdMS_TO_TICKS(2000));



					printf("StopDispose \n");
					HAL_GPIO_WritePin(PUMP2, GPIO_PIN_SET);  // PC0: Pump OFF
					vTaskDelay(pdMS_TO_TICKS(200)); //todo time
					xSemaphoreGive(xLowPowerSemaphore);
					vTaskDelay(pdMS_TO_TICKS(5000)); //ADDED 10 seconds delay before next cycle
				}
			}
		}
	}


	void TX_Task(void *params)//----------------------------------------------------------------------------------------------------//
	{
	    char tx_buf[256];
	    const TickType_t ack_timeout = pdMS_TO_TICKS(30000);  // 60 seconds
	    const TickType_t poll_interval = pdMS_TO_TICKS(100);  // 100ms polling interval

	    for (;;)
	    {
	        // Wait for initial trigger (from MainTask or Receive_Task)
	        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	        BaseType_t ack_received = pdFALSE;
	        int retry = 0;

	        while (retry < MAX_RETRIES)
	        {
	            resend_requested = false;

	            // Decide what to send
	            bool send_all = true;
	            for (int k = 1; k <= MAX_READINGS; k++) {
	                if (resend_indexes[k - 1] != 0) {
	                    send_all = false;
	                    break;
	                }
	            }

	            // Transmit selected readings
	            for (int i = 0; i < MAX_READINGS; i++) {
	                if (!send_all && resend_indexes[i] == 0)
	                    continue;

	                SensorReading r = readings[i];
	                uint16_t id = reading_ids[i];

	                const char* tx_type_str = (tx_type_override == 3) ? "type:3" : "type:0";

	                snprintf(tx_buf, sizeof(tx_buf),
	                         "%s,id:%04d,time:%s,temp:%.2f,ph:%.2f,do:%.2f,batt:%02d,ind:%d",
	                         tx_type_str, id, r.timestamp, r.temp, r.ph, r.do_val, r.batt, r.index);
	                HAL_UART_Transmit(&huart6, (uint8_t *)tx_buf, strlen(tx_buf), HAL_MAX_DELAY);
	                printf("%s \n", tx_buf);
	                vTaskDelay(pdMS_TO_TICKS(5000));  // Delay between packets
	            }

	            // Wait for ACK or Resend (with resettable timeout)
	            TickType_t wait_start = xTaskGetTickCount();

	            while ((xTaskGetTickCount() - wait_start) < ack_timeout)
	            {
	                if (xSemaphoreTake(xAckSemaphore, poll_interval) == pdTRUE) {
	                    printf("ACK received. Proceeding to STOP mode.\n");

	                    ack_received = pdTRUE;
	                    break;
	                }

	                if (resend_requested) {
	                    printf("Resend request received. Resetting ACK timer.\n");
	                    resend_requested = false;
	                    wait_start = xTaskGetTickCount();  // 🔁 Reset timeout window
	                    break;  // Re-enter transmission loop with updated resend_indexes[]
	                }
	            }

	            if (ack_received) {
	                break;  // ACK success → exit retry loop
	            }

	            retry++;
	            printf("No ACK. Retrying attempt %d...\n", retry);
	        }

	        if (!ack_received) {
	            printf("[TX_Task] Failed after %d attempts. Forcing STOP mode.\n", MAX_RETRIES);
	            xSemaphoreGive(xLowPowerSemaphore);
	        }

	        // Reset resend tracking after transmission session
	        memset(resend_indexes, 0, sizeof(resend_indexes));
	        tx_type_override = 0;
	    }
	}



	void Receive_Task(void *argument)
	{
	    for (;;)
	    {
	        if (xSemaphoreTake(xRtcUpdateSemaphore, portMAX_DELAY) == pdTRUE)
	        {
	            // Ensure buffer is null-terminated
	            uart_rx_buffer[UART_BUFFER_SIZE - 1] = '\0';


	            // Strip newlines or carriage return
	            for (int i = 0; i < UART_BUFFER_SIZE; i++) {
	                if (uart_rx_buffer[i] == '\r' || uart_rx_buffer[i] == '\n') {
	                    uart_rx_buffer[i] = '\0';
	                    break;
	                }
	            }

	            if (strstr((char*)uart_rx_buffer, "type:1") != NULL)
	            {
	                printf("received (type 1)\n");

	                if (xBoot != NULL) {
	                    vTaskDelete(xBoot);
	                    xBoot = NULL;
	                    xSemaphoreGive(xMainTaskSemaphore); // Signal MainTask to start
	                }


	                else {
	                    printf("ack rcvd");
	                    // Signal TX task that ACK has been received
						xSemaphoreGive(xAckSemaphore);
	                }


	            }
	            else if (strstr((char*)uart_rx_buffer, "type:2") != NULL)
	            {
	                printf("Type 2 message — TODO\n");
	            }
	            else if (strstr((char*)uart_rx_buffer, "type:3") != NULL)
	            {
	                memset(resend_indexes, 0, sizeof(resend_indexes));
	                tx_type_override = 3;  // <-- set flag

	                char *ind_str = strstr(uart_rx_buffer, "ind:");
	                if (ind_str) {
	                    ind_str += 4;
	                    char *token = strtok(ind_str, ",");
	                    while (token != NULL) {
	                        int idx = atoi(token);
	                        if (idx >= 1 && idx <= MAX_READINGS) {
	                            resend_indexes[idx - 1] = 1;  // fix index offset
	                        }
	                        token = strtok(NULL, ",");
	                    }

	                    resend_requested = true;
	                    xTaskNotifyGive(xTX_Task);
	                }
	            }

	            else
	            {
	                printf("Unknown type\n");
	            }

	            // Reset buffer
	            memset(uart_rx_buffer, 0, UART_BUFFER_SIZE);
	            uart_rx_index = 0;
	            newline_count = 0;

	            HAL_UART_Receive_IT(&huart6, (uint8_t*)&uart_rx_char, 1);
	        }
	    }
	}



	void BootTask(void *params) {

		for(;;){
			//ParseGPSAndSetRTC();
			PrintGPSTimeFormatted();
			Batt_raw = Read_ADC_Channel(ADC_CHANNEL_0); // Read battery voltage from ADC channel 0
			if (Batt_raw < Level_0 ) {// 0-10%
				printf("Battery Level: 0-10%%\n");
				//todo UART Transmit Low battery %d Batt_raw
				sleep_time = 30; // 30 seconds in seconds
				sleep_lvl = 6;
			}
			else if (Batt_raw < Level_1) {// 10-20%
				sleep_time = 60; // 6 hours in seconds
				sleep_lvl = 6;
				printf("Battery Level: 10-20%%\n");

			}else if (Batt_raw < Level_2) {
				//set wakeup timer to Every 4 hours
				sleep_time = 14400; // 4 hours in seconds
				printf("Battery Level: 20-30%%\n");
				sleep_lvl = 2;
			}else if (Batt_raw < Level_3) {
				//set wakeup timer to Every 2 hours
				sleep_time = 7200; // 2 hours in seconds
				printf("Battery Level: 30-40%%\n");
				sleep_lvl = 2;
			}else {
				//set wakeup timer to Every 1 hour
				sleep_time = 3600; // 1 hour in seconds
				sleep_lvl = 1;
				printf("Battery Level: 40-50%%\n");
				}
			char timestamp[32] = {0};
			get_timestamp(timestamp);  // Must return "YYYY-MM-DDTHH:MM:SS+08:00"
			//Transmit the Wakeuptime via UART6
			snprintf(uart_buf, sizeof(uart_buf), "type:4,time:%s,awc:%d",timestamp, sleep_lvl);
			HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
			printf("Waiting..\n");
			vTaskDelay(pdMS_TO_TICKS(60000)); // Wait 1 minute before continuing

	    // Continue regardless of ACK status
		}
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
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//  	  HAL_NVIC_SetPriority(USART6_IRQn, 5, 0); // Set USART6 interrupt priority
//  	  HAL_NVIC_EnableIRQ(USART6_IRQn);
//	  Mount_SD("/");
//	  Format_SD();
//	  Create_File("LOGS.TXT");
//	  Unmount_SD("/");

	  HAL_TIM_Base_Start(&htim1); // periodic delay timer for SD Card
	  HAL_TIM_Base_Start(&htim2); // Timer for Temp



//	  xTaskCreate(MainTask_handle, "Main-Task", 1000, NULL, 3,  &xMainTask);
//	  configASSERT(xMainTask != NULL);
	  xTaskCreate(BootTask, "Boot", 1024, NULL, 2, &xBoot); // Priority 3
	  configASSERT(BootTask != NULL);
	  xTaskCreate(Receive_Task, "Receive_Handle", 500, NULL, 5, &RcvTask);
	  configASSERT(RcvTask != NULL);
	  xTaskCreate(MainTask_handle, "Main-Task", 1000, NULL, 3, &xMainTask);
	  configASSERT(xMainTask != NULL);
	  xTaskCreate(TX_Task, "TX-Task", 1024, NULL, 1, &xTX_Task);
	  configASSERT(xTX_Task != NULL);
	  HAL_UART_Receive_IT(&huart6, (uint8_t *)&uart_rx_char, 1);
	  // Create binary semaphore for UART transmission
	  xLowPowerSemaphore = xSemaphoreCreateBinary();
	  configASSERT(xLowPowerSemaphore != NULL);
	  xRtcUpdateSemaphore = xSemaphoreCreateBinary();
	  configASSERT(xRtcUpdateSemaphore != NULL);

	  xMainTaskSemaphore = xSemaphoreCreateBinary();
	  configASSERT(xMainTaskSemaphore != NULL);
	  xAckSemaphore = xSemaphoreCreateBinary();
	  configASSERT(xAckSemaphore != NULL);
	  vTaskStartScheduler();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */
  __HAL_RCC_PWR_CLK_ENABLE(); // Enable power interface clock
  HAL_PWR_EnableBkUpAccess();
  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
//	  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2)
//	   {
//		 // First-time setup: set default time & date
//		 sTime.Hours = 12;
//		 sTime.Minutes = 1;
//		 sTime.Seconds = 0;
//		 sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//		 sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//
//		 sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
//		 sDate.Month = RTC_MONTH_JUNE;
//		 sDate.Date = 19;
//		 sDate.Year = 25;
//
//		 if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
//		 {
//		   Error_Handler();
//		 }
//
//		 if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
//		 {
//		   Error_Handler();
//		 }
//
//		 HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);
//	   }
//	   else
//	   {
//		 // Already initialized; just read existing values
//		 HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
//		 HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
//	   }
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x12;
  sTime.Minutes = 0x1;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_JUNE;
  sDate.Date = 0x19;
  sDate.Year = 0x25;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 20, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 60000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 60-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 38400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
  HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC3 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  // USART6_RX = PC7
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;     // Alternate Function Push-Pull
  GPIO_InitStruct.Pull = GPIO_PULLUP;         // Needed to ensure falling edge on idle->start bit
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Reconfigure PC7 as EXTI line for wake-up
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Enable EXTI line for PC7

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


	//TODO

	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
		if (huart->Instance == USART6) {
			if (uart_rx_char == '\n' || uart_rx_char == '\r') {
				newline_count++;
				if (newline_count >= 1) {  // 1 line message
					uart_rx_buffer[uart_rx_index] = '\0';  // Null-terminate

					BaseType_t xHigherPriorityTaskWoken = pdFALSE;
					xSemaphoreGiveFromISR(xRtcUpdateSemaphore, &xHigherPriorityTaskWoken);
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

					return;  // Don't restart reception here, let task do it
				}
			} else if (uart_rx_index < UART_BUFFER_SIZE - 1) {
				uart_rx_buffer[uart_rx_index++] = uart_rx_char;
			} else {
				uart_rx_index = 0;  // Overflow protection
			}

			HAL_UART_Receive_IT(&huart6, (uint8_t*)&uart_rx_char, 1);
		}
	}
	void get_timestamp(char* buffer)
	{
	    RTC_DateTypeDef sDate;
	    RTC_TimeTypeDef sTime;
	    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	    sprintf(buffer, "2025-%02d-%02dT%02d:%02d:%02d+08:00",
	            sDate.Month, sDate.Date,
	            sTime.Hours, sTime.Minutes, sTime.Seconds);
	}
	void ParseGPSAndSetRTC(void) {

	    char *token;
	    RTC_TimeTypeDef sTime = {0};
	    RTC_DateTypeDef sDate = {0};
	    printf("Parsing GPS data to RTC...\n");

	    uint32_t startTick = HAL_GetTick();
	    uint32_t timeout_ms = 5000;  // 5 seconds
	    uint16_t gps_index = 0;
	    gps_buffer[0] = '\0';

	    // Step 1: Read character-by-character with timeout

	    while ((HAL_GetTick() - startTick) < timeout_ms && gps_index < sizeof(gps_buffer) - 1) {
	        uint8_t byte;
	        HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, &byte, 1, 50); // Short timeout

	        if (status == HAL_OK) {
	            gps_buffer[gps_index++] = byte;

	            if (byte == '\n') break;  // End of NMEA sentence
	        } else if (status == HAL_TIMEOUT) {

	        } else {
	            printf("UART Receive error: %d\n", status);
	            break;
	        }
	    }

	    gps_buffer[gps_index] = '\0';

	    if (gps_index == 0) {
	        printf("Timeout: No GPS data received.\n");
	        return;
	    }

	    printf("Received GPS data: %s\n", gps_buffer);

	    // Step 2: Find the $GPRMC sentence
	    char *rmc_ptr = strstr(gps_buffer, "$GPRMC");
	    if (!rmc_ptr) {
	        printf("No GPRMC found\n");
	        return;
	    }

	    // Step 3: Tokenize the sentence
	    token = strtok(rmc_ptr, ","); // $GPRMC
	    token = strtok(NULL, ",");    // Time
	    if (token && strlen(token) >= 6) {
	        char hour_str[3] = {token[0], token[1], '\0'};
	        char min_str[3] = {token[2], token[3], '\0'};
	        char sec_str[3] = {token[4], token[5], '\0'};
	        sTime.Hours = atoi(hour_str);
	        sTime.Minutes = atoi(min_str);
	        sTime.Seconds = atoi(sec_str);
	        sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	        sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	    }

	    // Skip next fields until date
	    for (int i = 0; i < 8; i++) token = strtok(NULL, ",");

	    if (token && strlen(token) == 6) {
	        char day_str[3] = {token[0], token[1], '\0'};
	        char month_str[3] = {token[2], token[3], '\0'};
	        char year_str[3] = {token[4], token[5], '\0'};
	        sDate.Date = atoi(day_str);
	        sDate.Month = atoi(month_str);
	        sDate.Year = atoi(year_str); // RTC supports 0–99 for 2000–2099
	    }

	    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK &&
	        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) == HAL_OK) {

	        HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_SYNC_FLAG, RTC_SYNC_MAGIC);  // Save sync flag
	        printf("RTC set from GPS and sync flag saved.\n");
	    } else {
	        printf("Failed to set RTC from GPS.\n");
	    }

	    printf("RTC initialized to GPS time: %02d:%02d:%02d %02d/%02d/%02d\n",
	           sTime.Hours, sTime.Minutes, sTime.Seconds,
	           sDate.Date, sDate.Month, sDate.Year);
	}

	void PrintGPSTimeFormatted(void) {
	    char line[128] = {0};
	    uint8_t byte;
	    int idx = 0;

	    while (1) {
	        if (HAL_UART_Receive(&huart1, &byte, 1, HAL_MAX_DELAY) == HAL_OK) {
	        	printf("%c", byte); // Print received byte for debugging
	            //HAL_UART_Transmit(&huart2, &byte, 1, HAL_MAX_DELAY); // Debug echo

	            if (byte == '\n') {
	                line[idx] = '\0';

	                if (strstr(line, "$GPRMC")) {
	                    char *token = strtok(line, ","); // $GPRMC
	                    token = strtok(NULL, ",");       // UTC time
	                    if (!token || strlen(token) < 6) { idx = 0; continue; }

	                    int hh = (token[0] - '0') * 10 + (token[1] - '0') + 8; // +8 for timezone
	                    int mm = (token[2] - '0') * 10 + (token[3] - '0');
	                    int ss = (token[4] - '0') * 10 + (token[5] - '0');
	                    if (hh >= 24) hh -= 24; // wrap around midnight UTC+8

	                    token = strtok(NULL, ","); // Status (A/V)
	                    if (token && token[0] == 'A') {

	                        RTC_TimeTypeDef sTime = {0};
	                        RTC_DateTypeDef sDate = {0};

	                        sTime.Hours = hh;
	                        sTime.Minutes = mm;
	                        sTime.Seconds = ss;

	                        // Example fallback date (will be overwritten later if needed)
	                        sDate.Year = 25;
	                        sDate.Month = RTC_MONTH_JULY;
	                        sDate.Date = 22;
	                        sDate.WeekDay = RTC_WEEKDAY_THURSDAY;

	                        // Optional: try parsing date from GPRMC
	                        token = strtok(NULL, ","); // Latitude
	                        token = strtok(NULL, ","); // N/S
	                        token = strtok(NULL, ","); // Longitude
	                        token = strtok(NULL, ","); // E/W
	                        token = strtok(NULL, ","); // Speed
	                        token = strtok(NULL, ","); // Course
	                        token = strtok(NULL, ","); // Date (ddmmyy)
	                        if (token && strlen(token) == 6) {
	                            sDate.Date = (token[0] - '0') * 10 + (token[1] - '0');
	                            sDate.Month = (token[2] - '0') * 10 + (token[3] - '0');
	                            sDate.Year = (token[4] - '0') * 10 + (token[5] - '0');
	                        }

	                        if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
	                            Error_Handler();
	                        }

	                        if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
	                            Error_Handler();
	                        }

	                        HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_SYNC_FLAG, RTC_SYNC_MAGIC); // Save sync flag
	                        printf("RTC set from GPS: %02d:%02d:%02d %02d/%02d/%02d\n",
	                               sTime.Hours, sTime.Minutes, sTime.Seconds,
	                               sDate.Date, sDate.Month, sDate.Year);
	                        return; // ✅ Exit after setting RTC
	                    }
	                }

	                idx = 0;
	                memset(line, 0, sizeof(line));
	            } else if (idx < sizeof(line) - 1) {
	                line[idx++] = byte;
	            }
	        }
	    }
	}

	void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
	{
		if (xSemaphoreTake(xLowPowerSemaphore, 0) == pdPASS)
		{
			printf("Entered low power mode\n");

			if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 60, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK) //TODO change sleep time
			{
			Error_Handler();
			}
			//---------------------------------------------------------------------------------------------------//

			//---------------------------------------------------------------------------------------------------//
			// Disable SysTick

			SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

			// Enter critical section
			taskENTER_CRITICAL();
			// Prepare for Stop Mode (clear wakeup flags, etc.)
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
			//HAL_PWREx_EnableFlashPowerDown();

			// Enter Stop Mode
			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

			//TODO

			// After wakeup: restore clock
			SystemClock_Config();
			// Re-enable SysTick
			SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;

			//---------------------------------------------------------------------------------------------------//



			//---------------------------------------------------------------------------------------------------//
			// Check wakeup source

			printf("Exited low power mode\n");
			HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
			//HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 20, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

			//HAL_PWREx_DisableFlashPowerDown();
			xSemaphoreGive(xMainTaskSemaphore); // Notify main task to continue
			// Exit critical section
			taskEXIT_CRITICAL();

		}
		else
		{
			// Do nothing, let FreeRTOS idle task spin
		}
	}

	uint32_t Read_ADC_Channel(uint32_t channel) {

		ADC_ChannelConfTypeDef sConfig = {
			.Channel = channel,
			.Rank = 1,
			.SamplingTime = ADC_SAMPLETIME_84CYCLES
		};
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		return HAL_ADC_GetValue(&hadc1);
	}

//	void WaitForACKOrRetry(void)
//	{
//	    int retry_count = 0;
//	    BaseType_t ackReceived;
//
//	    while (retry_count < MAX_RETRIES)
//	    {
//	        printf("Waiting for ACK... Attempt %d\n", retry_count + 1);
//
//	        // Start UART interrupt receive
//
//
//	        // Wait for ACK semaphore (set in ISR callback)
//
//
//	        if (xSemaphoreTake(xAckSemaphore, pdMS_TO_TICKS(ACK_TIMEOUT_MS) == pdTRUE))
//	        {
//	            printf("ACK received.\n");
//	            return;  // Success
//	        }
//	        else
//	        {
//	            printf("ACK not received. Retrying...\n");
//
//	            // Retransmit last message
//	            snprintf(uart_buf, sizeof(uart_buf), "Wakeup Time: %d seconds\r\n", sleep_time);
//	            HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
//	        }
//
//	        retry_count++;
//	    }
//
//	    printf("No ACK after %d attempts. Proceeding...\n", MAX_RETRIES);
//	}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
