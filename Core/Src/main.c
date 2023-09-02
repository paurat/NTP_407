/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "lwip.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <math.h>
#include "time.h"
#include "lwip/api.h"
#include "lwip/netif.h"
#include "lwip/apps/httpd.h"
#include "myapi.h"
#include "local_files.h"
#include "MyFlash.h"
#include <sys/socket.h>
#include <arpa/inet.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define NTP_MS_TO_FS_U32  (4294967296.0 / 1000.0)
#define NTP_MS_TO_FS_U16  (65536.0 / 1000.0)
/* Number of seconds between 1970 and Feb 7, 2036 06:28:16 UTC (epoch 1) */
#define DIFF_SEC_1970_2036          ((uint32_t)2085978496L)
#define NTP_TIMESTAMP_DELTA 2208988800ull
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
/* USER CODE BEGIN PV */
time_t rtc_read(void);
char calc_crc(char c,int cnt);
void tcpecho_init(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI3_Init(void);
void StartDefaultTask(void const * argument);
void tcpecho_thread(void const * argument);

/* USER CODE BEGIN PFP */
char calc_crc(char c,int cnt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int ERRORS=0;
int Tipe_Mes=0;// 1-ZDA  2-RMC
int Test=0;
long long int gps_unix=0;
int zpt=0;
int start_crc=0;
int z=0;
int ind=0;
int crc=0;
int count=0;
int dec;
int PPS_count=0;//вынес для проверки
int PPS_mass[10]={0};//вынес для проверки
int crc_pars=0;
char time_buff[9]={0};
char crc_buff[3]={0};
char buff[1]={0};
int dataReceived=1;
int dataTransmitted=1;
char year_str[2]={0};
int century=100;
//_______________________________0_______1_______2________3_______4______5_______6_______7_______8_______9______10______11_____12_____13___14____15____16_____17______18_____19______20_____21______22_____23_____24_____25_____26_____27______28_____29______30
//int offset_minutes[]={         0,      0,      0,       0,      0,     0,      0,      0,    -30,      0,      0,      0,     0,     0,   0,    0,   30,     0,      0,    30,      0,    30,      0,     0,     0,    30,     0,     0,      0,     0,      0 };
//int offset_hours[]={         -11,    -10,     -9,      -8,     -7,    -6,     -5,     -4,     -3,      3,     -2,     -1,     0,     1,   2,    3,    3,     4,      5,     5,      6,     6,      7,     8,     9,     9,    10,    11,     12,    13,     14 };
const int offset_unix[]={   -39600, -36000, -32400,  -28800, -25200,-21600, -18000, -14400, -12600, -10800,  -7200,  -3600,     0, -3600,7200,10800,12600, 14400,  18000, 19800,  21600, 23400,  25200, 28800, 32400, 34200, 36000, 39600,  43200, 46800,  50400};
struct tm Time_calc;

typedef struct
{
	uint8_t li_vn_mode;

	uint8_t stratum;
	uint8_t poll;
	uint8_t precision;

	uint32_t rootDelay;

	uint16_t rootDispersion_s;
	uint16_t rootDispersion_f;

	uint32_t refId;

	uint32_t refTm_s;
	uint32_t refTm_f;

	uint32_t origTm_s;
	uint32_t origTm_f;

	uint32_t rxTm_s;
	uint32_t rxTm_f;

	uint32_t txTm_s;
	uint32_t txTm_f;

} ntp_packet_t;

/* From GNSS PPS */
 uint32_t time_ref_s;
 uint32_t time_ref_f;

typedef enum {
	NTPD_UNSYNC = 0,
	NTPD_IN_LOCK = 1,
	NTPD_IN_HOLDOVER = 2,
	NTPD_DEGRADED = 3
} ntpd_status_status_t;
typedef struct {
	ntpd_status_status_t status;
	uint32_t requests_count;
	uint8_t stratum;
} ntpd_status_t;

ntpd_status_t ntpd_status = {
  .status = NTPD_IN_LOCK,
  .requests_count = 0,
  .stratum = 1
};
//RTCDateTime ntpd_datetime;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	memset(&gps,0,sizeof(gps));
		// ZDA-38;RMC-68



		 //включение ZDA
		 char MESZDA[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x08, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x0B, 0x6B};
		 char CONZDA[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x08, 0x01, 0x19};

		 //отключение ZDA
		 //char MESZDA[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x5B};
		 //char CONZDA[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x08, 0x01, 0x19};

		 //отключение остального
		 char MESGGA[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};
		 char CONGGA[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x00, 0xF9, 0x11};

		 char MESGLL[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
		 char CONGLL[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x01, 0xFA, 0x12};

		 char MESGSA[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
		 char CONGSA[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x02, 0xFB, 0x13};

		 char MESGSV[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
		 char CONGSV[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x03, 0xFC, 0x14};

		 char MESVTG[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
		 char CONVTG[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x05, 0xFE, 0x16};

		 //отключение RMC на всякий
		 //char MESRMC[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F};
		 //char CONRMC[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x04, 0xFD, 0x15};

		 //включение RMC
		 char MESRMC[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x07, 0x4F};
		 char CONRMC[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x04, 0xFD, 0x15};

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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  //TIM1
    HAL_TIM_Base_Start_IT(&htim2);

    HAL_Delay(5000);

   //ON ZDA
    HAL_UART_Transmit(&huart2,(uint8_t*) MESZDA, 16, 1000);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart2,(uint8_t*) CONZDA, 10, 1000);
    HAL_Delay(100);

    // OFF protokol
    HAL_UART_Transmit(&huart2,(uint8_t*) MESGGA, 16, 1000);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart2,(uint8_t*) CONGGA, 10, 1000);
    HAL_Delay(100);

    HAL_UART_Transmit(&huart2,(uint8_t*) MESGLL, 16, 1000);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart2,(uint8_t*) CONGLL, 10, 1000);
    HAL_Delay(100);

    HAL_UART_Transmit(&huart2,(uint8_t*) MESGSA, 16, 1000);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart2,(uint8_t*) CONGSA, 10, 1000);
    HAL_Delay(100);

    HAL_UART_Transmit(&huart2,(uint8_t*) MESGSV, 16, 1000);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart2,(uint8_t*) CONGSV, 10, 1000);
    HAL_Delay(100);

    HAL_UART_Transmit(&huart2,(uint8_t*) MESVTG, 16, 1000);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart2,(uint8_t*) CONVTG, 10, 1000);
    HAL_Delay(100);

    //отключение и включение RMC на всякий
    HAL_UART_Transmit(&huart2,(uint8_t*) MESRMC, 16, 1000);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart2,(uint8_t*) CONRMC, 10, 1000);
    HAL_Delay(100);

    //start the web server
    int offset =0;
   ReadDeviceAddressOffset((char*) &user_info, sizeof(user_info), offset);
   offset+=sizeof(user_info);
   //Обнуление PPS
   PPS_count=0;
   gps.year[0]='V';
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, tcpecho_thread, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 23;
  sTime.Minutes = 59;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_DECEMBER;
  sDate.Date = 31;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  huart1.Init.BaudRate = 115200;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PPS_Pin */
  GPIO_InitStruct.Pin = PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PPS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	int PPS_Counter_period=0;
	//int PPS_count=0;//вынес для проверки
	//int PPS_mass[10]={0};//вынес для проверки
	if(GPIO_Pin == PPS_Pin) {
		if(PPS_count>2&&PPS_count<12){
		PPS_mass[PPS_count-2] = TIM2->CNT;
		}
		if(PPS_count==12){
			//HAL_GPIO_TogglePin(Timled_GPIO_Port, Timled_Pin);
			PPS_Counter_period=(PPS_mass[0]+PPS_mass[1]+PPS_mass[2]+PPS_mass[3]+PPS_mass[4]+PPS_mass[5]+PPS_mass[6]+PPS_mass[7]+PPS_mass[8]+PPS_mass[9])/9;
			TIM2->ARR=PPS_Counter_period;
		}
	}
	if(PPS_count<13){
		PPS_count=PPS_count+1;
		TIM2->CNT = 0;//обнуление счетчика
		}


		}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	if(huart == &huart2) {
		//$ message start
		if(buff[0]=='$'){
			count=0;
			zpt=0;
		}
		//Message error
		else if (count==1&&buff[0]!='G'){
			count=0;
			ERRORS++;
		}


		//CRC calculation
		int res = calc_crc(buff[0],count);
		if(res==1){
			//printf("crc=%d\t crc_buff=%s\t dec=%d\n\r",crc,crc_buff,dec);
			//RTC READ
			rtc_read();
			//printf("rtc_read=%llu\t",rtc_read());
			//comparison RTC&CRC
			//Time_calc.tm_wday = 1;//atoi(gps.);
			Time_calc.tm_mon = atoi(gps.month)-1;//-1 do January==0 month
			Time_calc.tm_mday = atoi(gps.day);
			if(year_str[0]=='0'&&year_str[1]=='0'){
				century=century+100;//atoi(gps.year)
			}
			Time_calc.tm_year = atoi(year_str) + century;
			Time_calc.tm_hour = atoi(gps.hours);
			Time_calc.tm_min = atoi(gps.minuttes);
			Time_calc.tm_sec = atoi(gps.seconds);
			gps_unix = mktime(&Time_calc);
			//printf("tm_year=%d\t tm_mon=%d\t tm_mday=%d\t tm_hour=%d\t tm_min=%d\t tm_sec=%d\n",Time_calc.tm_year,Time_calc.tm_mon,Time_calc.tm_mday,Time_calc.tm_hour,Time_calc.tm_min,Time_calc.tm_sec);
			//printf("rtc_read=%llu\t Time_calc=%llu\n",rtc_read(),gps_unix);

		}
		if(res==1&&gps_unix!=rtc_read()){

			time_ref_s=htonl(gps_unix- DIFF_SEC_1970_2036);
			sTime.Hours = Time_calc.tm_hour;
			sTime.Minutes = Time_calc.tm_min;
			sTime.Seconds = Time_calc.tm_sec;
			sTime.StoreOperation = RTC_STOREOPERATION_RESET;




			if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
			{
				Error_Handler();
			}
			sDate.Month = Time_calc.tm_mon;
			sDate.Date = Time_calc.tm_mday;
			sDate.Year = Time_calc.tm_year-century;
			if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
			{
				Error_Handler();
			}


		}
		//ZDA OR RMC
				if (count==3&&buff[0]=='Z'){
					Tipe_Mes=1;
				}
				else if(count==3&&buff[0]=='R'){
					Tipe_Mes=2;
				}
				//If ZDA
				if(Tipe_Mes==1){

					if(count==6&&buff[0]!=','){
						count=0;
						ERRORS++;
					}
					if(buff[0]==','){
						zpt++;
						ind=0;
					}
					if(zpt==1&&buff[0]!=','){
						time_buff[ind]=buff[0];
						ind++;
					}
					if(zpt==2&&buff[0]==','){
						gps.hours[0]=time_buff[0];
						gps.hours[1]=time_buff[1];
						gps.minuttes[0]=time_buff[2];
						gps.minuttes[1]=time_buff[3];
						gps.seconds[0]=time_buff[4];
						gps.seconds[1]=time_buff[5];
						gps.seconds[2]=time_buff[6];
						gps.seconds[3]=time_buff[7];
						gps.seconds[4]=time_buff[8];
					}
					if(zpt==2&&buff[0]!=','){

						time_buff[ind]=buff[0];
						ind++;
					}
					if(zpt==3&&buff[0]==','){
						gps.day[0]=time_buff[0];
						gps.day[1]=time_buff[1];
					}
					if(zpt==3&&buff[0]!=','){

						time_buff[ind]=buff[0];
						ind++;
					}
					if(zpt==4&&buff[0]==','){
						gps.month[0]=time_buff[0];
						gps.month[1]=time_buff[1];
					}
					if(zpt==4&&buff[0]!=','){

						time_buff[ind]=buff[0];
						ind++;
					}
					if(zpt==5&&buff[0]==','){
						gps.year[0]=time_buff[0];
						gps.year[1]=time_buff[1];
						gps.year[2]=time_buff[2];
						gps.year[3]=time_buff[3];
						year_str[0]=time_buff[2];
						year_str[1]=time_buff[3];
					}
				}


				//IF RMC
				if(Tipe_Mes==2){

					if(count==6&&buff[0]!=','){
						count=0;
						ERRORS++;
					}
					if(buff[0]==','){
						zpt++;
						ind=0;
					}
					if(zpt==1&&buff[0]!=','){

						time_buff[ind]=buff[0];
						ind++;
					}
					if(zpt==2&&buff[0]==','){
						gps.hours[0]=time_buff[0];
						gps.hours[1]=time_buff[1];
						gps.minuttes[0]=time_buff[2];
						gps.minuttes[1]=time_buff[3];
						gps.seconds[0]=time_buff[4];
						gps.seconds[1]=time_buff[5];
						gps.seconds[2]=time_buff[6];
						gps.seconds[3]=time_buff[7];
						gps.seconds[4]=time_buff[8];
					}
					if(zpt==2&&buff[0]!=','){

						time_buff[ind]=buff[0];
						ind++;
					}
					if(zpt==3&&buff[0]==','){
						gps.sinc[1]=time_buff[0];
					}

					if(zpt==9&&buff[0]!=','){

						time_buff[ind]=buff[0];
						ind++;
					}
					if(zpt==10&&buff[0]==','){
						gps.day[0]=time_buff[0];
						gps.day[1]=time_buff[1];
						gps.month[0]=time_buff[2];
						gps.month[1]=time_buff[3];
						gps.year[0]=time_buff[4];
						gps.year[1]=time_buff[5];
						year_str[0]=time_buff[4];
						year_str[1]=time_buff[5];
					}
				}

				//printf("buff=%c\tcount=%d\tzpt=%d\tind=%d\tTipe_Mes=%d\n\r",buff[0],count,zpt,ind,Tipe_Mes);
				//printf("crc_hx=%s\t crc=%d\t crc_buff=%s\t dec=%d\n\r",crc_hx,crc,crc_buff,dec);
				dataReceived=1;

				if( dataTransmitted != 0 ) {

					//HAL_UART_Transmit_IT(&huart6, (uint8_t *)buff, 1);

					dataReceived=0;
					dataTransmitted=0;
				}

				HAL_UART_Receive_IT (&huart2, (uint8_t *)buff, 1);
				gps.errors[1]=ERRORS;
				count++;
			}
		}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

//	//if(huart == &huart6) {

		dataTransmitted=1;

		if( dataReceived != 0 ) {
			//HAL_UART_Transmit_IT(&huart6, (uint8_t *)buff, 1);
			dataReceived=0;
			dataTransmitted=0;
		}
	//}
}char Hex_to_dec(char hex[2]){
    int i;
    int dig; /*to store digit*/
    int cont = 0;
    dec = 0;
    for (i = (strlen(hex) - 1); i >= 0; i--) {
        switch (hex[i]) {
        case 'A':
            dig = 10;
            break;
        case 'B':
            dig = 11;
            break;
        case 'C':
            dig = 12;
            break;
        case 'D':
            dig = 13;
            break;
        case 'E':
            dig = 14;
            break;
        case 'F':
            dig = 15;
            break;
        default:
            dig = hex[i] - 0x30;
        }
        dec = dec + (dig)*pow((double)16, (double)cont);
        cont++;
    }
    return dec;
}
char calc_crc(char c,int cnt){
	if (c=='*'){
		start_crc=0;
		crc_pars=1;
		z=0;
	}
	if(start_crc==1){
		crc^=c;
	}
	if(crc_pars==1&&c!='*'&&z<=1){
		crc_buff[z]=c;
		z++;
	}
	if(c=='\n'){
		Hex_to_dec(crc_buff);
		if(crc==dec){
			//Test++;
			return 1;
		}
	}
	if(cnt==0){
		start_crc=1;
		crc_pars=0;
		crc=0;
	}
//	printf("crc=%d\t crc_buff=%s\t dec=%d\n\r",crc,crc_buff,dec);
	return 0;
}
time_t rtc_read(void) {
	RTC_DateTypeDef dateStruct;
	RTC_TimeTypeDef timeStruct;
	struct tm timeinfo;

	hrtc.Instance = RTC;

	// Read actual date and time
	HAL_RTC_GetTime(&hrtc, &timeStruct, FORMAT_BIN); // Read time first!
	HAL_RTC_GetDate(&hrtc, &dateStruct, FORMAT_BIN);

	// Setup a tm structure based on the RTC
	// monday==1 sunday==7
	timeinfo.tm_wday = dateStruct.WeekDay;
	timeinfo.tm_mon = dateStruct.Month;//-1 do January==0 month
	timeinfo.tm_mday = dateStruct.Date;
	timeinfo.tm_year = dateStruct.Year + 100;
	timeinfo.tm_hour = timeStruct.Hours;
	timeinfo.tm_min = timeStruct.Minutes;
	timeinfo.tm_sec = timeStruct.Seconds;
	//printf("tm_wday=%d\t\n",timeinfo.tm_wday);

	// Convert to timestamp
	time_t t = mktime(&timeinfo)+offset_unix[user_info.zone];


	return t;
}

void tcpecho_init(void)
{
	sys_thread_new("tcpecho_thread", tcpecho_thread, NULL,DEFAULT_THREAD_STACKSIZE, 1);
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_tcpecho_thread */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tcpecho_thread */
void tcpecho_thread(void const * argument)
{
  /* USER CODE BEGIN tcpecho_thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END tcpecho_thread */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
