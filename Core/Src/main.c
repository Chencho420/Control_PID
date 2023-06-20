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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "eeprom_emul.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRUEBA false
#define PRUEBA_DAC false
#define PRUEBA_EEPROM false
// botones
#define NULO      0
#define ACEPTAR   1
#define CANCELAR  2
#define PERILLA   3
// menu principal
#define INSTALADOR  1  
#define MONITOREO   2
#define AJUSTES     3
// menu monitoreo
#define DIGITAL   1
#define ANALOGICO 2
// menu offsets
#define AI1 1
#define AI2 2
#define AI3 3
#define A0  4
#define KP  5
#define KI  6
#define KD  7
#define TL  8
#define TD  9
#define TI  10
//Insalador
#define PROMEDIADOR_1         1
#define PROMEDIADOR_2         2
#define PROMEDIADOR_3         3
#define TIEMPO_FILTRO_DIGITAL 4
//Instalador 1
#define SALIDA_UNIPOLAR                 1
#define KPI_ABIERTO                     2
#define PID_SALIDA_INV                  3
#define PORCENTAJE_PID_POSITIVO         4
#define PORCENTAJE_PID_NEGATIVO         5
#define SUMA_PID_REFERENCIA             6
#define LIMITE_SALIDA_ANALOG_POSITIVA   7
#define LIMITE_SALIDA_ANALOG_NEGATIVA   8
// Definir la dirección de inicio de la sección de memoria flash que se utilizará para almacenar las variables
#define FLASH_USER_START_ADDR ((uint32_t)0x08040000) // Dirección de inicio de la sección de memoria flash

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
int offset_AI_1 = 0;
int offset_AI_2 = 0;
int offset_AI_3 = 0;
int offset_A0 = 2000;
int kp = 0;
int ki = 0;
int kd = 0;
int tiempo_lazo = 0;
int td = 0;
int ti = 0;
int promediador_1 = 0;
int promediador_2 = 0;
int promediador_3 = 0;
int tiempo_filtro_digital = 0;
bool salida_unipolar = false;
bool pid_salida_inv = false;
bool sumar_pid_con_referencia = false;
int kpi_abierto = 0;
int pid_positivo = 0;
int pid_negativo = 0;
int limite_analogica_positiva = 0;
int limite_analogica_negativa = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void Pantalla_menu(int opcion);       // plantilla para menus, no se usa directamente
void Pantalla_inicial(int tiempo_ms); // Muestra el logo por el tiempo seleccionado
int Leer_botones(void);               // Lee los 3 botones de usuario y devuelve el valor del botón pesionado
int Pantalla_menu_inicial(void);      // Menú inicial, devuelve la opción seleccionada
int Leer_encoder(int maximo);
int AI1_leer(void);
int AI2_leer(void);
int AI3_leer(void);
void Encoder_reiniciar(void);
int Establecer_valor(char *texto, int *variable, int valor_maximo);
void monitoreo_analogico(void);
void monitoreo_digital(void);
int menu_monitoreo(void);
int menu_offsets(void);
void read_variables_from_flash(void);
void write_variables_to_flash(void);
uint32_t read_data_address(uint32_t Address);
void save_data(uint32_t Address,uint32_t data);
int menu_instalador_1(void);
bool cambiar_opcion(char *texto ,bool *valor);
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
  EE_Status ee_status = EE_OK;
  int dac_canal_1 = 2000, dac_canal_2 = 2000;
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
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_DAC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  TIM4->CR1 |= TIM_CR1_CEN; // Inicializa la perilla
  TIM2->CR1 |= TIM_CR1_CEN; // Inicializa el encoder 1
  Pantalla_inicial(1000);
  #if PRUEBA 
  uint32_t valor;
  char buf[10];
  save_data(FLASH_USER_START_ADDR, 255);
  valor = 0;//read_data_address(FLASH_USER_START_ADDR);
  itoa(valor, buf, 10);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString(buf, Font_7x10, White);
  HAL_Delay(10000);
  #endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 0x100);
  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(DO_2_ERROR_GPIO_Port, DO_2_ERROR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(DO_1_OK_GPIO_Port, DO_1_OK_Pin, GPIO_PIN_SET);
  //write_variables_to_flash();
  HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1 , DAC_ALIGN_8B_R, 0);
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
  HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2 , DAC_ALIGN_8B_R, 0);
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_2);

#if PRUEBA_EEPROM
  uint32_t Index = 1;
  __IO uint32_t ErasingOnGoing = 0;
  uint32_t a_VarDataTab[NB_OF_VARIABLES] = {0};
  uint32_t VarValue = 0;
  /* Store 10 values of all variables in EEPROM, ascending order */
  for (VarValue = 1; VarValue <= 10; VarValue++)
  {
    for (Index = 1; Index < NB_OF_VARIABLES+1; Index++)
    {
      /* Wait any cleanup is completed before accessing flash again */
      while (ErasingOnGoing == 1) { }
      
      ee_status = EE_WriteVariable32bits(Index, Index*VarValue);
      ee_status|= EE_ReadVariable32bits(Index, &a_VarDataTab[Index-1]);
      if (Index*VarValue != a_VarDataTab[Index-1]) {Error_Handler();}

      /* Start cleanup IT mode, if cleanup is needed */
      if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {ErasingOnGoing = 1;ee_status|= EE_CleanUp_IT();}
      if ((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {Error_Handler();}
    }
  }

  /* Read all the variables */
  for (Index = 1; Index < NB_OF_VARIABLES+1; Index++)
  {
    ee_status = EE_ReadVariable32bits(Index, &VarValue);
    if (VarValue != a_VarDataTab[Index-1]) {Error_Handler();}
    if (ee_status != EE_OK) {Error_Handler();}
  }

  /* Store 1000 values of Variable1,2,3 in EEPROM */
  for (VarValue = 1; VarValue <= 12; VarValue++)
  {
    while (ErasingOnGoing == 1) { }

    ee_status = EE_WriteVariable32bits(1, VarValue);
    ee_status|= EE_ReadVariable32bits(1, &a_VarDataTab[0]);
    if (VarValue != a_VarDataTab[0]) {Error_Handler();}

    ee_status|= EE_WriteVariable32bits(2, ~VarValue);
    ee_status|= EE_ReadVariable32bits(2, &a_VarDataTab[1]);
    if (~VarValue != a_VarDataTab[1]) {Error_Handler();}

    ee_status|= EE_WriteVariable32bits(3, VarValue << 1);
    ee_status|= EE_ReadVariable32bits(3, &a_VarDataTab[2]);
    if ((VarValue << 1) != a_VarDataTab[2]) {Error_Handler();}

    /* Start cleanup polling mode, if cleanup is needed */
    if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {ErasingOnGoing = 0;ee_status|= EE_CleanUp();}
    if ((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {Error_Handler();}
  }

  /* Read all the variables */
  for (Index = 1; Index < NB_OF_VARIABLES+1; Index++)
  {
    ee_status = EE_ReadVariable32bits(Index, &VarValue);
    if (VarValue != a_VarDataTab[Index-1]) {Error_Handler();}
    if (ee_status != EE_OK) {Error_Handler();}
  }
#endif 
  HAL_FLASH_Lock();


  HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1 , DAC_ALIGN_8B_R, 0);
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
  HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2 , DAC_ALIGN_8B_R, 0);
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_2);

  while (true)
  {

  /* Test is completed successfully */
  /* Lock the Flash Program Erase controller */

    switch (Pantalla_menu_inicial())
    {
    case INSTALADOR:
      switch (menu_instalador_1())
      {
      case SALIDA_UNIPOLAR:
      cambiar_opcion("Salida Unipolar", &salida_unipolar);
        break;
      case KPI_ABIERTO:
      //kpi_abierto = Establecer_valor("KPI abierto", kpi_abierto, 1000);
      Establecer_valor("KPI abierto", &kpi_abierto, 1000);
        break;
      case PID_SALIDA_INV:
      cambiar_opcion("PID salida", &pid_salida_inv);
        break;
      case PORCENTAJE_PID_POSITIVO:
      Establecer_valor("Porcentaje PID +", &pid_positivo, 100);
        break;
      case PORCENTAJE_PID_NEGATIVO:
      Establecer_valor("Porcentaje PID -", &pid_negativo, 100);
        break;
      case SUMA_PID_REFERENCIA:
      cambiar_opcion("Suma PID ref", &sumar_pid_con_referencia);
        break;
      case LIMITE_SALIDA_ANALOG_POSITIVA:
      Establecer_valor("Voltaje salida +", &limite_analogica_positiva, 10);
        break;
      case LIMITE_SALIDA_ANALOG_NEGATIVA:
      Establecer_valor("Voltaje salida -", &limite_analogica_negativa, 10);
        break;
      }
      
      continue;
      switch (menu_instalador())
      {
      case PROMEDIADOR_1:
        Establecer_valor("Promediador 1", &promediador_1, 20000);
        break;
      case PROMEDIADOR_2:
        Establecer_valor("Promediador 2", &promediador_2, 20000);
        break;
      case PROMEDIADOR_3:
        Establecer_valor("Promediador 3", &promediador_3, 20000);
        break;
      case TIEMPO_FILTRO_DIGITAL:
        Establecer_valor("Filtro digital", &tiempo_filtro_digital, 20000);
        break;
      }
      break;
    case MONITOREO:
      switch (menu_monitoreo())
      {
      case DIGITAL:
        monitoreo_digital();
        break;
      case ANALOGICO:
        monitoreo_analogico();
        break;
      }
      break;
    case AJUSTES:
      switch (menu_offsets())
      {
      case AI1:
        Establecer_valor("Offset AI 1", &offset_AI_1, 2000);
        break;
      case AI2:
        Establecer_valor("Offset AI 2", &offset_AI_2, 2000);
        break;
      case AI3:
        Establecer_valor("Offset AI 3", &offset_AI_3, 2000);
        break;
      case A0:
        Establecer_valor("Offset A0", &offset_A0, 4096);
        #if PRUEBA_DAC
        HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1 , DAC_ALIGN_12B_R, offset_A0);
        HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);

        canal2 = Establecer_valor_DAC("Canal 2", canal2, 4096);
        //HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2 , DAC_ALIGN_12B_R, 2048);
        HAL_DAC_Start(&hdac1, DAC1_CHANNEL_2);
        #endif
        break;
      case KP:
        Establecer_valor("KP", &kp, 32000);
        break;
      case KI:
        Establecer_valor("KI", &ki, 32000);
        break;
      case KD:
        Establecer_valor("KD", &kd, 32000);
        break;
      case TL:
        Establecer_valor("TL", &tiempo_lazo, 20000);
        break;
      case TD:
        Establecer_valor("KI", &td, 20000);
        break;
      case TI:
        Establecer_valor("KD", &ti, 20000);
        break;
      }
      break;
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

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
   */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
   */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
   */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
   */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
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
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
   */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1700;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4.294967295E9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 17000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO_2_ERROR_GPIO_Port, DO_2_ERROR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO_1_OK_GPIO_Port, DO_1_OK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BOTON_1_Pin BOTON_2_Pin */
  GPIO_InitStruct.Pin = BOTON_1_Pin | BOTON_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PERILLA_BOTON_Pin */
  GPIO_InitStruct.Pin = PERILLA_BOTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PERILLA_BOTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DO_2_ERROR_Pin */
  GPIO_InitStruct.Pin = DO_2_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_2_ERROR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DO_1_OK_Pin */
  GPIO_InitStruct.Pin = DO_1_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_1_OK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_4_Pin DI_3_Pin DI_2_PID_APAGADO_Pin DI_1_ENABLED_Pin */
  GPIO_InitStruct.Pin = DI_4_Pin | DI_3_Pin | DI_2_PID_APAGADO_Pin | DI_1_ENABLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Pantalla_menu(int opcion)
{
  ssd1306_Fill(Black);
  ssd1306_SetCursor(46, 0);
  ssd1306_WriteString("menu", Font_7x10, White);

  ssd1306_SetCursor(0, 18);
  if (opcion == 1)
  {
    ssd1306_WriteString("opcion 1", Font_7x10, Black);
  }
  else
  {
    ssd1306_WriteString("opcion 1", Font_7x10, White);
  }

  ssd1306_SetCursor(0, 36);
  if (opcion == 2)
  {
    ssd1306_WriteString("opcion 2", Font_7x10, Black);
  }
  else
  {
    ssd1306_WriteString("opcion 2", Font_7x10, White);
  }

  ssd1306_SetCursor(0, 54);
  if (opcion == 3)
  {
    ssd1306_WriteString("opcion 3", Font_7x10, Black);
  }
  else
  {
    ssd1306_WriteString("opcion 3", Font_7x10, White);
  }

  ssd1306_SetCursor(64, 18);
  if (opcion == 4)
  {
    ssd1306_WriteString("opcion 4", Font_7x10, Black);
  }
  else
  {
    ssd1306_WriteString("opcion 4", Font_7x10, White);
  }

  ssd1306_SetCursor(64, 36);
  if (opcion == 5)
  {
    ssd1306_WriteString("opcion 5", Font_7x10, Black);
  }
  else
  {
    ssd1306_WriteString("opcion 5", Font_7x10, White);
  }

  ssd1306_SetCursor(64, 54);
  if (opcion == 6)
  {
    ssd1306_WriteString("opcion 6", Font_7x10, Black);
  }
  else
  {
    ssd1306_WriteString("opcion 6", Font_7x10, White);
  }
  ssd1306_UpdateScreen();
}

void Pantalla_inicial(int tiempo_ms)
{
  ssd1306_Fill(Black);
  ssd1306_SetCursor(32, 16);
  ssd1306_WriteString("ACM", Font_16x24, White);
  ssd1306_SetCursor(10, 50);
  ssd1306_WriteString("automatizacion", Font_7x10, White);
  ssd1306_UpdateScreen();
  HAL_Delay(tiempo_ms);
  return;
}

int Leer_botones(void)
{
  int estado_anterior = NULO;
  int estado_actual = NULO;
  int boton_presionado = false;

  // Leer el estado de BOTON_1
  if (!HAL_GPIO_ReadPin(BOTON_1_GPIO_Port, BOTON_1_Pin))
  {
    estado_actual = ACEPTAR;
    boton_presionado = true;
  }
  // Leer el estado de BOTON_2
  else if (!HAL_GPIO_ReadPin(BOTON_2_GPIO_Port, BOTON_2_Pin))
  {
    estado_actual = CANCELAR;
    boton_presionado = true;
  }
  // Leer el estado de PERILLA_BOTON
  else if (!HAL_GPIO_ReadPin(PERILLA_BOTON_GPIO_Port, PERILLA_BOTON_Pin))
  {
    estado_actual = PERILLA;
    boton_presionado = true;
  }
  else
  {
    estado_actual = NULO;
    boton_presionado = false;
  }

  // Si el estado actual es diferente al estado anterior
  if (estado_actual != estado_anterior)
  {
    // Si se presionó algún botón, esperar a que se suelte
    if (boton_presionado)
    {
      while (true)
      {
        // Esperar a que el botón se suelte
        if (HAL_GPIO_ReadPin(BOTON_1_GPIO_Port, BOTON_1_Pin) &&
            HAL_GPIO_ReadPin(BOTON_2_GPIO_Port, BOTON_2_Pin) &&
            HAL_GPIO_ReadPin(PERILLA_BOTON_GPIO_Port, PERILLA_BOTON_Pin))
        {
          break;
        }
      }
      return estado_actual;
    }

    // Actualizar el estado anterior
    estado_anterior = estado_actual;
  }
}

// Lee el estado de la perilla
int Leer_encoder(int maximo)
{
  int encoder;
  encoder = TIM4->CNT;
  return ((encoder / 2) % maximo) + 1;
}

// lee el encoder 1
int Leer_encoder1(void)
{
  return TIM2->CNT;
}

int Pantalla_menu_inicial(void)
{
  int opcion, boton;

  while (true)
  {
    opcion = Leer_encoder(3);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(42, 0);
    ssd1306_WriteString("inicial", Font_7x10, White);
    ssd1306_SetCursor(0, 18);
    if (opcion == 1)
    {
      ssd1306_WriteString("Instalador", Font_7x10, Black);
    }
    else
    {
      ssd1306_WriteString("Instalador", Font_7x10, White);
    }

    ssd1306_SetCursor(0, 36);
    if (opcion == 2)
    {
      ssd1306_WriteString("Monitoreo", Font_7x10, Black);
    }
    else
    {
      ssd1306_WriteString("Monitoreo", Font_7x10, White);
    }

    ssd1306_SetCursor(0, 54);
    if (opcion == 3)
    {
      ssd1306_WriteString("Ajustes", Font_7x10, Black);
    }
    else
    {
      ssd1306_WriteString("Ajustes", Font_7x10, White);
    }

    ssd1306_UpdateScreen();

    boton = Leer_botones();

    if (boton == ACEPTAR || boton == PERILLA)
    {
      break;
    }
  }
  return opcion;
}

int menu_monitoreo(void)
{
  int opcion;
  while (true)
  {
    opcion = Leer_encoder(2);

    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 18);

    if (opcion == 1)
    {
      ssd1306_WriteString("E/S digital", Font_7x10, Black);
    }
    else
    {
      ssd1306_WriteString("E/S digital", Font_7x10, White);
    }

    ssd1306_SetCursor(0, 36);
    if (opcion == 2)
    {
      ssd1306_WriteString("E/S analogica", Font_7x10, Black);
    }
    else
    {
      ssd1306_WriteString("E/S analogica", Font_7x10, White);
    }

    ssd1306_UpdateScreen();

    switch (Leer_botones())
    {
    case ACEPTAR:
    case PERILLA:
      return opcion;
      break;
    case CANCELAR:
      return NULO;
      break;
    }
  }
}

void monitoreo_digital(void)
{
  char valor_encoder[10] = {0};
  while (true)
  {

    ssd1306_Fill(Black);

    switch (Leer_encoder(2))
    {
    case 1:
      itoa(Leer_encoder1(), valor_encoder, 10);
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("E/S digital 1/2", Font_7x10, White);
      ssd1306_SetCursor(0, 18);
      ssd1306_WriteString("Encoder", Font_7x10, White);
      ssd1306_SetCursor(64, 18);
      ssd1306_WriteString(valor_encoder, Font_7x10, White);
      ssd1306_SetCursor(0, 36);
      ssd1306_WriteString("DI 1", Font_7x10, HAL_GPIO_ReadPin(DI_1_ENABLED_GPIO_Port, DI_1_ENABLED_Pin));
      ssd1306_SetCursor(0, 54);
      ssd1306_WriteString("DI 2", Font_7x10, HAL_GPIO_ReadPin(DI_2_PID_APAGADO_GPIO_Port, DI_2_PID_APAGADO_Pin));
      ssd1306_SetCursor(64, 36);
      ssd1306_WriteString("DI 3", Font_7x10, HAL_GPIO_ReadPin(DI_3_GPIO_Port, DI_3_Pin));
      ssd1306_SetCursor(64, 54);
      ssd1306_WriteString("DI 4", Font_7x10, HAL_GPIO_ReadPin(DI_4_GPIO_Port, DI_4_Pin));
      break;
    case 2:
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("E/S digital 2/2", Font_7x10, White);
      ssd1306_SetCursor(0, 18);
      ssd1306_WriteString("OK", Font_7x10, HAL_GPIO_ReadPin(DO_1_OK_GPIO_Port, DO_1_OK_Pin));
      ssd1306_SetCursor(64, 18);
      ssd1306_WriteString("ERR", Font_7x10, HAL_GPIO_ReadPin(DO_2_ERROR_GPIO_Port, DO_2_ERROR_Pin));
      break;
    }

    ssd1306_UpdateScreen();

    switch (Leer_botones())
    {
    case ACEPTAR:
    case CANCELAR:
      return;
      break;
    case PERILLA:
      break;
    }
  }
}

void monitoreo_analogico(void)
{
  char bufferAI1[10], bufferAI2[10], bufferAI3[10], bufferA0[10];
  while (true)
  {
    ssd1306_Fill(Black);
    if(Leer_encoder(2) == 1){
      itoa(AI1_leer() - offset_AI_1, bufferAI1, 10);


      	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
    /**/


      itoa(AI1_leer() - offset_AI_2, bufferAI2, 10);
      itoa(AI1_leer() - offset_AI_3, bufferAI3, 10);
      

      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("E/S analogica 1/2", Font_7x10, White);
      ssd1306_SetCursor(0, 18);
      ssd1306_WriteString("AI1", Font_7x10, White);
      ssd1306_SetCursor(64, 18);
      ssd1306_WriteString(bufferAI1, Font_7x10, White);
      ssd1306_SetCursor(0, 36);
      ssd1306_WriteString("AI2", Font_7x10, White);
      ssd1306_SetCursor(64, 36);
      ssd1306_WriteString(bufferAI2, Font_7x10, White);
      ssd1306_SetCursor(0, 54);
      ssd1306_WriteString("AI3", Font_7x10, White);
      ssd1306_SetCursor(64, 54);
      ssd1306_WriteString(bufferAI3, Font_7x10, White);
    } else {
      itoa(0 - offset_A0, bufferA0, 10);

      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("E/S analogica 2/2", Font_7x10, White);
      ssd1306_SetCursor(0, 18);
      ssd1306_WriteString("A0", Font_7x10, White);
      ssd1306_SetCursor(64, 18);
    }
    ssd1306_UpdateScreen();

    switch (Leer_botones())
    {
    case ACEPTAR:
    case CANCELAR:
      return;
    case PERILLA:
      break;
    }
  }
}

int menu_offsets(void)
{
  int opcion; //Opcion seleccionada en el encoder
  char buffer[6];

  while (true)
  {
    opcion = Leer_encoder(10);

    ssd1306_Fill(Black);

    if (opcion <= 3) //Mostrar primera pantalla
    {
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("Ajustes 1/4", Font_7x10, White); //Cabecera de la primera página
      ssd1306_SetCursor(0, 18);
      /*Mostrar opciones*/
      if (opcion == 1)
      {
        ssd1306_WriteString("Offset AI 1", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("Offset AI 1", Font_7x10, White);
      }
      itoa(offset_AI_1, buffer, 10); //Iniciar conversion
      ssd1306_SetCursor(90, 18);
      ssd1306_WriteString(buffer, Font_7x10, White);

      ssd1306_SetCursor(0, 36);
      if (opcion == 2)
      {
        ssd1306_WriteString("Offset AI 2", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("Offset AI 2", Font_7x10, White);
      }
      itoa(offset_AI_2, buffer, 10); //Iniciar conversion
      ssd1306_SetCursor(90, 36);
      ssd1306_WriteString(buffer, Font_7x10, White);

      ssd1306_SetCursor(0, 54);
      if (opcion == 3)
      {
        ssd1306_WriteString("Offset AI 3", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("Offset AI 3", Font_7x10, White);
      }
      itoa(offset_AI_3, buffer, 10); //Iniciar conversion
      ssd1306_SetCursor(90, 54);
      ssd1306_WriteString(buffer, Font_7x10, White);
    }
    else if(opcion > 3 && opcion <= 6)//Mostrar segunda pantalla
    {
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("Ajustes 2/4", Font_7x10, White); //Cabecera de la segunda página
      
      ssd1306_SetCursor(0, 18);
      if (opcion == 4)
      {
        ssd1306_WriteString("Offset A0", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("Offset A0", Font_7x10, White);
      }

      itoa(offset_A0, buffer, 10);
      ssd1306_SetCursor(90, 18);
      ssd1306_WriteString(buffer, Font_7x10, White);

      ssd1306_SetCursor(0, 36);
      if (opcion == 5)
      {
        ssd1306_WriteString("KP", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("KP", Font_7x10, White);
      }

      itoa(kp, buffer, 10);
      ssd1306_SetCursor(90, 36);
      ssd1306_WriteString(buffer, Font_7x10, White);

      ssd1306_SetCursor(0, 54);
      if (opcion == 6)
      {
        ssd1306_WriteString("KI", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("KI", Font_7x10, White);
      }

      itoa(ki, buffer, 10);
      ssd1306_SetCursor(90, 54);
      ssd1306_WriteString(buffer, Font_7x10, White);
    }
    else if(opcion > 6 && opcion <= 9)//Mostrar tercera pantalla.
    {
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("Ajustes 3/4", Font_7x10, White); //Cabecera de la segunda página
      
      ssd1306_SetCursor(0, 18);
      if (opcion == 7)
      {
        ssd1306_WriteString("KD", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("KD", Font_7x10, White);
      }

      itoa(kd, buffer, 10);
      ssd1306_SetCursor(90, 18);
      ssd1306_WriteString(buffer, Font_7x10, White);

      ssd1306_SetCursor(0, 36);
      if (opcion == 8) //Tiempo de Lazo
      {
        ssd1306_WriteString("TL", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("TL", Font_7x10, White);
      }

      itoa(tiempo_lazo, buffer, 10);
      ssd1306_SetCursor(90, 36);
      ssd1306_WriteString(buffer, Font_7x10, White);

      ssd1306_SetCursor(0, 54);
      if (opcion == 9)
      {
        ssd1306_WriteString("TD", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("TD", Font_7x10, White);
      }

      itoa(td, buffer, 10);
      ssd1306_SetCursor(90, 54);
      ssd1306_WriteString(buffer, Font_7x10, White);
    } else {
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("Ajustes 4/4", Font_7x10, White); //Cabecera de la segunda página
      
      ssd1306_SetCursor(0, 18);
      if (opcion == 10)
      {
        ssd1306_WriteString("TI", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("TI", Font_7x10, White);
      }
      itoa(ti, buffer, 10);
      ssd1306_SetCursor(90, 18);
      ssd1306_WriteString(buffer, Font_7x10, White);

    }

    ssd1306_UpdateScreen();

    switch (Leer_botones()) //Comprobar se se ha presionado un botón
    {
    case ACEPTAR:
    case PERILLA:
      return opcion; //Devolver la opción seleccionada 
      break;
    case CANCELAR:
      return NULO; //Devolver cancelado
    }
  }
}

/*Recive como entrada una cadena da caracteres, y la dirección a la variable a modificar, 
recibe como último argumento el valor máximo, retorna el valor seleccionado*/
int Establecer_valor_prueba(char *texto, int valor_inicial, int valor_maximo)
{
  int cifra[4];                       // Valores enteros
  char digitos[5];                    // Caracteres (más espacio para el terminador nulo '\0')
  char buffer_multiplicador[2] = {0}; // Solo se necesita 1 byte para un dígito
  char digitos_maximos[5];
  int boton, multiplicador = 0;
  int valor_actual = valor_inicial;
  Encoder_reiniciar();
  while (true)
  {
    int numero = valor_actual;

    for (int i = 3; i >= 0; i--)
    {
      cifra[i] = numero % 10;
      numero /= 10;
    }

    // Convertir los valores enteros en caracteres
    for (int i = 0; i < 4; i++)
    {
      digitos[i] = '0' + cifra[i];
    }
    digitos[4] = '\0'; // Agregar terminador nulo al final del arreglo

    cifra[multiplicador] = Leer_encoder(10) - 1;

    // Convertir los dígitos actualizados a un número entero
    numero = 0;
    for (int i = 0; i < 4; i++)
    {
      numero = numero * 10 + cifra[i];
    }

    valor_actual = numero;

    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(texto, Font_7x10, White);

    /*Mostrar cada dígito*/

    ssd1306_SetCursor(0, 18);
    if (multiplicador == 0)
    {
      ssd1306_WriteChar(digitos[0], Font_16x24, Black);
    }
    else
    {
      ssd1306_WriteChar(digitos[0], Font_16x24, White);
    }

    ssd1306_SetCursor(18, 18);

    if (multiplicador == 1)
    {
      ssd1306_WriteChar(digitos[1], Font_16x24, Black);
    }
    else
    {
      ssd1306_WriteChar(digitos[1], Font_16x24, White);
    }

    ssd1306_SetCursor(36, 18);
    if (multiplicador == 2)
    {
      ssd1306_WriteChar(digitos[2], Font_16x24, Black);
    }
    else
    {
      ssd1306_WriteChar(digitos[2], Font_16x24, White);
    }

    if (multiplicador == 3)
    {
      ssd1306_WriteChar(digitos[3], Font_16x24, Black);
    }
    else
    {
      ssd1306_WriteChar(digitos[3], Font_16x24, White);
    }

    ssd1306_SetCursor(0, 54);
    ssd1306_WriteString("Digito", Font_7x10, White);
    ssd1306_SetCursor(100, 54);
    itoa(multiplicador + 1, buffer_multiplicador, 10);
    ssd1306_WriteString(buffer_multiplicador, Font_7x10, White);
    ssd1306_UpdateScreen();

    boton = Leer_botones();

    switch (boton)
    {
    case PERILLA: //Cambiar el dígito a editar 
      Encoder_reiniciar();
      if (multiplicador >= 3)
      {
        multiplicador = 0;
      }
      else
      {
        multiplicador++;
      }
      break;
    case ACEPTAR:
      if (valor_actual <= valor_maximo) //Devulver el valor sí establecido si es correcto
      {
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 18);
        ssd1306_WriteString("Guardado", Font_7x10, White);
        ssd1306_UpdateScreen();
        HAL_Delay(1000);
        valor_inicial = valor_actual;
        return valor_actual;
      }
      else //Volver a pedir el valor si es incorrecto
      {
        itoa(valor_maximo, digitos_maximos, 10);
        digitos_maximos[4] = '\0';
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 18);
        ssd1306_WriteString("Error, el valor", Font_7x10, White);
        ssd1306_SetCursor(0, 36);
        ssd1306_WriteString("maximo es", Font_7x10, White);
        ssd1306_SetCursor(70, 36);
        ssd1306_WriteString(digitos_maximos, Font_7x10, White);
        ssd1306_UpdateScreen();
        HAL_Delay(1000);
        break;
      }
    case CANCELAR: //Cancelar y devolver el valor anterior
      ssd1306_Fill(Black);
      ssd1306_SetCursor(0, 18);
      ssd1306_WriteString("Cancelado", Font_7x10, White);
      ssd1306_UpdateScreen();
      HAL_Delay(1000);
      return valor_inicial;
    }
  }
}


/*Recive como entrada una cadena da caracteres, y la dirección a la variable a modificar, 
recibe como último argumento el valor máximo, retorna el valor seleccionado*/
int Establecer_valor(char *texto, int *variable, int valor_maximo)
{
  int cifra[4];                       // Valores enteros
  char digitos[5];                    // Caracteres (más espacio para el terminador nulo '\0')
  char buffer_multiplicador[2] = {0}; // Solo se necesita 1 byte para un dígito
  char digitos_maximos[5];
  int boton, multiplicador = 0;
  int valor_actual = *variable;
  Encoder_reiniciar();
  while (true)
  {
    int numero = valor_actual;

    //Llenar el array con números decimales
    for (int i = 3; i >= 0; i--)
    {
      cifra[i] = numero % 10;
      numero /= 10;
    }

    // Convertir los valores enteros en caracteres
    for (int i = 0; i < 4; i++)
    {
      digitos[i] = '0' + cifra[i];
    }
    digitos[4] = '\0'; // Agregar terminador nulo al final del arreglo

    cifra[multiplicador] = Leer_encoder(10) - 1;

    // Convertir los dígitos actualizados a un número entero
    numero = 0;
    for (int i = 0; i < 4; i++)
    {
      numero = numero * 10 + cifra[i];
    }

    valor_actual = numero;

    //Mostrar la cabecera en pantalla
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(texto, Font_7x10, White);

    /*Mostrar cada dígito*/

    ssd1306_SetCursor(0, 18);
    if (multiplicador == 0)
    {
      ssd1306_WriteChar(digitos[0], Font_16x24, Black);
    }
    else
    {
      ssd1306_WriteChar(digitos[0], Font_16x24, White);
    }

    ssd1306_SetCursor(18, 18);

    if (multiplicador == 1)
    {
      ssd1306_WriteChar(digitos[1], Font_16x24, Black);
    }
    else
    {
      ssd1306_WriteChar(digitos[1], Font_16x24, White);
    }

    ssd1306_SetCursor(36, 18);
    if (multiplicador == 2)
    {
      ssd1306_WriteChar(digitos[2], Font_16x24, Black);
    }
    else
    {
      ssd1306_WriteChar(digitos[2], Font_16x24, White);
    }

    if (multiplicador == 3)
    {
      ssd1306_WriteChar(digitos[3], Font_16x24, Black);
    }
    else
    {
      ssd1306_WriteChar(digitos[3], Font_16x24, White);
    }
    /*
    ssd1306_SetCursor(0, 54);
    ssd1306_WriteString("Digito", Font_7x10, White);
    ssd1306_SetCursor(100, 54);
    itoa(multiplicador + 1, buffer_multiplicador, 10);
    ssd1306_WriteString(buffer_multiplicador, Font_7x10, White);
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 18);
    */
    ssd1306_UpdateScreen();

    //Ver qué botón se ha presionado
    switch (Leer_botones()) 
    {
    case PERILLA: //Cambiar el dígito a editar 
      Encoder_reiniciar();
      multiplicador = (multiplicador >= 3) ? 0 : (multiplicador + 1);

      break;
    case ACEPTAR:
      if (valor_actual <= valor_maximo) //Devolver el valor sí establecido si es correcto
      {
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 18);
        ssd1306_WriteString("Guardado", Font_7x10, White);
        ssd1306_UpdateScreen();
        HAL_Delay(1000);
        *variable = valor_actual; //Modificar el valor de la variable introducida
        return valor_actual;
      }
      else //Volver a pedir el valor si es incorrecto
      {
        itoa(valor_maximo, digitos_maximos, 10);
        digitos_maximos[4] = '\0';
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 18);
        ssd1306_WriteString("Error, el valor", Font_7x10, White);
        ssd1306_SetCursor(0, 36);
        ssd1306_WriteString("maximo es", Font_7x10, White);
        ssd1306_SetCursor(70, 36);
        ssd1306_WriteString(digitos_maximos, Font_7x10, White);
        ssd1306_UpdateScreen();
        HAL_Delay(1000);
        break;
      }
    case CANCELAR: //Cancelar y devolver el valor anterior
      ssd1306_Fill(Black);
      ssd1306_SetCursor(0, 18);
      ssd1306_WriteString("Cancelado", Font_7x10, White);
      ssd1306_UpdateScreen();
      HAL_Delay(1000);
      return *variable;
    }
  }
}


int Establecer_valor_bidireccional(char *texto, int valor_inicial, int valor_maximo)
{
  int cifra[4];                       // Valores enteros
  char digitos[6];                    // Caracteres (más espacio para el terminador nulo '\0' y el signo)
  char buffer_multiplicador[2] = {0}; // Solo se necesita 1 byte para un dígito
  char digitos_maximos[5];
  int boton, multiplicador = 0;
  int valor_actual = valor_inicial;
  bool negativo = false;
  Encoder_reiniciar();

  while (true)
  {
    int numero = valor_actual;

    for (int i = 3; i >= 0; i--)
    {
      cifra[i] = numero % 10;
      numero /= 10;
    }

    // Convertir los valores enteros en caracteres
    for (int i = 0; i < 4; i++)
    {
      digitos[i + 1] = '0' + cifra[i];
    }

    if (multiplicador == 0)
    {
      digitos[0] = negativo ? '-' : '+';
    }
    else
    {
      digitos[0] = ' ';
    }

    digitos[5] = '\0'; // Agregar terminador nulo al final del arreglo

    cifra[multiplicador] = Leer_encoder(10) - 1;

    if (multiplicador == 0)
    {
      switch (Leer_encoder(2))
      {
      case 0:
        negativo = !negativo; // Cambiar el estado del signo
        break;
      case 1:
        negativo = false; // Establecer el signo como positivo
        break;
      case 2:
        negativo = true; // Establecer el signo como negativo
        break;
      }
    }

    // Convertir los dígitos actualizados a un número entero
    numero = 0;
    for (int i = 0; i < 4; i++)
    {
      numero = numero * 10 + cifra[i];
    }

    if (negativo)
    {
      numero *= -1;
    }

    valor_actual = numero;

    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(texto, Font_7x10, White);

    /*Mostrar cada dígito*/

    for (int i = 0; i < 4; i++)
    {
      ssd1306_SetCursor(i * 18, 18);

      if (multiplicador == i)
      {
        ssd1306_WriteChar(digitos[i], Font_16x24, Black);
      }
      else
      {
        ssd1306_WriteChar(digitos[i], Font_16x24, White);
      }
    }

    ssd1306_SetCursor(0, 54);
    ssd1306_WriteString("Digito", Font_7x10, White);
    ssd1306_SetCursor(100, 54);
    itoa(multiplicador + 1, buffer_multiplicador, 10);
    ssd1306_WriteString(buffer_multiplicador, Font_7x10, White);
    ssd1306_UpdateScreen();

    boton = Leer_botones();

    switch (boton)
    {
    case PERILLA: //Cambiar el dígito a editar
      Encoder_reiniciar();
      if (multiplicador >= 3)
      {
        multiplicador = 0;
      }
      else
      {
        multiplicador++;
      }
      break;
    case ACEPTAR:
      if (valor_actual <= valor_maximo) //Devolver el valor sí establecido si es correcto
      {
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 18);
        ssd1306_WriteString("Guardado", Font_7x10, White);
        ssd1306_UpdateScreen();
        HAL_Delay(1000);
        return valor_actual;
      }
      else //Volver a pedir el valor si es incorrecto
      {
        itoa(valor_maximo, digitos_maximos, 10);
        digitos_maximos[4] = '\0';
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 18);
        ssd1306_WriteString("Error, el valor", Font_7x10, White);
        ssd1306_SetCursor(0, 36);
        ssd1306_WriteString("maximo es", Font_7x10, White);
        ssd1306_SetCursor(70, 36);
        ssd1306_WriteString(digitos_maximos, Font_7x10, White);
        ssd1306_UpdateScreen();
        HAL_Delay(1000);
        break;
      }
    case CANCELAR: //Cancelar y devolver el valor anterior
      ssd1306_Fill(Black);
      ssd1306_SetCursor(0, 18);
      ssd1306_WriteString("Cancelado", Font_7x10, White);
      ssd1306_UpdateScreen();
      HAL_Delay(1000);
      return valor_inicial;
    }
  }
}


#if PRUEBA_DAC

int Establecer_valor_DAC(char *texto, int valor_inicial, int valor_maximo)
{
  int cifra[4];                       // Valores enteros
  char digitos[5];                    // Caracteres (más espacio para el terminador nulo '\0')
  char buffer_multiplicador[2] = {0}; // Solo se necesita 1 byte para un dígito
  char digitos_maximos[5];
  int boton, multiplicador = 0;
  int valor_actual = valor_inicial;
  Encoder_reiniciar();
  while (true)
  {
    int numero = valor_actual;

    for (int i = 3; i >= 0; i--)
    {
      cifra[i] = numero % 10;
      numero /= 10;
    }

    // Convertir los valores enteros en caracteres
    for (int i = 0; i < 4; i++)
    {
      digitos[i] = '0' + cifra[i];
    }
    digitos[4] = '\0'; // Agregar terminador nulo al final del arreglo

    cifra[multiplicador] = Leer_encoder(10) - 1;

    // Convertir los dígitos actualizados a un número entero
    numero = 0;
    for (int i = 0; i < 4; i++)
    {
      numero = numero * 10 + cifra[i];
    }

    valor_actual = numero;

    HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2 , DAC_ALIGN_12B_R, numero);

    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(texto, Font_7x10, White);

    /*Mostrar cada dígito*/

    ssd1306_SetCursor(0, 18);
    if (multiplicador == 0)
    {
      ssd1306_WriteChar(digitos[0], Font_16x24, Black);
    }
    else
    {
      ssd1306_WriteChar(digitos[0], Font_16x24, White);
    }

    ssd1306_SetCursor(18, 18);

    if (multiplicador == 1)
    {
      ssd1306_WriteChar(digitos[1], Font_16x24, Black);
    }
    else
    {
      ssd1306_WriteChar(digitos[1], Font_16x24, White);
    }

    ssd1306_SetCursor(36, 18);
    if (multiplicador == 2)
    {
      ssd1306_WriteChar(digitos[2], Font_16x24, Black);
    }
    else
    {
      ssd1306_WriteChar(digitos[2], Font_16x24, White);
    }

    if (multiplicador == 3)
    {
      ssd1306_WriteChar(digitos[3], Font_16x24, Black);
    }
    else
    {
      ssd1306_WriteChar(digitos[3], Font_16x24, White);
    }

    ssd1306_SetCursor(0, 54);
    ssd1306_WriteString("Digito", Font_7x10, White);
    ssd1306_SetCursor(100, 54);
    itoa(multiplicador + 1, buffer_multiplicador, 10);
    ssd1306_WriteString(buffer_multiplicador, Font_7x10, White);
    ssd1306_UpdateScreen();

    boton = Leer_botones();

    switch (boton)
    {
    case PERILLA: //Cambiar el dígito a editar 
      Encoder_reiniciar();
      if (multiplicador >= 3)
      {
        multiplicador = 0;
      }
      else
      {
        multiplicador++;
      }
      break;
    case ACEPTAR:
      if (valor_actual <= valor_maximo) //Devulver el valor sí establecido si es correcto
      {
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 18);
        ssd1306_WriteString("Guardado", Font_7x10, White);
        ssd1306_UpdateScreen();
        HAL_Delay(1000);
        return valor_actual;
      }
      else //Volver a pedir el valor si es incorrecto
      {
        itoa(valor_maximo, digitos_maximos, 10);
        digitos_maximos[4] = '\0';
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 18);
        ssd1306_WriteString("Error, el valor", Font_7x10, White);
        ssd1306_SetCursor(0, 36);
        ssd1306_WriteString("maximo es", Font_7x10, White);
        ssd1306_SetCursor(70, 36);
        ssd1306_WriteString(digitos_maximos, Font_7x10, White);
        ssd1306_UpdateScreen();
        HAL_Delay(1000);
        break;
      }
    case CANCELAR: //Cancelar y devolver el valor anterior
      ssd1306_Fill(Black);
      ssd1306_SetCursor(0, 18);
      ssd1306_WriteString("Cancelado", Font_7x10, White);
      ssd1306_UpdateScreen();
      HAL_Delay(1000);
      return valor_inicial;
    }
  }
}

#endif


int menu_instalador(void)
{
  int opcion; //Opcion seleccionada en el encoder
  char buffer[6];

  while (true)
  {
    opcion = Leer_encoder(4);

    ssd1306_Fill(Black);

    if (opcion <= 3) //Mostrar primera pantalla
    {
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("Instalador 1/2", Font_7x10, White); //Cabecera de la primera página
      ssd1306_SetCursor(0, 18);
      /*Mostrar opciones*/
      if (opcion == 1)
      {
        ssd1306_WriteString("Prom AI 1", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("Prom AI 1", Font_7x10, White);
      }
      itoa(promediador_1, buffer, 10); //Iniciar conversiones
      ssd1306_SetCursor(90, 18);
      ssd1306_WriteString(buffer, Font_7x10, White);

      ssd1306_SetCursor(0, 36);
      if (opcion == 2)
      {
        ssd1306_WriteString("Prom AI 2", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("Prom AI 2", Font_7x10, White);
      }
      itoa(promediador_2, buffer, 10);
      ssd1306_SetCursor(90, 36);
      ssd1306_WriteString(buffer, Font_7x10, White);

      ssd1306_SetCursor(0, 54);
      if (opcion == 3)
      {
        ssd1306_WriteString("Prom AI 3", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("Prom AI 3", Font_7x10, White);
      }
      itoa(promediador_3, buffer, 10);
      ssd1306_SetCursor(90, 54);
      ssd1306_WriteString(buffer, Font_7x10, White);
    }
    else if(opcion > 3 && opcion <= 6)  //Mostrar segunda pantalla
    {
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("Instalador 2/2", Font_7x10, White); //Cabecera de la segunda página
      
      ssd1306_SetCursor(0, 18);
      if (opcion == 4)
      {
        ssd1306_WriteString("Filtro dig", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("Filtro dig", Font_7x10, White);
      }

      itoa(tiempo_filtro_digital, buffer, 10);
      ssd1306_SetCursor(90, 18);
      ssd1306_WriteString(buffer, Font_7x10, White);
    }

    ssd1306_UpdateScreen();

    switch (Leer_botones()) //Comprobar se se ha presionado un botón
    {
    case ACEPTAR:
    case PERILLA:
      return opcion; //Devolver la opción seleccionada 
      break;
    case CANCELAR:
      return NULO; //Devolver cancelado
    }
  }
}


int AI1_leer(void) //Leer el ADC sospechoso
{
  int adcValue = 0;

  // Start the ADC conversion
  HAL_ADC_Start(&hadc1);

  // Wait for the conversion to complete
  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
  {
    // Read the ADC value
    adcValue = HAL_ADC_GetValue(&hadc1);
  }

  // Stop the ADC conversion
  HAL_ADC_Stop(&hadc1);

  return adcValue;
}

int menu_instalador_1(void)
{
  int opcion;
  while (true)
  {
    opcion = Leer_encoder(8);

    ssd1306_Fill(Black);

    if(opcion <= 3){
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("instalador 1/3", Font_7x10, White); //Cabecera de la primera página
      
      ssd1306_SetCursor(0, 18);
      if (opcion == 1)
      {
        ssd1306_WriteString("Salida unipolar", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("Salida unipolar", Font_7x10, White);
      }

      ssd1306_SetCursor(0, 36);
      if (opcion == 2)
      {
        ssd1306_WriteString("KP I abierto", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("KP I abierto", Font_7x10, White);
      }

      ssd1306_SetCursor(0, 54);
      if (opcion == 3)
      {
        ssd1306_WriteString("PID Salida INV", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("PID Salida INV", Font_7x10, White);
      }
    } else if(opcion > 3 && opcion <= 6){

      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("instalador 2/3", Font_7x10, White); //Cabecera de la primera página
      
      ssd1306_SetCursor(0, 18);
      if (opcion == 4)
      {
        ssd1306_WriteString("% PID positivo", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("% PID positivo", Font_7x10, White);
      }

      ssd1306_SetCursor(0, 36);
      if (opcion == 5)
      {
        ssd1306_WriteString("% PID negativo", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("% PID negativo", Font_7x10, White);
      }

      ssd1306_SetCursor(0, 54);
      if (opcion == 6)
      {
        ssd1306_WriteString("Suma PID con ref", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("Suma PID con ref", Font_7x10, White);
      }
    } else if(opcion > 6 && opcion <= 9){
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("instalador 3/3", Font_7x10, White); //Cabecera de la primera página

      ssd1306_SetCursor(0, 18);
      if (opcion == 7)
      {
        ssd1306_WriteString("Lim salida analog +", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("Lim salida analog +", Font_7x10, White);
      }

      ssd1306_SetCursor(0, 36);
      if (opcion == 8)
      {
        ssd1306_WriteString("Lim salida analog -", Font_7x10, Black);
      }
      else
      {
        ssd1306_WriteString("Lim salida analog -", Font_7x10, White);
      }
    }
    ssd1306_UpdateScreen();

    switch (Leer_botones())
    {
    case ACEPTAR:
    case PERILLA:
      return opcion;
    case CANCELAR:
      return NULO;
    }
  }
}

bool cambiar_opcion(char *texto ,bool *valor){
  int opcion;
  while(true){
    opcion = Leer_encoder(2);

    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(texto, Font_7x10, White);
    ssd1306_SetCursor(0, 18);

    if(*valor == true){
      ssd1306_WriteString("Valor inicial: V", Font_7x10, White);
    } else
    {
      ssd1306_WriteString("Valor inicial: F", Font_7x10, White);
    }
    
    if (opcion % 2 == 0)
    {
      ssd1306_SetCursor(0, 36);
      ssd1306_WriteString("Verdadero", Font_7x10, Black);
      ssd1306_SetCursor(70, 36);
      ssd1306_WriteString("Falso", Font_7x10, White);
    } else
    {
      ssd1306_SetCursor(0, 36);
      ssd1306_WriteString("Verdadero", Font_7x10, White);
      ssd1306_SetCursor(70, 36);
      ssd1306_WriteString("Falso", Font_7x10, Black);
    }
    ssd1306_UpdateScreen();
    switch (Leer_botones())
    {
    case ACEPTAR:
    case PERILLA:
      *valor = (opcion % 2 == 0) ? true : false;
      pantalla_guardado();
      return *valor;
    case CANCELAR:
      pantalla_cancelar();
      return *valor;
    } 
  }
}

void pantalla_cancelar(void){
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 18);
  ssd1306_WriteString("Cancelado", Font_7x10, White);
  ssd1306_UpdateScreen();
  HAL_Delay(500);
}

void pantalla_guardado(void){
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 18);
  ssd1306_WriteString("Guardado", Font_7x10, White);
  ssd1306_UpdateScreen();
  HAL_Delay(500);
}

void Encoder_reiniciar(void)
{
  TIM4->CNT = 1000;
  return;
}

void save_data(uint32_t Address,uint32_t data){

    HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = Address;
	EraseInitStruct.NbPages = 1;

	uint32_t PageError;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)		//Erase the Page Before a Write Operation
			return;

	HAL_Delay(500);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST,Address,(uint64_t)data);
	HAL_Delay(500);
	HAL_FLASH_Lock();
}

uint32_t read_data_address(uint32_t Address){

	__IO uint32_t data_read = *(__IO uint32_t *)Address;
	return (uint32_t)data_read;
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
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_WriteString("Error", Font_7x10, Black);
    ssd1306_UpdateScreen();
    HAL_Delay(10000);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.    HAL_Delay(10);
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
