/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//------------------------------------------------------------------------ MAKRA
#define REF_VOLTAGE_MV 3250.0f // Napiecie referencyjne w mV (3.25V)

// Współczynnik dla dzielników pomiarowych: ((Ru+Rl)/Rl) * (REF_VOLTAGE_CV / 4095.0f)
#define U_SENS_COEFF_MV (((220.0f + 12.0f) / 12.0f) * (REF_VOLTAGE_MV / 4095.0f))

// Bezpośredni współczynnik pinów MCU:
#define ADC_VOLTAGE_COEFF_MV (REF_VOLTAGE_MV / 4095.0f)

// Współczynnik dla czujników ACS37030: 20.3mV/A
#define I_SENS_COEFF_MA ((ADC_VOLTAGE_COEFF_MV / 20.3f) * 1000.0f)

// Makro konwertujące dla czujnika TMP235 (wynik w rozdzielczości 0.1°C)
// Parametry sprzętowe: Offset = 500 mV, Czułość = 10 mV/°C
#define TMP235_ADC_TO_0C1(adc_raw)  (((adc_raw) * ADC_VOLTAGE_COEFF_MV) - 500.0f)

#define DC_BUS_OVP_THRESHOLD_MV 60000 // Próg przepięcia: 60.00 V
#define DC_BUS_OCP_THRESHOLD_MA 50000 // Próg nadprądowy: 50.00 A (na przyszłość)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//------------------------------------------------------------------------ DIAGNOSTYKA I BŁĘDY (Styl ST MC SDK)
#define FAULT_NONE              	((uint16_t)0x0000)
#define FAULT_OVER_CURR         	((uint16_t)0x0001)
#define FAULT_OVER_VOLT         	((uint16_t)0x0002)
#define FAULT_PRECHARGE_TIMEOUT 	((uint16_t)0x0004)
#define FAULT_SDC_FEEDBACK      	((uint16_t)0x0008)
// [...]
#define FAULT_CORDIC_FLAG_TIMEOUT	((uint16_t)0x0010)

volatile uint16_t Active_Faults = FAULT_NONE; // Globalny rejestr usterkowy

// Status wykonania samej powłoki funkcji
typedef enum {
    	EXEC_SUCCESS = 0,
    	EXEC_ERROR
} FuncStatus;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

//------------------------------------------------------------------------ ADC
// Definicja buforów i pomiarów składowych

typedef enum {
   		IHB_TEMP_SENS,	// CH14

    	ADC1_ACTIVE_CH_NUM
} ADC1_measurements;

uint8_t current_adc1_measurement;
volatile int32_t ADC1_buff[ADC1_ACTIVE_CH_NUM];


typedef enum {
    	PH1_U_SENS,		// CH12
    	BATT_U_SENS,	// CH13
    	DC_BUS_U_SENS,	// CH17

    	ADC2_ACTIVE_CH_NUM,

   		PH2_I_SENS,		// Nieobsługiwane w buforze ADC2_buff
    	PH2_I_SENS_N 	//
} ADC2_measurements;

uint8_t current_adc2_measurement;
volatile int32_t ADC2_buff[ADC2_ACTIVE_CH_NUM];
// ADC2_buff jest asynchronicznie odświeżany w przerwaniu ADC2_IRQHandler


// Zmienne do sprzętowego SPWM ----------------------------------------------------------|
uint32_t phase_acc = 0;         // Akumulator fazy (od 0 do 0xFFFFFFFF)
uint32_t phase_step = 0;        // Wyliczyne w main()
// ====================================================================
float spwm_freq_hz = 50.0f;      // <--- ZMIANA CZĘSTOTLIWOŚCI (1.0 -> 1Hz)
float mod_index = 0.82f;         // <--- ZMIANA AMPLITUDY (0.1 -> 10%)
// ====================================================================
uint16_t tim1_arr_value = 0;         // Zmienna przechowująca maksymalne wypełnienie


// Zmienna do procedury soft start ---------------------------------------------------------------|
float soft_start_end_index = 0.0f;


// TESTOWANIE POMIARÓW ADC:
	  //int32_t Pomiar_pradu_DC = 0;
	  //int32_t Pomiar_pradu_FAZA1 = 0;
	  int32_t Pomiar_pradu_FAZA2 = 0;
	  //int32_t Pomiar_pradu_FAZA3 = 0;
	  int32_t Pomiar_napiecia_BATT = 0;
	  int32_t Pomiar_napiecia_DC = 0;
	  int32_t Pomiar_napiecia_FAZA1 = 0;
	  //int32_t Pomiar_napiecia_FAZA2 = 0;
	  //int32_t Pomiar_napiecia_FAZA3 = 0;
	  int32_t Pomiar_temp_stopnia_mocy = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

//------------------------------------------------------------------------ PROTOTYPY FUNKCJI
FuncStatus Inverter_startup_procedure(void);
FuncStatus DC_BUS_charge_monitor(void);
FuncStatus Check_OCP_status(void);
FuncStatus Check_OVP_status(void);
int32_t adc1_2val(ADC1_measurements adc1_meas);
int32_t adc2_2val(ADC2_measurements adc2_meas);
void Enable_PWM_signals(void);

void TIM1_UP_TIM16_IRQHandler(void);
void ADC1_2_IRQHandler(void);

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CORDIC_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  //------------------------------------------------------------------------ MAIN

  //=========================================================================|| Potrzebne w celu generowania modulacji PWM

  // Pobranie wartości ARR z konfiguracji timera
  tim1_arr_value = TIM1->ARR;

  // Dynamiczne wyliczenie częstotliwości przerwań TIM1 (w Hz).
  // SystemCoreClock to wbudowana zmienna CMSIS przechowująca aktualne taktowanie (zazwyczaj 170 MHz).
  // W trybie Center-Aligned pełny okres to 2 * ARR. Przerwanie Update zależy też od dzielników PSC i RCR.
  uint32_t tim1_update_freq_hz = SystemCoreClock / ( (TIM1->PSC + 1) * tim1_arr_value * (TIM1->RCR + 1) );

  // Automatyczne wyliczenie kroku dla sprzętowego akumulatora fazy DDS
  phase_step = (uint32_t)((spwm_freq_hz / (float)tim1_update_freq_hz) * 4294967296.0f);

  // Konfiguracja koprocesora CORDIC dla obliczania funkcji Sinus
  // Format wejściowy q1.31, format wyjściowy q1.31, najwyższa precyzja (6 cykli)
  LL_CORDIC_Config(CORDIC,
                   LL_CORDIC_FUNCTION_SINE,
                   LL_CORDIC_PRECISION_6CYCLES,
                   LL_CORDIC_SCALE_0,
                   LL_CORDIC_NBWRITE_1,
                   LL_CORDIC_NBREAD_1,
                   LL_CORDIC_INSIZE_32BITS,
                   LL_CORDIC_OUTSIZE_32BITS);

  //=========================================================================||

  // Konfiguracja zmiennych pod procedure soft start
  soft_start_end_index = mod_index;
  mod_index = 0.01f;

  //=========================================================================||
  	//ADC1:
    // a. Kalibracja przetwornika (krytyczna w STM32G4 dla dokładności pomiarów)
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0) {}
    // b. Aktywacja przetwornika ADC2
    LL_ADC_Enable(ADC1);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0) {}
    // c. Odblokowanie przerwania od końca konwersji (EOC)
    LL_ADC_EnableIT_EOC(ADC1);
    // d. Ręczne wyzwolenie pierwszej konwersji w tle (Software Trigger)
    LL_ADC_REG_StartConversion(ADC1);

    //ADC2:
    // a1. Kalibracja przetworników single
    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0) {}
    // a2. Kalibracja przetwornika diff
    LL_ADC_StartCalibration(ADC2, LL_ADC_DIFFERENTIAL_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0) {}
    // b. Aktywacja przetwornika ADC2
    LL_ADC_Enable(ADC2);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0) {}
    	// === KRYTYCZNA POPRAWKA DLA STM32G4 ===
        // Nadpisanie rejestru JSQR (zablokowanego przed komendą LL_ADC_Enable)
        // Konfiguracja sprzętowego nasłuchu na kanał 3 (PA6/PA7 - prąd fazy V [1])
        LL_ADC_INJ_SetTriggerSource(ADC2, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO);
        LL_ADC_INJ_SetTriggerEdge(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);
        LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_3);
        // ======================================
    // c. Uzbrojenie pomiaru wstrzykniętego (Przetwornik czeka w tle na impuls z TIM1)
    LL_ADC_INJ_StartConversion(ADC2);
    // d. Odblokowanie przerwania od końca konwersji (EOC)
    LL_ADC_EnableIT_EOC(ADC2);
    // e. Ręczne wyzwolenie pierwszej konwersji w tle (Software Trigger)
    LL_ADC_REG_StartConversion(ADC2);


    /* TEST PROCEDURY STARTOWEJ BARE-METAL */
    if (Inverter_startup_procedure() == EXEC_ERROR) //<---------------------------------- EXEC_ERROR -> [DEBUG ONLY]
    {
    	// 0. Start poprawny - zapalenie zielonego LEDa testowego
        HAL_GPIO_WritePin(GPIOB, STATUS_LED_Pin, GPIO_PIN_SET);

        // 1. Wyczyszczenie sprzętowej flagi Break (BIF), która zatrzasnęła się
        // podczas startu mikrokontrolera z powodu domyślnego stanu 0 na przerzutnikach OCP/OVP.
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
        TIM1->BDTR &= ~TIM_BDTR_BKE; // Wyłączenie wejścia Break (Break Disable) -------- [DEBUG ONLY]

        // 2. Włączenie timera od procedury soft startu
        HAL_TIM_Base_Start_IT(&htim7);

        // 3. Włączenie sygnałów PWM (od tego momentu bit MOE bezpiecznie ustawi się na 1)
        Enable_PWM_signals();
    }
    else
    {
        // Błąd procedury - zabezpieczenie sprzętu i wystawienie sygnału błędu
        HAL_GPIO_WritePin(GPIOA, HB_EN_n_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_SET);


        // Dekodowanie i obsługa błędów z rejestru bitowego
        if (Active_Faults & FAULT_OVER_CURR)
        {
      	  // Logika reakcji na zadziałanie komparatora prądowego (OCP)
        }
        if (Active_Faults & FAULT_OVER_VOLT)
        {
            // Logika reakcji na przekroczenie napięcia 60V (OVP)
        }
        if (Active_Faults & FAULT_PRECHARGE_TIMEOUT)
        {
            // Logika po nieudanym naładowaniu rezystora 180R
        }
        if (Active_Faults & FAULT_SDC_FEEDBACK)
        {
            // Usterka izolatora fotowoltaicznego lub zwarcie mosfetów 'przekaźnika'
        }
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // TESTOWANIE POMIARÓW ADC:

	  //Pomiar_pradu_DC
	  //Pomiar_pradu_FAZA1
	  Pomiar_pradu_FAZA2 = adc2_2val(PH2_I_SENS);
	  //Pomiar_pradu_FAZA3
	  Pomiar_napiecia_BATT = adc2_2val(BATT_U_SENS);
	  Pomiar_napiecia_DC = adc2_2val(DC_BUS_U_SENS);
	  Pomiar_napiecia_FAZA1 = adc2_2val(PH1_U_SENS);
	  //Pomiar_napiecia_FAZA2
	  //Pomiar_napiecia_FAZA3
	  Pomiar_temp_stopnia_mocy = adc1_2val(IHB_TEMP_SENS);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
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

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**ADC1 GPIO Configuration
  PB11   ------> ADC1_IN14
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ADC1 interrupt Init */
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(ADC1_2_IRQn);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC1, 0);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
  LL_ADC_INJ_Init(ADC1, &ADC_INJ_InitStruct);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_14);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_14, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_14, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**ADC2 GPIO Configuration
  PA4   ------> ADC2_IN17
  PA5   ------> ADC2_IN13
  PA6   ------> ADC2_IN3
  PA7   ------> ADC2_IN4
  PB2   ------> ADC2_IN12
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ADC2 interrupt Init */
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(ADC1_2_IRQn);

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC2, 0);
  LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);
  ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO;
  ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_DISABLE;
  ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
  LL_ADC_INJ_Init(ADC2, &ADC_INJ_InitStruct);
  LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_DISABLE);
  LL_ADC_INJ_SetTriggerEdge(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC2);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC2);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_12);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_12CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_12, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_13);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_13, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_13, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_17);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_17, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_17, LL_ADC_SINGLE_ENDED);

  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_3);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_12CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_3, LL_ADC_DIFFERENTIAL_ENDED);
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */

  /* nothing else to be configured */

  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4249;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_LOW;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 3;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 4;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 8499;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DC_BUS_OC_RST_Pin|DC_BUS_OV_RST_Pin|PRC_RST_n_Pin|DC_BUS_READY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HB_EN_n_GPIO_Port, HB_EN_n_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SDC_RELAY_ON_Pin */
  GPIO_InitStruct.Pin = SDC_RELAY_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDC_RELAY_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_BUS_OC_RST_Pin DC_BUS_OV_RST_Pin PRC_RST_n_Pin DC_BUS_READY_Pin */
  GPIO_InitStruct.Pin = DC_BUS_OC_RST_Pin|DC_BUS_OV_RST_Pin|PRC_RST_n_Pin|DC_BUS_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : HB_EN_n_Pin */
  GPIO_InitStruct.Pin = HB_EN_n_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HB_EN_n_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PRC_RELAY_ON_Pin */
  GPIO_InitStruct.Pin = PRC_RELAY_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PRC_RELAY_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DC_BUS_OV_n_Pin */
  GPIO_InitStruct.Pin = DC_BUS_OV_n_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DC_BUS_OV_n_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//------------------------------------------------------------------------ DEFINICJE FUNKCJI

int32_t adc1_2val(ADC1_measurements adc1_meas)
{
    	switch(adc1_meas)
    	{
    		case IHB_TEMP_SENS:	return (int32_t)TMP235_ADC_TO_0C1(ADC1_buff[adc1_meas]);
        	default:            return 0;
    	}
}

int32_t adc2_2val(ADC2_measurements adc2_meas)
{
    	switch(adc2_meas)
    	{
    		case PH2_I_SENS:
    			// Odczyt bezpośrednio z rejestru wstrzykniętego (Injected Rank 1) [2]
    			// W trybie różnicowym STM32 zwraca 2048 dla prądu 0A. Odejmując 2048,
    			// otrzymujemy surową różnicę do przetworzenia przez stałą I_SENS_COEFF_MA.
    			int32_t raw_diff = (int32_t)LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1) - 2048;
    			return (int32_t)(raw_diff * I_SENS_COEFF_MA);
    	    case PH2_I_SENS_N:	return 0;

    		// Konwersja statyczna z uint16_t z przerwaniowego bufora do cV:
        	case PH1_U_SENS:    return (int32_t)(ADC2_buff[adc2_meas] * U_SENS_COEFF_MV);
        	case BATT_U_SENS:   return (int32_t)(ADC2_buff[adc2_meas] * U_SENS_COEFF_MV);
        	case DC_BUS_U_SENS: return (int32_t)(ADC2_buff[adc2_meas] * U_SENS_COEFF_MV);
        	default:            return 0;
    	}
}

FuncStatus Check_OCP_status(void)
{
    	// Sprawdzenie flagi ze sprzętowego przerzutnika OCP
    	if(HAL_GPIO_ReadPin(GPIOB, DC_BUS_OC_n_Pin) == GPIO_PIN_RESET)
    	{
        	// Próba zresetowania usterki
        	HAL_GPIO_WritePin(GPIOC, DC_BUS_OC_RST_Pin, GPIO_PIN_SET);
        	HAL_Delay(1);
        	HAL_GPIO_WritePin(GPIOC, DC_BUS_OC_RST_Pin, GPIO_PIN_RESET);
        	HAL_Delay(1);

        	if(HAL_GPIO_ReadPin(GPIOB, DC_BUS_OC_n_Pin) == GPIO_PIN_RESET)
        	{
            		Active_Faults |= FAULT_OVER_CURR; // Ustawienie flagi w rejestrze systemowym
            		return EXEC_ERROR;                // Przerwanie wykonania z błędem
        	}
    	}
    	return EXEC_SUCCESS;
}

FuncStatus Check_OVP_status(void)
{
    	// 1. Sprawdzenie flagi ze sprzętowego przerzutnika OVP
    	if(HAL_GPIO_ReadPin(GPIOB, DC_BUS_OV_n_Pin) == GPIO_PIN_RESET)
    	{
        	// Próba zresetowania usterki
        	HAL_GPIO_WritePin(GPIOC, DC_BUS_OV_RST_Pin, GPIO_PIN_SET);
        	HAL_Delay(1);
        	HAL_GPIO_WritePin(GPIOC, DC_BUS_OV_RST_Pin, GPIO_PIN_RESET);
        	HAL_Delay(1);

        	if(HAL_GPIO_ReadPin(GPIOB, DC_BUS_OV_n_Pin) == GPIO_PIN_RESET)
        	{
            		Active_Faults |= FAULT_OVER_VOLT;
            		return EXEC_ERROR;
        	}
    	}

    	// 2. Nadmiarowe, statyczne sprawdzenie programowe odczytu ADC
    	if(adc2_2val(DC_BUS_U_SENS) > DC_BUS_OVP_THRESHOLD_MV)
    	{
        	Active_Faults |= FAULT_OVER_VOLT;
        	return EXEC_ERROR;
    	}

    	return EXEC_SUCCESS;
}

FuncStatus DC_BUS_charge_monitor(void)
{
    	uint32_t start_time = HAL_GetTick();

    	while((HAL_GetTick() - start_time) < 3000)
    	{
        	uint16_t batt_v = adc2_2val(BATT_U_SENS);
        	uint16_t dc_bus_v = adc2_2val(DC_BUS_U_SENS);

        	if ((batt_v > 0) && (dc_bus_v >= (batt_v * 95 / 100)))
            		return EXEC_SUCCESS;
        	HAL_Delay(1);
    	}

    	Active_Faults |= FAULT_PRECHARGE_TIMEOUT;
    	return EXEC_ERROR;
}

FuncStatus Inverter_startup_procedure(void)
{
		// 0. JAWNA BLOKADA BRAMEK I PODTRZYMANIE PĘTLI SDC
		HAL_GPIO_WritePin(GPIOA, HB_EN_n_Pin, GPIO_PIN_SET);

		//Przy NUCLEO niepotrzebne:
		//HAL_GPIO_WritePin(GPIOA, MCU_WATCHDOG_n_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(GPIOA, MCU_SDC_ON_n_Pin, GPIO_PIN_RESET);

    	// 1. Sprawdzenie i ewentualny reset sprzętowych zabezpieczeń zwarciowych i przepięciowych
    	if (Check_OCP_status() != EXEC_SUCCESS) return EXEC_ERROR;
    	if (Check_OVP_status() != EXEC_SUCCESS) return EXEC_ERROR;

    	// 2. Asynchroniczne monitorowanie przyrostu napięcia szyny DC do 95% napięcia baterii
    	//if (DC_BUS_charge_monitor() != EXEC_SUCCESS) return EXEC_ERROR;
    	HAL_Delay(5000);

    	// 3. Wyłączenie Pre-charge'a i włączenie głównego przekaźnika:
    	HAL_GPIO_WritePin(GPIOC, DC_BUS_READY_Pin, GPIO_PIN_SET);
    	HAL_Delay(1000); // Czas propagacji izolatora VOM1271 i MOSFETów to ~65us.

    	// Sprzętowe potwierdzenie stanu przekaźnikow
    	if (HAL_GPIO_ReadPin(GPIOB, SDC_RELAY_ON_Pin) == GPIO_PIN_RESET)
    	{
        	Active_Faults |= FAULT_SDC_FEEDBACK;
        	return EXEC_ERROR;
    	}

    	// 4. Ostateczne odblokowanie bramek GaN na stopniu mocy (EPC23101)
    	HAL_GPIO_WritePin(GPIOA, HB_EN_n_Pin, GPIO_PIN_RESET);

    	return EXEC_SUCCESS;
}

void Enable_PWM_signals(void)
{
	// 1. Aktywacja podstawowych kanałów PWM (dla górnych tranzystorów High-Side)
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // 2. Aktywacja komplementarnych kanałów PWM (dla dolnych tranzystorów Low-Side)
    // Funkcje z przyrostkiem "Ex" i "PWMN" pochodzą z rozszerzonej biblioteki HAL
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    // 3. Odblokowanie przerwania Update (od przepełnienia timera)
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

// Funkcja pomocnicza zlecająca sprzętowe policzenie sinusa w CORDIC
static inline float CORDIC_sinf(uint32_t angle)
{
    // Wpisanie kąta natychmiast uruchamia obliczenia w koprocesorze
    LL_CORDIC_WriteData(CORDIC, (int32_t)angle);

    // Odczyt wyniku i przeskalowanie
    return (float)((int32_t)LL_CORDIC_ReadData(CORDIC)) / 2147483648.0f;
}


//------------------------------------------------------------------------ FUNKCJE PRZERWAŃ

void TIM1_UP_TIM16_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

        // 1. Zlecenie obliczeń CORDIC z odpowiednimi przesunięciami fazowymi
        float sin_u = CORDIC_sinf(phase_acc);
        float sin_v = CORDIC_sinf(phase_acc + 0x55555555); // Faza V (+120 stopni)
        float sin_w = CORDIC_sinf(phase_acc + 0xAAAAAAAA); // Faza W (+240 stopni)

        // 2. Modulacja i wpisanie sprzętowe do rejestrów CCR
        // Ponieważ rdzeń posiada sprzętowe FPU, te operacje mnożenia trwają ułamki mikrosekund
        TIM1->CCR1 = (uint16_t)(tim1_arr_value * (0.5f + (0.5f * mod_index * sin_u)));
        TIM1->CCR2 = (uint16_t)(tim1_arr_value * (0.5f + (0.5f * mod_index * sin_v)));
        TIM1->CCR3 = (uint16_t)(tim1_arr_value * (0.5f + (0.5f * mod_index * sin_w)));

        // 3. Inkrementacja kąta (częstotliwość f = (phase_step * f_pwm) / 0xFFFFFFFF)
        phase_acc += phase_step;
    }
}

void ADC1_2_IRQHandler(void)
{
    	// Sprawdzenie flagi End Of Conversion (EOC)
		if (LL_ADC_IsActiveFlag_EOC(ADC1) != 0)
	    {
	       	// Dla wersji z MC SDK: Funkcja 12L (czyta całe słowo dla Left-Aligned)
	       	// i przesunięcie >> 4 (aby "naprawić" wyrównanie do lewej i otrzymać 0-4095)
        	// ADC2_buff[current_measurement] = (LL_ADC_REG_ReadConversionData12L(ADC2) >> 4);

	   		// Dla wersji z MC SDK Funkcja 12 (czyta całe słowo dla Right-Aligned):
	   		ADC1_buff[current_adc1_measurement] = LL_ADC_REG_ReadConversionData12(ADC1);

    		current_adc1_measurement++;
        	if(current_adc1_measurement == ADC1_ACTIVE_CH_NUM)
        	{
           		current_adc1_measurement = 0;
           		// Natychmiastowe wznowienie sekwencji (Software Continuous Mode)
           		LL_ADC_REG_StartConversion(ADC1);
        	}

        	// Obowiązkowe sprzętowe wyczyszczenie flagi przed wyjściem z przerwania
        	LL_ADC_ClearFlag_EOC(ADC1);
	    }

    	if (LL_ADC_IsActiveFlag_EOC(ADC2) != 0)
    	{
    		ADC2_buff[current_adc2_measurement] = LL_ADC_REG_ReadConversionData12(ADC2);

    		current_adc2_measurement++;
        	if(current_adc2_measurement == ADC2_ACTIVE_CH_NUM)
        	{
            	current_adc2_measurement = 0;
            	LL_ADC_REG_StartConversion(ADC2);
        	}

        	LL_ADC_ClearFlag_EOC(ADC2);
    	}
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
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  	  // ============================================================================================= SOFT START TIMER
  	  if (htim->Instance == TIM7)
  	  {
  		  if(mod_index < soft_start_end_index-0.01) // -0.01 bo wartość soft_start_end_index z jakiegos powodu wynosi mikroskopijnie mniej niz to zadano na poczatku
  		  {
  			  mod_index += 0.01f;
  			  HAL_GPIO_TogglePin(GPIOB, STATUS_LED_Pin);
  		  }
  		  else
  		  {
  			  HAL_TIM_Base_Stop_IT(&htim7);
  			  HAL_GPIO_WritePin(GPIOB, STATUS_LED_Pin, GPIO_PIN_SET);
  			  // tu można ustawić bit poprawnie wykonanej procedury soft start do bitów statusowych
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
#ifdef USE_FULL_ASSERT
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
