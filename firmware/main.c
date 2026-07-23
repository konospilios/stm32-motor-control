/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file             : main.c
* @brief            : Main program body
******************************************************************************
* @attention
*
* Copyright (c) 2024 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
******************************************************************************
*/
/* USER CODE END Header */
/* Includes------------------------------------------------------------------*/
#include "main.h"

/* Private includes----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef-----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265

// Data sampling
#define ARRAY_SIZE 2750 // This defines the amount of data points saved
int Data_rate = 10; // Defines the number of 10kHz iterations for each data sample. So if it is 10 then we save at 1kHz
int Data_indx; // Counter using for the data_rate
int count = 0; // Used as index for the arrays the data is saved in.
int sample_data = 0; // Should be set to 1 to start data acquisition.
int w = 0; // Defines the number of 10kHz iterations before the data starts sampling

int Global_enable = 0; // Variable that is changed to globally enable or disable the inverter and control loop.

float omega_ref = 0; //[Rad/s]
float i_qref = 0;
int pos_loop = 0; // 1 if position controller is active.
int Speed_loop = 0; // 1 if speed controller is active.
float phi_ref = 0; // Reference position.
int send_data = 0;

// Controller Variables
float T_e = 0.0001;
float T_m = 0.001;

// For controllers using Kp and Ki with antiwindup
float K_p_m = 0.0792;//1.5842;//0.045;
float K_i_m = 0.0200;//0.035;

float K_p_e =   7;
float K_i_e = 700;


// Define arrays to save the data in
float array1[ARRAY_SIZE]; //
float array2[ARRAY_SIZE]; //
float array3[ARRAY_SIZE];
float array4[ARRAY_SIZE];
float array5[ARRAY_SIZE];
float array6[ARRAY_SIZE];
float array7[ARRAY_SIZE];
float array8[ARRAY_SIZE];
float array9[ARRAY_SIZE];
float array10[ARRAY_SIZE];
float array11[ARRAY_SIZE];
//uint16_t array12[ARRAY_SIZE];
//uint16_t array13[ARRAY_SIZE];
//uint16_t array14[ARRAY_SIZE];

int j;//Counter used to see that the while loop is executed
int k; //

int location = 0;


float V_alpha;
float V_beta;
float t = 0;
float omega_dot = 62.83;
int constant_acc = 0;


// Variables for SVPWM
#define V_s 16 // Supply voltage [V]
float V_s_factor = 1.0/(2.0*16.0/3.0); // TH
double T_1;
double T_2;
double T_0;
uint32_t T1_count;
uint32_t T2_count;
uint32_t T0_count;
int N; // Used to determine the sector
int Sector; // Variable for which sector we are in. Only used for validation.


// Angle measurements
int angle;
float angle_abs = 0;
float abs_add = 0;
uint16_t angle_HS;
uint16_t angle_HS_shift;
float angleRad;
uint16_t angle_encoder;

// Function definitions for functions defined later in the code
float getSine(unsigned int angle);
float getCoSine(unsigned int angle);

float FilterSpeed(float speed, float LP_old);
void readAngleFromMA730(uint16_t *angle_HS);

//Terkel
// Current variables.
float i1 = 0; // This is the current converted from ADC measurement
float i2 = 0;
float i3 = 0;

// Currents converted to dq-system
float i_d = 0;
float i_q = 0;

float V_dref = 0;
float V_dref_sat = 0;
float V_qref = 0;
float V_qref_sat = 0;
float C_omega_i_old = 0;
float C_omega_i = 0;
float i_qref_sat = 0;

float C_dCurr_old = 0;
float C_qCurr_old = 0;
float C_dCurr_i = 0;
float C_qCurr_i = 0;


// Position controller variables


float e_omega;

float e_dCurr = 0;
float e_qCurr = 0;


// speed calculation variables
float phi_old = 0;
int indx = 0;
float speed_old = 0;
float speed_LP;
float LP_old = 0;
float alpha = 0.9101; // Discretization







// Speed_rad, Position_rad_encoder, Position_rad_hall, i1, i2, i3,speed_reference, position_reference.




// Generate empty array for sine lookup and the step size.
#define NUM_POINTS 536 // Number of data points in lookup array.
float SinTable[NUM_POINTS];
float CosTable[NUM_POINTS];
const double step = 2.0 * M_PI /( 4*NUM_POINTS); // Angle step size

// Variables used to store the sine and cosine lookup value for the given iteration.
float sin_lookup;
float cos_lookup;


/* USER CODE END PD */

/* Private macro-------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes-----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t tim1_cnt = 0;
uint16_t tim11_cnt = 0;
uint32_t tim2_cnt = 0;
uint16_t enc_cnt = 0;


float phi = 0;
float phi_nomod = 0;
float phi_el = 0;
float speed = 0;
float speed_rad = 0;
int rpm = 0;
int ext = 0;
float filter_coeff = 0.1;
float speed_LP = 0;
//float speed_kal = 0;

//int EMFA = 0;
//int EMFB = 0;
//int EMFC = 0;

uint16_t adc[3];

float i1_raw;
float i2_raw;
float i3_raw;

#define VSENSE 3.3/4096

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
        TIM3->CNT = 0;
        ext = ext + 1;
    }

/* USER CODE END 0 */

/**
* @brief The application entry point.
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
     MX_TIM11_Init();
     MX_TIM1_Init();
     MX_ADC1_Init();
     MX_TIM3_Init();
     MX_TIM2_Init();
     MX_USART2_UART_Init();
     MX_SPI3_Init();
     /* USER CODE BEGIN 2 */

     // Create sin angle Table.
     for (int i = 0; i < NUM_POINTS; ++i) {
         double angle = i * step;
         SinTable[i] = sin(angle);
     }

     // Create cosine angle Table.
     for (int i = 0; i < NUM_POINTS; ++i) {
         double angle = i * step;
         CosTable[i] = cos(angle);
     }

     // Global enable from the begining

     // enable the transistors from the begining.
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1);

 // initialize counter compare value to 0 and start the timer PWM for timer 1 channel 1.
TIM1->CCR1 = 0;
TIM1->CCR2 = 0;
TIM1->CCR3 = 0;
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4); //slet

HAL_TIM_Base_Start(&htim2);
HAL_TIM_Base_Start_IT(&htim11);

//Global_enable = 1;

//Terkel
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc, 3);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
#define VREFINT 1.21
#define ADCMAX 4095.0

/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      if(Global_enable == 0 && sample_data == 0){
          count = 0;
          w = 0;
          Data_indx = 0;
          //indx = 0;
          //k = 0;
          t = 0;


      }
      j = j +1;
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
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
RCC_OscInitStruct.HSIState = RCC_HSI_ON;
RCC_OscInitStruct.HSICalibrationValue =RCC_HSICALIBRATION_DEFAULT;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
RCC_OscInitStruct.PLL.PLLM = 8;
RCC_OscInitStruct.PLL.PLLN = 96;
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
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) !=HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution,Data Alignment and number of conversion)
*/
hadc1.Instance = ADC1;
hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
hadc1.Init.Resolution = ADC_RESOLUTION_12B;
hadc1.Init.ScanConvMode = ENABLE;
hadc1.Init.ContinuousConvMode = DISABLE;
hadc1.Init.DiscontinuousConvMode = ENABLE;
hadc1.Init.NbrOfDiscConversion = 3;
hadc1.Init.ExternalTrigConvEdge =ADC_EXTERNALTRIGCONVEDGE_FALLING;
hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
hadc1.Init.NbrOfConversion = 3;
hadc1.Init.DMAContinuousRequests = ENABLE;
hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
if (HAL_ADC_Init(&hadc1) != HAL_OK)
{
    Error_Handler();
}
/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
*/
sConfig.Channel = ADC_CHANNEL_0;
sConfig.Rank = 1;
sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
    Error_Handler();
}
/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
*/
sConfig.Channel = ADC_CHANNEL_11;
sConfig.Rank = 2;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
    Error_Handler();
}
/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
*/
sConfig.Channel = ADC_CHANNEL_10;
sConfig.Rank = 3;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
    Error_Handler();
}
/* USER CODE BEGIN ADC1_Init 2 */

/* USER CODE END ADC1_Init 2 */

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
hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
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
* @brief TIM1 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM1_Init(void)
{

/* USER CODE BEGIN TIM1_Init 0 */

/* USER CODE END TIM1_Init 0 */

TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_OC_InitTypeDef sConfigOC = {0};
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

/* USER CODE BEGIN TIM1_Init 1 */

/* USER CODE END TIM1_Init 1 */
htim1.Instance = TIM1;
htim1.Init.Prescaler = 96-1;
htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
htim1.Init.Period = 49;
htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim1.Init.RepetitionCounter = 0;
htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
{
    Error_Handler();
}
if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
{
    Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)!= HAL_OK)
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
sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
sConfigOC.Pulse = 1200;
if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) !=HAL_OK)
{
    Error_Handler();
}
sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
sBreakDeadTimeConfig.DeadTime = 0;
sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
sBreakDeadTimeConfig.AutomaticOutput =TIM_AUTOMATICOUTPUT_DISABLE;
if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
{
    Error_Handler();
}
/* USER CODE BEGIN TIM1_Init 2 */

/* USER CODE END TIM1_Init 2 */
HAL_TIM_MspPostInit(&htim1);

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

     TIM_SlaveConfigTypeDef sSlaveConfig = {0};
     TIM_MasterConfigTypeDef sMasterConfig = {0};

     /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 192-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 48;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)!= HAL_OK)
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

TIM_Encoder_InitTypeDef sConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};

     /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 15000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)!= HAL_OK)
{
    Error_Handler();
}
/* USER CODE BEGIN TIM3_Init 2 */

HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

/* USER CODE END TIM3_Init 2 */

}

/**
* @brief TIM11 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM11_Init(void)
{

/* USER CODE BEGIN TIM11_Init 0 */

/* USER CODE END TIM11_Init 0 */

/* USER CODE BEGIN TIM11_Init 1 */
669      /* USER CODE BEGIN TIM3_Init 1 */670 671   /* USER CODE END TIM3_Init 1 */672   htim3.Instance = TIM3;673   htim3.Init.Prescaler = 0;674   htim3.Init.CounterMode = TIM_COUNTERMODE_UP;675   htim3.Init.Period = 15000;676   htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;677   htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;678   sConfig.EncoderMode = TIM_ENCODERMODE_TI1;679   sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;680   sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;681   sConfig.IC1Prescaler = TIM_ICPSC_DIV1;682   sConfig.IC1Filter = 0;683   sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;684   sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;685   sConfig.IC2Prescaler = TIM_ICPSC_DIV1;686   sConfig.IC2Filter = 0;687   if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)688   {689     Error_Handler();690   }691   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;692   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;693   if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)!= HAL_OK)694 {695     Error_Handler();696 }697 /* USER CODE BEGIN TIM3_Init 2 */698 699 HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);700 701 /* USER CODE END TIM3_Init 2 */702 703 }704 705 /**706 * @brief TIM11 Initialization Function 707 * @param None 708 * @retval None 709 */710 static void MX_TIM11_Init(void)711 {712 713 /* USER CODE BEGIN TIM11_Init 0 */714 715 /* USER CODE END TIM11_Init 0 */716 717 /* USER CODE BEGIN TIM11_Init 1 */718
/* USER CODE END TIM11_Init 1 */
htim11.Instance = TIM11;
htim11.Init.Prescaler = 192-1;
htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
htim11.Init.Period = 48;
htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
{
    Error_Handler();
}
/* USER CODE BEGIN TIM11_Init 2 */

/* USER CODE END TIM11_Init 2 */

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
* Enable DMA controller clock
*/
static void MX_DMA_Init(void)
{

/* DMA controller clock enable */
__HAL_RCC_DMA2_CLK_ENABLE();

/* DMA interrupt init */
/* DMA2_Stream0_IRQn interrupt configuration */
HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();
__HAL_RCC_GPIOD_CLK_ENABLE();

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOA, LED_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

/*Configure GPIO pins : PC3 PC9 PC10 PC11
                           PC12 */
GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/*Configure GPIO pins : LED_Pin PA6 */
GPIO_InitStruct.Pin = LED_Pin|GPIO_PIN_6;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/*Configure GPIO pin : PB1 */
GPIO_InitStruct.Pin = GPIO_PIN_1;
GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/*Configure GPIO pin : PD2 */
GPIO_InitStruct.Pin = GPIO_PIN_2;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* EXTI interrupt init*/
HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
// Callback: timer has rolled over

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
// Check which version of the timer triggered. Calculate the new times for for the different gates.
if (htim == &htim11)
      {







//        readAngleFromMA730(&angle_HS);

//        angleRad = (angle_HS * 0.000095873799);
//        angle_HS_shift = angle_HS >> 2;
//        uint16_t enc_cnt_old = enc_cnt;
          enc_cnt = __HAL_TIM_GET_COUNTER(&htim3);


          angle = (enc_cnt + (14238)) % 2143; // 15000 - 762 = 14238
            angle_encoder = ((enc_cnt + (14238)) % 15000);
            phi_nomod = (float)((enc_cnt + (14238)) % 15000) *0.000418879;

          indx++;
        int indx_max = 10;
        float phi_diff = phi_nomod - phi_old;
        if (indx == indx_max){
            if(phi_diff > 3.1415){
                 speed_rad = (phi_diff - 6.283185)/(indx_max*0.0001);
            }
            else if(phi_diff < -3.1415){
                 speed_rad = (phi_diff + 6.283185)/(indx_max*0.0001);
            }
            else {
                 speed_rad = phi_diff/(indx_max*0.0001);
            }
            phi_old = phi_nomod;
            indx = 0;

            LP_old = speed_LP;
            speed_LP = FilterSpeed(speed_rad, LP_old);
            if(phi_diff > 3.1415){
                 abs_add = abs_add - 6.28318;
            }
            else if(phi_diff < -3.1415){
                 abs_add = abs_add + 6.28318;
            }
                 angle_abs = phi_nomod + abs_add;


        }
        if (Global_enable == 1){
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
              t = t + 0.0001;

             // Lookup table local variable - Fixed point
            sin_lookup = getSine(angle);
            cos_lookup = getCoSine(angle);

            // Current
            i1 = 0.001595690196*adc[0] - 3.08973;
            i2 = 0.001595690196*adc[1] - 3.08973;
            i3 = 0.001595690196*adc[2] - 3.08973;


//      Park Transform (abc to dq0) - alpha-d alligned
            i_d = (0.667)*(i1*cos_lookup + i2*(0.866*sin_lookup -0.5*cos_lookup) + i3*(-0.5*cos_lookup - 0.866*sin_lookup)); // I think this can be speeded up from multiplying into the parenteces ourself and simplifying.
            i_q = (0.667)*(- i1*sin_lookup - i2*(-0.866*cos_lookup -0.5*sin_lookup) - i3*(0.866*cos_lookup - 0.5*sin_lookup));
            // i_d = 0.667*cos_lookup*i1 - 0.3335*cos_lookup*i2 -0.3335*cos_lookup*i3 + 0.57762*i2*sin_lookup - 0.57762*i3*sin_lookup
            // i_d = cos_lookup * (0.667*i1 - 0.3335*i2 - 0.3335*i3)+ sin_lookup * (0.57762*i2 - 0.57762*i3); // potential improvement.


            // i_0 = (0.667)*0.5*(i1 + i2 + i3); // Commented out since it is not used



//              Controller - speed indx == 0 correspond to indx = 10 before. This is Validated
            if (Speed_loop == 1 && indx == 0){
                // The position controller is inside the speed controller to sync them.
                if (pos_loop == 1){
                    if (k == 10){
                        float K_pos = 20;
                        omega_ref = K_pos*(phi_ref - angle_abs);
                        k = 0;
                    }
                        else{
                            k++;
                    }
                }


                if(constant_acc == 1){
                    omega_ref = t*omega_dot;

                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
                }
                if( constant_acc == 1 && t > 2){
                    omega_ref = -5;
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
                                 }
                float K_AW_m = 0.001;
                e_omega = omega_ref - speed_rad;

                C_omega_i = K_i_m * T_m *e_omega + C_omega_i_old +K_AW_m*(i_qref_sat - i_qref);
                i_qref = K_p_m * e_omega + C_omega_i;
                C_omega_i_old = C_omega_i;

                i_qref_sat = i_qref;
                if (i_qref > 2.85){
                    i_qref_sat = 2.85;
                }
                else if(i_qref < -2.85){
                    i_qref_sat = -2.85;
                }
                else{
                    i_qref_sat = i_qref;
                }
            }



       //   Controllers - Current (i_dref = 0)

            e_dCurr = (0 - i_d);
            e_qCurr = (i_qref_sat - i_q);

            //PI controller


            float K_AW = 0.1;
            C_dCurr_i = K_i_e * T_e *e_dCurr + C_dCurr_old + K_AW*(V_dref_sat - V_dref);
            V_dref = K_p_e * e_dCurr + C_dCurr_i;// -speed_LP*7*0.0024*i_q;
            C_dCurr_old = C_dCurr_i;

            C_qCurr_i = K_i_e * T_e *e_qCurr + C_qCurr_old + K_AW*(V_qref_sat - V_qref);
            V_qref = K_p_e * e_qCurr +C_qCurr_i;// + speed_LP*7*(0.0024*i_q + 0.0102);
            C_qCurr_old = C_qCurr_i;
            // P-controller
//          V_dref = e_dCurr *28;
//          V_qref = e_qCurr *28;

            // Saturation of the vector magnitude
            float mag_V = sqrtf(V_dref*V_dref + V_qref*V_qref);
            if (mag_V > 9.2376){
                V_dref_sat = V_dref/mag_V*9.2376;
                V_qref_sat = V_qref/mag_V*9.2376;
            }
            else{
                V_dref_sat = V_dref;
                V_qref_sat = V_qref;
            }

      //    Inverse Park - d-alpha alligned
            V_alpha = cos_lookup*V_dref_sat - sin_lookup*V_qref_sat;
            V_beta = sin_lookup*V_dref_sat + cos_lookup*V_qref_sat;


            if(sample_data == 1){
               w = w+1;
           }

           // Data sampling

            if(sample_data == 1 && w > 0){
            // This functions saves the variables to an array each iteration. Remember to have the array be the right type.
                Data_indx = Data_indx + 1;
                if(Data_indx == Data_rate && count < ARRAY_SIZE ){
                    array1[count] = (float) phi_nomod;
                    array2[count] = (float) angle_abs;
                    array3[count] = (float) phi_ref;
                    array4[count] = (float) speed;
                    array5[count] = (float) speed_LP;
                    array6[count] = (float) omega_ref;
                    array7[count] = (float) i_qref_sat;
                    array8[count] = (float) i_q;
                    array9[count] = (float) i_qref;
                    array10[count] = (float) V_qref;
                    array11[count] = (float) V_qref_sat;
//                  array12[count] = (uint16_t) angle_encoder;
//                  array13[count] = (uint16_t) angle_HS;
//                  array14[count] = (uint16_t) angle_HS_shift;
                    Data_indx = 0;
                    count = count + 1; }
                }


               if(count == ARRAY_SIZE && send_data == 1){
                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0); //Disable global enable when the data is saved. This is to stop current flowing in the motor.

                   HAL_UART_Transmit(&huart2, (uint8_t *)array1,sizeof(float)*ARRAY_SIZE, HAL_MAX_DELAY); // This sends the array over uart. Remember to specify correct datatype in sizeof(). >The other should always be uint8_t *
                   HAL_UART_Transmit(&huart2, (uint8_t *)array2,sizeof(float)*ARRAY_SIZE, HAL_MAX_DELAY);
                   HAL_UART_Transmit(&huart2, (uint8_t *)array3,sizeof(float)*ARRAY_SIZE, HAL_MAX_DELAY);
                   HAL_UART_Transmit(&huart2, (uint8_t *)array4,sizeof(float)*ARRAY_SIZE, HAL_MAX_DELAY);
                   HAL_UART_Transmit(&huart2, (uint8_t *)array5,sizeof(float)*ARRAY_SIZE, HAL_MAX_DELAY);
                   HAL_UART_Transmit(&huart2, (uint8_t *)array6,sizeof(float)*ARRAY_SIZE, HAL_MAX_DELAY);
                   HAL_UART_Transmit(&huart2, (uint8_t *)array7,sizeof(float)*ARRAY_SIZE, HAL_MAX_DELAY);
                   HAL_UART_Transmit(&huart2, (uint8_t *)array8,sizeof(float)*ARRAY_SIZE, HAL_MAX_DELAY);
                   HAL_UART_Transmit(&huart2, (uint8_t *)array9,sizeof(float)*ARRAY_SIZE, HAL_MAX_DELAY);
                   HAL_UART_Transmit(&huart2, (uint8_t *)array10,sizeof(float)*ARRAY_SIZE, HAL_MAX_DELAY);
                   HAL_UART_Transmit(&huart2, (uint8_t *)array11,sizeof(float)*ARRAY_SIZE, HAL_MAX_DELAY);

//                 HAL_UART_Transmit(&huart2, (uint8_t *)array12,sizeof(uint16_t)*ARRAY_SIZE, HAL_MAX_DELAY);
//                 HAL_UART_Transmit(&huart2, (uint8_t *)array13,sizeof(uint16_t)*ARRAY_SIZE, HAL_MAX_DELAY);
//                 HAL_UART_Transmit(&huart2, (uint8_t *)array14,sizeof(uint16_t)*ARRAY_SIZE, HAL_MAX_DELAY);

                   // transmit controller values
                   float controller[4];
                   controller[0] = K_p_m;
                   controller[1] = K_i_m;
                   controller[2] = K_p_e;
                   controller[3] = K_i_e;

                   HAL_UART_Transmit(&huart2, (uint8_t *)controller,sizeof(float)*4, HAL_MAX_DELAY);


                   count = 0;
                   w = 0;
                   }


                float V_ref2 = 0.866025*V_alpha - 0.5*V_beta;
                float V_ref3 = -0.866025*V_alpha - 0.5*V_beta;
                N = copysign(1,V_beta) + 2*copysign(1,V_ref2) +4*copysign(1,V_ref3);
                float C1 = 0.57735027;
                float C2 = 1.15470054;
                int CP = 48;
                switch(N){
                case -1:
                    Sector = 1;
                    T_1 = ( V_alpha - C1*V_beta )*V_s_factor;
                    T_2 = (            C2*V_beta )*V_s_factor;
                    T_0 = 1 - (T_1 + T_2);
                    T1_count = T_1*CP ;//- 1;
                    T2_count = T_2*CP ;//- 1;
                    T0_count = T_0*CP ;//- 1;
                    TIM1->CCR1 = 0.5*T0_count;
                    TIM1->CCR2 = 0.5*T0_count + T1_count;
                    TIM1->CCR3 = 0.5*T0_count + T1_count + T2_count;
                    break;
    case -5:
        Sector = 2;
        T_1 = ( V_alpha + C1*V_beta )*V_s_factor;
        T_2 = (-V_alpha + C1*V_beta )*V_s_factor;
        T_0 = 1 - (T_1 + T_2);
        T1_count = T_1*CP ;//- 1;
        T2_count = T_2*CP ;//- 1;
        T0_count = T_0*CP ;//- 1;
        TIM1->CCR1 = 0.5*T0_count + T2_count;
        TIM1->CCR2 = 0.5*T0_count;
        TIM1->CCR3 = 0.5*T0_count + T2_count + T1_count;
        break;

    case 3:
        Sector = 3;
        T_1 = (           C2*V_beta )*V_s_factor;
        T_2 = (-V_alpha - C1*V_beta )*V_s_factor;
        T_0 = 1 - (T_1 + T_2);
        T1_count = T_1*CP ;//- 1;
        T2_count = T_2*CP ;//- 1;
        T0_count = T_0*CP ;//- 1;
        TIM1->CCR1 = 0.5*T0_count + T1_count + T2_count;
        TIM1->CCR2 = 0.5*T0_count;
        TIM1->CCR3 = 0.5*T0_count + T1_count;
        break;
    case 1:
        Sector = 4;
        T_1 = (-V_alpha + C1*V_beta )*V_s_factor;
        T_2 = (         - C2*V_beta )*V_s_factor;
        T_0 = 1 - (T_1 + T_2);
        T1_count = T_1*CP ;//- 1;
        T2_count = T_2*CP ;//- 1;
        T0_count = T_0*CP ;//- 1;
        TIM1->CCR1 = 0.5*T0_count + T2_count + T1_count;
        TIM1->CCR2 = 0.5*T0_count + T2_count;
        TIM1->CCR3 = 0.5*T0_count;
        break;
    case 5:
        Sector = 5;
        T_1 = (-V_alpha - C1*V_beta )*V_s_factor;
        T_2 = ( V_alpha - C1*V_beta )*V_s_factor;
        T_0 = 1 - (T_1 + T_2);
        T1_count = T_1*CP ;//- 1;
        T2_count = T_2*CP ;//- 1;
        T0_count = T_0*CP ;//- 1;
        TIM1->CCR1 = 0.5*T0_count + T1_count;
        TIM1->CCR2 = 0.5*T0_count + T1_count + T2_count;
        TIM1->CCR3 = 0.5*T0_count;
        break;
    case -3:
        Sector = 6;
                    T_1 = (          - C2*V_beta )*V_s_factor;
                    T_2 = ( V_alpha + C1*V_beta )*V_s_factor;
                    T_0 = 1 - (T_1 + T_2);
                    T1_count = T_1*CP ;//- 1;
                    T2_count = T_2*CP ;//- 1;
                    T0_count = T_0*CP ;//- 1;
                    TIM1->CCR1 = 0.5*T0_count ;
                    TIM1->CCR2 = 0.5*T0_count + T2_count + T1_count;
                    TIM1->CCR3 = 0.5*T0_count + T2_count;
                    break;
                }
        }
        else{
                           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
        }
//          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);



              }

}
float getSine(unsigned int angle) {
    angle = angle % 2143; //angle &= 15000; // Reduce angle to the range of 1 circle
    if (angle < 536){
        return SinTable[angle];
    } else if(angle >= 536 && angle < 1072 ) {
        return SinTable[1072 - (angle + 1)];
    } else if(angle >= 1072 && angle < 1607){
        return -SinTable[angle-1072];
    } else if(angle >= 1607 && angle < 2143){
        return -SinTable[2143-(angle + 1)];
    }
}
float getCoSine(unsigned int angle) {
    angle = angle % 2143; //angle &= 15000; // Reduce angle to the range of 1 circle
    if (angle < 536){
        return CosTable[angle];
    } else if(angle >= 536 && angle < 1072 ) {
        return -CosTable[1072 - (angle + 1)];
    } else if(angle >= 1072 && angle < 1607){
        return -CosTable[angle-1072];
    } else if(angle >= 1607 && angle < 2143){
        return CosTable[2143-(angle + 1)];
    }
}


float FilterSpeed(float speed, float LP_old) {
     speed_LP = alpha*LP_old + (1-alpha)*speed;
}

void readAngleFromMA730(uint16_t *angle_HS) { // Hall sensor
//     uint8_t rxBuffer[2] = {0}; // Buffer to store the received data
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // CS Low
     HAL_SPI_Receive(&hspi3, angle_HS, 2, 100); // Receive 2 bytes(16 bits) of data
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // CS High


}



//Terkel
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
       HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
//     HAL_Delay(1);
//     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
}

/* USER CODE END 4 */

/**
* @brief This function is executed in case of error occurrence.
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
* @brief Reports the name of the source file and the source line number
*            where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{
/* USER CODE BEGIN 6 */
/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n",file, line) */
/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


