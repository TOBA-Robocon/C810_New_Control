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

#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "stdbool.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

ADC_ChannelConfTypeDef ADC1_Config = {0};
ADC_ChannelConfTypeDef ADC2_Config = {0};

typedef struct PI_value{
	float kp;
	float ki;
	float goal;
	float now_value;
	float error;
	float error_integral;
	float integral_limit;
}PI_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//PWMの使用タイマ
#define MOTOR_PWM_TIMER htim3
//各チャンネル
#define U_PHASE_PWM_CHANNEL TIM_CHANNEL_3
#define V_PHASE_PWM_CHANNEL TIM_CHANNEL_2
#define W_PHASE_PWM_CHANNEL TIM_CHANNEL_1

//PWM出力関数
#define SET_PWM_DUTY(PHASE, DUTY) __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIMER, PHASE##_PHASE_PWM_CHANNEL, DUTY)

// PI 定数
#define PI 3.14159265f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//制御周期生成用フラグ
volatile bool tim2_intrupt_flag = false;
volatile bool tim1_intrupt_flag = false;

/*	強制転流用変数		*/
//#define sin_num 1000
//
//float sin_table[sin_num] = {0};
//
//float one_step = 0.0;
//int loop_count = 0;
//#define max_duty 530
//#define min_duty 470

/*	センサ用変数	*/
float ADC_sin = 0.0; //max:1.597216 min:0.382784
float ADC_cos = 0.0; //max:1.619780 min:0.355385

float ADC_sin_max = 1.597216;
float ADC_sin_min = 0.382784;
float ADC_cos_max = 1.619780;
float ADC_cos_min = 0.355385;

//float ADC_sin_max = 0.0;
//float ADC_sin_min = 3.3;
//float ADC_cos_max = 0.0;
//float ADC_cos_min = 3.3;

//現在の電流センサからの電圧値
uint16_t U_voltage = 0;
uint16_t V_voltage = 0;
uint16_t W_voltage = 0;

//初期化時の電流センサからの電圧値
uint32_t U_voltage_init = 0;
uint32_t V_voltage_init = 0;
uint32_t W_voltage_init = 0;

//現在の電流値
float U_Current_now = 0.0;
float V_Current_now = 0.0;
float W_Current_now = 0.0;

/*	移動平均用の変数	*/
#define sample_num 2	//移動平均のサンプル数

//サンプル値の格納配列
float U_Current_samples[sample_num] = {0};
float V_Current_samples[sample_num] = {0};
float W_Current_samples[sample_num] = {0};

//サンプル値の総和
float U_Current_sum = 0.0;
float V_Current_sum = 0.0;
float W_Current_sum = 0.0;

//サンプル値の平均
float U_Current_ave = 0.0;
float V_Current_ave = 0.0;
float W_Current_ave = 0.0;

//リアル値
float U_Current_real = 0.0;
float V_Current_real = 0.0;
float W_Current_real = 0.0;

//合計の電流値
float Current_SUM = 0.0;

/* ベクトル制御用変数 */
//dq電流
float Id = 0.0;
float Iq = 0.0;

//αβ電流
float Ia = 0.0;
float Ib = 0.0;

//dq電圧
float Vd = 0.0;
float Vq = 0.0;

//αβ電圧
float Va = 0.0;
float Vb = 0.0;

//PWM
int U_PWM = 0;
int V_PWM = 0;
int W_PWM = 0;

//PI制御用構造体
PI_t Id_PI;
PI_t Iq_PI;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//mapf関数
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
	return((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

//sinテーブルを生成
//void sin_table_generate(){
//	//1要素あたりに進むラジアン角を計算
//	one_step = (2 * PI) / sin_num;
//
//	//サインテーブルを生成
//	for(int i = 0; i < sin_num; i++){
//		sin_table[i] = sinf(i * one_step);
//	}
//
//}

//clarke変換
void clarke_transform(){
	Ia = U_Current_real;
	Ib = (U_Current_real + 2 * V_Current_real) / sqrtf(3.0);
}

//park変換
void park_transform(){
	Id = Ia * ADC_cos + Ib * ADC_sin;
	Iq = -Ia * ADC_sin + Ib * ADC_cos;

	Id_PI.now_value = Id;
	Iq_PI.now_value = Iq;
}

//逆clarke変換
void inv_clarke_transform(){
	U_PWM = (int)Va;
	V_PWM = (int)((sqrtf(3.0) * Vb - Va) / 2);
	W_PWM = (int)((-sqrtf(3.0) * Vb - Va) / 2);
}

//逆park変換
void inv_park_transform(){
	Va = Vd * ADC_cos - Vq * ADC_sin;
	Vb = Vd * ADC_sin + Vq * ADC_cos;
}

//PI制御
float PI_Control(PI_t *hPI){
	hPI->error = hPI->goal - hPI->now_value;
	hPI->error_integral += hPI->error;

	if(hPI->ki != 0){
		if(hPI->error_integral * hPI->ki > hPI->integral_limit){
			hPI->error_integral = hPI->integral_limit / hPI->ki;
		}
		else if(hPI->error_integral * hPI->ki < -hPI->integral_limit){
			hPI->error_integral = -hPI->integral_limit / hPI->ki;
		}
	}

	return (hPI->kp * hPI->error + hPI->ki * hPI->error_integral);
}

//初期化時の電流センサの出力電圧を計算
void init_voltage_sensing(){
	for(int i = 0; i < 30; i++){
		//U相の電流値を取得
		ADC1_Config.Channel = ADC_CHANNEL_3;
		HAL_ADC_ConfigChannel(&hadc1, &ADC1_Config);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		U_voltage_init += HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		//V相の電流値を取得
		ADC1_Config.Channel = ADC_CHANNEL_2;
		HAL_ADC_ConfigChannel(&hadc1, &ADC1_Config);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		V_voltage_init += HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		//W相の電流値を取得
		ADC1_Config.Channel = ADC_CHANNEL_1;
		HAL_ADC_ConfigChannel(&hadc1, &ADC1_Config);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		W_voltage_init += HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}

	U_voltage_init = U_voltage_init / 30;
	V_voltage_init = V_voltage_init / 30;
	W_voltage_init = W_voltage_init / 30;
}


//各相の電流を計算
void current_sensing(){
	//U相の電流値を取得
	ADC1_Config.Channel = ADC_CHANNEL_3;
	HAL_ADC_ConfigChannel(&hadc1, &ADC1_Config);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	U_voltage = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	//V相の電流値を取得
	ADC1_Config.Channel = ADC_CHANNEL_2;
	HAL_ADC_ConfigChannel(&hadc1, &ADC1_Config);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	V_voltage = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	//W相の電流値を取得
	ADC1_Config.Channel = ADC_CHANNEL_1;
	HAL_ADC_ConfigChannel(&hadc1, &ADC1_Config);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	W_voltage = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	//現在の電流値を算出
	U_Current_now = (((float)U_voltage - (float)U_voltage_init) * (3.3/4095)) / 0.05;
	V_Current_now = (((float)V_voltage - (float)V_voltage_init) * (3.3/4095)) / 0.05;
	W_Current_now = (((float)W_voltage - (float)W_voltage_init) * (3.3/4095)) / 0.05;

	/*	移動平均を計算	  */

	//サンプルの値を一つずらす
	for(int i = 0; i < sample_num-1; i++){
	  U_Current_samples[i] = U_Current_samples[i + 1];
	  V_Current_samples[i] = V_Current_samples[i + 1];
	  W_Current_samples[i] = W_Current_samples[i + 1];
	}

	//新しい値(現在の電流値)を最新の値として代入
	U_Current_samples[sample_num - 1] = U_Current_now;
	V_Current_samples[sample_num - 1] = V_Current_now;
	W_Current_samples[sample_num - 1] = W_Current_now;

	//サンプルの総和を初期化
	U_Current_sum = 0;
	V_Current_sum = 0;
	W_Current_sum = 0;

	//サンプルの総和を計算
	for(int i = 0; i < sample_num; i++){
	  U_Current_sum += U_Current_samples[i];
	  V_Current_sum += V_Current_samples[i];
	  W_Current_sum += W_Current_samples[i];
	}

	//サンプル内の平均値を計算
	U_Current_ave = U_Current_sum / sample_num;
	V_Current_ave = V_Current_sum / sample_num;
	W_Current_ave = W_Current_sum / sample_num;

	//移動平均の結果のUVWの総和電流を3相ブリッジに流れ込む電流の現在値として計算
	Current_SUM = U_Current_ave + V_Current_ave + W_Current_ave;

	//3総電流を導出
	U_Current_real = U_Current_ave;
	V_Current_real = V_Current_ave;
//	W_Current_real = -U_Current_real - V_Current_real;
	W_Current_real = W_Current_ave;

}

//電気角の取得
void degree_sensing(){
	//電気角(sin)の値を取得
	ADC2_Config.Channel = ADC_CHANNEL_1;
	HAL_ADC_ConfigChannel(&hadc2, &ADC2_Config);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 10);
	ADC_sin = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);

	//電気角(cos)の値を取得
	ADC2_Config.Channel = ADC_CHANNEL_2;
	HAL_ADC_ConfigChannel(&hadc2, &ADC2_Config);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 10);
	ADC_cos = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);

	ADC_sin = mapf(ADC_sin, 0, 4095, 0, 3.3);
	ADC_cos = mapf(ADC_cos, 0, 4095, 0, 3.3);

	ADC_sin = (ADC_sin - ADC_sin_min - (ADC_sin_max - ADC_sin_min)/2) / ((ADC_sin_max - ADC_sin_min)/2);
	ADC_cos = (ADC_cos - ADC_cos_min - (ADC_cos_max - ADC_cos_min)/2) / ((ADC_cos_max - ADC_cos_min)/2);


	/*  エンコーダーのmax,min測定用	 */
//	if(ADC_sin_max < ADC_sin){
//		ADC_sin_max = ADC_sin;
//	}
//
//	if(ADC_sin_min > ADC_sin){
//		ADC_sin_min = ADC_sin;
//	}
//
//	if(ADC_cos_max < ADC_cos){
//		ADC_cos_max = ADC_cos;
//	}
//
//	if(ADC_cos_min > ADC_cos){
//		ADC_cos_min = ADC_cos;
//	}
}

//タイマ割り込み関数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2){
    	tim2_intrupt_flag = true;
    }

    if(htim == &htim1){
    	tim1_intrupt_flag = true;
    }
}

//printfの中身
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1,(uint8_t *)ptr,len,10);
  return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  setbuf(stdout, NULL);
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
  MX_ADC2_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //ADCキャリブレーション
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  //タイマ割り込みスタート
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim1);

  //タイマースタート
  HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER, U_PHASE_PWM_CHANNEL);
  HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER, V_PHASE_PWM_CHANNEL);
  HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER, W_PHASE_PWM_CHANNEL);

  //IR2302のShutDownを解除
  HAL_GPIO_WritePin(U_SD_GPIO_Port, U_SD_Pin, SET);
  HAL_GPIO_WritePin(V_SD_GPIO_Port, V_SD_Pin, SET);
  HAL_GPIO_WritePin(W_SD_GPIO_Port, W_SD_Pin, SET);

  //すべてのDuty比を0.5に
  SET_PWM_DUTY(U, 500);
  SET_PWM_DUTY(V, 500);
  SET_PWM_DUTY(W, 500);

  //ADCの初期値を取得
  init_voltage_sensing();

  //d軸電流のパラメータを設定
  Id_PI.goal = 0.0;
  Id_PI.kp = 0.5;
  Id_PI.ki = 0.1;
  Id_PI.integral_limit = 100;

  //q軸電流のパラメータを設定
  Iq_PI.goal = 1.0;
  Iq_PI.kp = 0.5;
  Iq_PI.ki = 0.1;
  Iq_PI.integral_limit = 100;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(tim2_intrupt_flag){
		  //M2006から角度取得
		  degree_sensing();
		  //Iu,Iv,Iwを計測
		  current_sensing();
		  //Iu,Iv,Iw -> Iα,Iβ
		  clarke_transform();
		  //Iα,Iβ -> Id,Iq
		  park_transform();

		  //Id,IqをPI制御
		  Vd = PI_Control(&Id_PI);
		  Vq = PI_Control(&Iq_PI);

		  //Vd,Vq -> Vα,Vβ
		  inv_park_transform();
		  //Vα,Vβ -> Iu,Iv,Iw
		  inv_clarke_transform();

		  //PWMをセット
		  SET_PWM_DUTY(U, 500 + U_PWM);
		  SET_PWM_DUTY(V, 500 + V_PWM);
		  SET_PWM_DUTY(W, 500 + W_PWM);
//		  printf("sin:%f cos:%f\r\n", ADC_sin, ADC_cos);

		  //割り込みのフラグをリセット
		  tim2_intrupt_flag = false;
	  }

	  if(tim1_intrupt_flag){
//		  Iq_PI.goal = Iq_PI.goal * -1;
//		  printf("Id:%.2f Iq:%.2f\r\n", Id, Iq);

		  //割り込みのフラグをリセット
		  tim1_intrupt_flag = false;
	  }


	  if(2 > Iq){
		  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, RESET);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, RESET);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, SET);
	  }

	  if(5 > Iq && Iq > 3){
		  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, SET);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SET);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, RESET);
	  }

	  if(Iq > 5){
		  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, SET);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, RESET);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, RESET);
	  }

//	  printf("sin:%f cos:%f\r\n", ADC_sin, ADC_cos);
//	  printf("U:%f V:%f W:%f\r\n", U_Current_real, V_Current_real, W_Current_real);
//	  printf("U:%f V:%f W:%f\r\n", U_Current_now, V_Current_now, W_Current_now);
//	  printf("%d %d %d \r\n", U_voltage_init, V_voltage_init, W_voltage_init);
//	  printf("%f\r\n", Current_SUM);
//	  printf("Id:%f Iq:%f\r\n", Id, Iq);
//	  U_PWM = (int)(mapf(sin_table[(int)(loop_count%sin_num)], -1.0, 1.0, min_duty, max_duty));
//	  V_PWM = (int)(mapf(sin_table[(int)((loop_count + sin_num / 3)%(sin_num))], -1.0, 1.0, min_duty, max_duty));
//	  W_PWM = (int)(mapf(sin_table[(int)((loop_count + (sin_num * 2) / 3)%(sin_num))], -1.0, 1.0, min_duty, max_duty));

//	  SET_PWM_DUTY(U, U_PWM);
//	  SET_PWM_DUTY(V, V_PWM);
//	  SET_PWM_DUTY(W, W_PWM);
//	  loop_count = loop_count + 10;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV4;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL15;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  ADC1_Config = sConfig;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
  ADC2_Config = sConfig;
  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  htim1.Init.Prescaler = 6000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 500000;
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

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, W_SD_Pin|U_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, V_SD_Pin|LED_G_Pin|LED_R_Pin|LED_B_Pin
                          |test_GPIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : W_SD_Pin U_SD_Pin */
  GPIO_InitStruct.Pin = W_SD_Pin|U_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : V_SD_Pin LED_G_Pin LED_R_Pin LED_B_Pin
                           test_GPIO_Pin */
  GPIO_InitStruct.Pin = V_SD_Pin|LED_G_Pin|LED_R_Pin|LED_B_Pin
                          |test_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, SET);
	  printf("error\r\n");
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
