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
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "math.h"
#include "pca9685.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SERVO_COUNT	4
uint8_t data[4] = {0};
uint16_t data_16 = 0;
uint16_t data_r = 0;
uint16_t data_angle = 0;
float speed = 0;
float speed_r = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
PIDController MPID;
#define MKp 1.14
#define MKi 25.59
#define MKd 0.0

// need to configuration
#define CPR 64
#define Sample_time 10 // ms

#define pi 3.1415
#define Motor1 0
#define Motor2 1
volatile uint8_t nowA[2];
volatile uint8_t nowB[2];
volatile uint8_t lastA[2];
volatile uint8_t lastB[2];
volatile uint8_t dir[2];
uint16_t cnt[2];
uint16_t Enc_count[2];

uint16_t count[2]; // count pulse from encoder
uint16_t new_count[2];
uint8_t count_state[2];
uint16_t diff[2]; // difference between count and new_count in a sample time

float speedM[2];
float rdps[2];

float Motor1_speed = 0;
float Motor2_speed = 0;
float V1 = 0; // target speed of motor1
float V2 = 0; // target speed of motor2
float pwm_M1 = 0;
float pwm_M2 = 0;
float pwm_r = 0;
int i =0;
int t = 0;
int r = 0;
//===== Limit switch =====
uint8_t data_limit;

//===== Pick up angle =====
int pick_data;
uint8_t degree =85;

// === Limit servo ====
uint8_t angle = 0;
int shooter_limit;

int j = 0;
int c = 0;

// === data of task complete ===
uint8_t pick = 0;
uint8_t fall = 0;
uint8_t stop_fall = 0;
uint8_t shoot = 0;
uint8_t store = 0;
uint8_t close = 0;
uint8_t open = 0;
uint8_t push = 0;
uint8_t control_fall = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[3];
uint32_t TxMailbox;
uint8_t cntt;

float map(float Input, float Min_Input, float Max_Input, float Min_Output, float Max_Output)
{

	return (float)((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	cntt++;
	while (cntt - 100 > 0)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
		cntt = 0;
	}
	if (RxHeader.DLC == 2 && RxHeader.StdId == 0x222)
	{
			data_16 = (RxData[0] << 8) | RxData[1];
			if(data_16 != 0)
			{
				shoot = 1;
			}
			else
			{
				shoot = 0;
			}
			speed = map(data_16,0,65535,0,1500);
	}

	if (RxHeader.DLC == 1 && RxHeader.StdId == 0x444)
	{
		data_limit = RxData[0];
	}
	if (RxHeader.DLC == 1 && RxHeader.StdId == 0x666)
	{
		if (RxData[0] == 1) // start to pick up
		{
			degree = 180;
			pick = 0;
			store = 0;
			push = 0;
		}
		if(RxData[0] == 0) // after pick up
		{
			degree = 85;

			push = 0;
			control_fall = 0;
		}

		else if(RxData[0] == 2) //start shoot
		{
			degree = 85;
			store = 1;
			pick = 1;
		}
		else if(RxData[0] == 3)
		{
			degree = 85;
			control_fall = 1;

		}
	}
}
void servo_rotation(uint8_t degree)
{
	float y = 0.556 * (float) degree + 25;
	uint8_t pwm = round(y);
	TIM3->CCR2 = pwm;
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t ActiveServo;
void pick_up(int deg, int t)
{
	degree = deg;
	 if(degree >=0)
		  {

		  	while(i<degree)
		  	{
		  		servo_rotation(i);
		  		HAL_Delay(t);
		  		i = i + 1;

		  	}

		  	if(degree < 250)
		  	{
		  		while(i>degree)
		  		{
		  			servo_rotation(i);
		  			HAL_Delay(t);
		  			i = i - 1;
		  		}
		  	}
		  }

}

void ring_falling()
{
	PCA9685_SetServoAngle(4, 90);
	PCA9685_SetServoAngle(5, 90);
	fall = 0;
}
void ring_back()
{
	PCA9685_SetServoAngle(4, 180);
	PCA9685_SetServoAngle(5, 180);
}
void ring_stop_falling()
{
	PCA9685_SetPin(4, 4096, 0);
	PCA9685_SetPin(5, 4096, 0);
}
void open_pickup()
{
	PCA9685_SetServoAngle(0,0);
	PCA9685_SetServoAngle(1, 180);
	open = 1;
}
void close_pickup()
{
	PCA9685_SetServoAngle(0, 140);
	PCA9685_SetServoAngle(1, 40);
	open = 0;
}
void fall1_function()
{
	pick_up(92,10);
	HAL_Delay(200);
	open_pickup();
	HAL_Delay(400);
	ring_falling();
	HAL_Delay(600);
	ring_stop_falling();
	HAL_Delay(400);
//	pick_up(78,10);
//	HAL_Delay(200);
	close_pickup();
	HAL_Delay(400);
	ring_falling();
	HAL_Delay(1000);
	ring_stop_falling();
	pick_up(125,10);
}
void fall1_5_function()
{
	pick_up(95,10);
	HAL_Delay(200);
	open_pickup();
	HAL_Delay(400);
	ring_falling();
	HAL_Delay(400);
	ring_stop_falling();
	HAL_Delay(400);
//	pick_up(78,10);
//	HAL_Delay(200);
	close_pickup();
	HAL_Delay(400);
	ring_falling();
	HAL_Delay(1000);
	ring_stop_falling();
	pick_up(125,10);
}
void fall2_function()
{
	pick_up(95,10);
	HAL_Delay(200);
	open_pickup();
	HAL_Delay(600);
	ring_falling();
	HAL_Delay(1000);
	ring_stop_falling();
	HAL_Delay(700);
//	pick_up(78,10);
//	HAL_Delay(200);
	close_pickup();
	HAL_Delay(400);
	ring_falling();
	HAL_Delay(2000);
	ring_stop_falling();
	pick_up(125,12);



}
void fall2_5_function()
{
	pick_up(95,10);
	HAL_Delay(200);
	open_pickup();
	HAL_Delay(600);
	ring_falling();
	HAL_Delay(1700);
	ring_stop_falling();
	HAL_Delay(700);
//	pick_up(78,10);
//	HAL_Delay(200);
	close_pickup();
	HAL_Delay(400);
	ring_falling();
	HAL_Delay(2000);
	ring_stop_falling();
	pick_up(125,15);



}
void shooter1_function(float input_speed)
{
	pick_up(125,10);
	HAL_Delay(1200);
	speed = input_speed;
	HAL_Delay(700);
	data_r = 1;
	HAL_Delay(400);
	data_r = 0;
	HAL_Delay(500);
	speed = 0;
	shoot = 0;
	pick_up(90,10);

}
void shooter2_function(float input_speed)
{
		pick_up(125,10);
		HAL_Delay(1200);
		speed = input_speed;
		HAL_Delay(700);
		data_r = 1;
		HAL_Delay(400);
		data_r = 0;
		HAL_Delay(500);
		speed = 0;
		shoot = 0;
		pick_up(90,10);

}
void shooter2_5_function(float input_speed)
{
		pick_up(125,10);
		HAL_Delay(1200);
		speed = input_speed;
		HAL_Delay(700);
		data_r = 1;
		HAL_Delay(400);
		data_r = 0;
		HAL_Delay(500);
		speed = 0;
		shoot = 0;
		pick_up(90,10);

}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t encoder(int i)
{
	if (nowA[i] != lastA[i])
	{
		lastA[i] = nowA[i];
		if (lastA[i] == 0)
		{
			if (nowB[i] == 0)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
		else
		{
			if (nowB[i] == 1)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	if (nowB[i] != lastB[i])
	{
		lastB[i] = nowB[i];
		if (lastB[i] == 0)
		{
			if (nowA[i] == 1)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
		else
		{
			if (nowA[i] == 0)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	return cnt[i];
}
float Motors_RPS(int j, float SampleTime, float N_round)
{
	new_count[Motor1] = Enc_count[0];
	new_count[Motor2] = Enc_count[1];

	count_state[Motor1] = !dir[0];
	count_state[Motor2] = !dir[1];

	if (count_state[j])
	{
		if (new_count[j] <= count[j])
		{ // Check for counter underflow
			diff[j] = count[j] - new_count[j];
		}
		else
		{
			diff[j] = (65536 - new_count[j]) + count[j];
		}
		speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime) * -1.0;
	}
	else
	{
		if (new_count[j] >= count[j])
		{ // Check for counter overflow
			diff[j] = new_count[j] - count[j];
		}
		else
		{
			diff[j] = (65536 - count[j]) + new_count[j];
		}
		speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime);
	}

	rdps[j] = -2.0f * pi * speedM[j];
	count[j] = new_count[j];

	return rdps[j];
}

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
	PID_Init(&MPID, 2);
	MPID.T = 0.01; // T = 20ms
	MPID.limMax = 1000;
	MPID.limMin = -10;
	MPID.limMaxInt = 1000;
	MPID.limMinInt = -10;
	MPID.tau = 0; // for K

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  // TIMER INTERUPT
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);				// M1
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);				// M2
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);				// M1
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);				// Servo limit
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);				// Servo pickup

	HAL_CAN_Start(&hcan);

	// Activate the notification
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

	PCA9685_Init(&hi2c2);
	PCA9685_SetServoAngle(0, 140);
	PCA9685_SetServoAngle(1, 40);
	PCA9685_SetPin(4, 4096, 0);
	PCA9685_SetPin(5, 4096, 0);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  pick_up(degree,10);
	  if(HAL_GetTick() < 5000){
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
		  HAL_Delay(100);
	  }
	  if(pick == 1 && push == 0)
	  {
		  if(c<5)
		  {
			  fall1_function();
		  }
		  else if (c>=3 && c<7)
		  {
			  fall1_function();
		  }
		  else if(c>=7&&c<9){
			  fall2_function();
		  }
		else if(c>=8)
			{
				fall2_5_function();
				if(c ==10)
				{
					c = 0;
				}
			}
		  c = c+1;
		  pick = 0;
	  }
	  if(shoot == 1){
		  if(c<5)
		  {
			  shooter1_function(speed);
		  }
		  else if(c>=5){
		  		shooter2_function(speed);
		  		if(c == 10)
		  		{
		  			c = 0;
		  		}
		  	}

	  }
	  if(control_fall == 1)
	  {
		  ring_falling();
	  }
	  else if(control_fall == 0)
	  {
		  ring_stop_falling();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == En1_C1_Pin || En1_C2_Pin)
	{ // ENCODER Motor 1
		nowA[0] = HAL_GPIO_ReadPin(En1_C1_GPIO_Port, En1_C1_Pin);
		nowB[0] = HAL_GPIO_ReadPin(En1_C2_GPIO_Port, En1_C2_Pin);
		Enc_count[0] = encoder(0);
	}
	if (GPIO_Pin == En2_C1_Pin || En2_C2_Pin)
	{ // ENCODER Motor 2
		nowA[1] = HAL_GPIO_ReadPin(En2_C1_GPIO_Port, En2_C1_Pin);
		nowB[1] = HAL_GPIO_ReadPin(En2_C2_GPIO_Port, En2_C2_Pin);
		Enc_count[1] = encoder(1);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM2){
		// PID // need to change for using
		pwm_M1 = PID(&MPID, speed, Motor1_speed, MKp, MKi, MKd, Motor1);
		pwm_M2 = PID(&MPID, speed, Motor2_speed, MKp, MKi, MKd, Motor2);
		// feedback speed
//		pwm_M1 = speed;
//		pwm_M2 = speed;
		Motor1_speed = (float)fabs(Motors_RPS(Motor1, Sample_time, CPR));
		Motor2_speed = (float)fabs(Motors_RPS(Motor2, Sample_time, CPR));
		pwm_r = speed_r;

		if(data_r == 1 && data_limit == 0)
		{
			HAL_GPIO_WritePin(M1_dir_GPIO_Port, M1_dir_Pin, 1);
			pwm_r =450;
			TIM4->CCR1 = pwm_r;

		}
		else if(data_r == 1 && data_limit == 1)
		{
			HAL_GPIO_WritePin(M1_dir_GPIO_Port, M1_dir_Pin, 0);
			pwm_r = 0;
			TIM4->CCR1 = pwm_r;
		}
		else if(data_r == 0 && data_limit == 1)
		{
			HAL_GPIO_WritePin(M1_dir_GPIO_Port, M1_dir_Pin, 0);
			pwm_r = 450;
			TIM4->CCR1 = pwm_r;

		}
		else
		{
			HAL_GPIO_WritePin(M1_dir_GPIO_Port, M1_dir_Pin, 1);
			pwm_r = 0;
			TIM4->CCR1 = pwm_r;
		}

		// dir
		HAL_GPIO_WritePin(M2_dir_GPIO_Port, M2_dir_Pin, 0); // M2
		HAL_GPIO_WritePin(M3_dir_GPIO_Port, M3_dir_Pin, 0); // M1





		if (pwm_M1 > 20)
		{
			TIM4->CCR2 = pwm_M1;
		}
		else
		{
			TIM4->CCR2 = 0;
		}


		if (pwm_M2 > 20)
		{
			TIM4->CCR3 = pwm_M2;
		}
		else
		{
			TIM4->CCR3 = 0;
		}






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
