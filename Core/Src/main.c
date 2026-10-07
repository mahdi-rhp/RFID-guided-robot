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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "rc522.h"
#include "stdio.h"
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_DUTY 300
#define STRAIGHT_TOLERANCE .80     // Allowed yaw angle deviation (in degrees)
#define PWM_BASE_SPEED 300         // Base motor speed
#define PWM_ADJUST 80              // Speed adjustment for correction
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t CardID[MFRC522_MAX_LEN];
uint8_t LastCardID[4] = {0};
uint8_t Turn_Right[4]={0x55,0xF6,0x41,0x9A};
uint8_t Turn_Left[4]={0xF1,0x13,0xBE,0x1F};
uint8_t Turn_Around[4]={0xD3,0x43,0x50,0x1A};  //1A 50 43 D3 
uint8_t Repeat_Command_Tag[4] = {0x83, 0x1A, 0xBB, 0x27}; //27 BB 1A 83 
uint8_t Stop_Command_Tag[4] = {  0x06, 0xA8, 0xDB};// DB A8 06 3
///// E13FF23 ///// replace
uint8_t t;


MPU6050_t MPU6050;
double yaw_angle = 0;
uint32_t lastTick = 0;
double gyroZ_offset = 1.142;

double yaw_error_integral = 0;  // ????? ???? ????? (???? ???????)
double Kp = 120.0;              // ???? ?????? (??? ?? ?? 100 ?? 200)
double Ki = 10.0;               // ???? ???????? (??? ?? ?? 5 ?? 20)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Update_Yaw(double* yaw_angle, uint32_t* lastTick, double gyroZ_offset);
void PerformRotation(double target_angle, void (*TurnMotorFunc)(void),
                     TIM_HandleTypeDef* htim_a, uint32_t channel_a,
                     TIM_HandleTypeDef* htim_b, uint32_t channel_b);
void Maintain_Straight_Line(double* yaw_angle, uint32_t* lastTick, double gyroZ_offset);
void PerformRotation_PI(double target_angle);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

void Motor_Forward(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_DUTY);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_DUTY);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

void Motor_Backward(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 400);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 400);
}

void Motor_TurnRight(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_DUTY);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_DUTY);
}

void Motor_TurnLeft(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_DUTY);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_DUTY);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

void Update_Yaw(double* yaw_angle, uint32_t* lastTick, double gyroZ_offset)
{
    MPU6050_Read_All(&hi2c1, &MPU6050);
    uint32_t now = HAL_GetTick();
    double dt = (now - *lastTick) / 1000.0;
    *lastTick = now;
    double Gz_corrected = MPU6050.Gz - gyroZ_offset;
    *yaw_angle += Gz_corrected * dt;	
}

void PerformRotation(double target_angle, void (*TurnMotorFunc)(void),
                     TIM_HandleTypeDef* htim_a, uint32_t channel_a,
                     TIM_HandleTypeDef* htim_b, uint32_t channel_b)
{
    TurnMotorFunc();

    if (target_angle > 0)  // 
    {
        while (yaw_angle < target_angle)
        {
            Update_Yaw(&yaw_angle, &lastTick, gyroZ_offset);

            if (yaw_angle > target_angle - 15.0)
            {
                __HAL_TIM_SET_COMPARE(htim_a, channel_a, 200);
                __HAL_TIM_SET_COMPARE(htim_b, channel_b, 200);
            }

            HAL_Delay(5);
        }
    }
    else if (target_angle < 0)  // 
    {
        while (yaw_angle > target_angle)
        {
            Update_Yaw(&yaw_angle, &lastTick, gyroZ_offset);

            if (yaw_angle < target_angle + 15.0)
            {
                __HAL_TIM_SET_COMPARE(htim_a, channel_a, 200);
                __HAL_TIM_SET_COMPARE(htim_b, channel_b, 200);
            }

            HAL_Delay(5);
        }
    }

    Motor_Stop();
		HAL_Delay(200);
		
		    if (target_angle > 0)  // 
    {
        while (yaw_angle < target_angle)
        {
            Update_Yaw(&yaw_angle, &lastTick, gyroZ_offset);

            if (yaw_angle > target_angle - 5.0)
            {
                __HAL_TIM_SET_COMPARE(htim_a, channel_a, 180);
                __HAL_TIM_SET_COMPARE(htim_b, channel_b, 180);
            }

            HAL_Delay(5);
        }
    }
    else if (target_angle < 0)  // 
    {
        while (yaw_angle > target_angle)
        {
            Update_Yaw(&yaw_angle, &lastTick, gyroZ_offset);

            if (yaw_angle < target_angle + 5.0)
            {
                __HAL_TIM_SET_COMPARE(htim_a, channel_a, 180);
                __HAL_TIM_SET_COMPARE(htim_b, channel_b, 180);
            }

            HAL_Delay(5);
        }
    }

    Motor_Stop();
		
		
    yaw_angle = 0;
}


void Maintain_Straight_Line(double* yaw_angle, uint32_t* lastTick, double gyroZ_offset)
{
    // Update yaw angle using gyroscope
    Update_Yaw(yaw_angle, lastTick, gyroZ_offset);

    if (*yaw_angle > STRAIGHT_TOLERANCE) {
        // Deviating to the left – increase left wheel speed, decrease right wheel speed
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_BASE_SPEED + PWM_ADJUST); // Left motor
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_BASE_SPEED - PWM_ADJUST); // Right motor
    }
    else if (*yaw_angle < -STRAIGHT_TOLERANCE) {
        // Deviating to the right – increase right wheel speed, decrease left wheel speed
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_BASE_SPEED - PWM_ADJUST); // Left motor
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_BASE_SPEED + PWM_ADJUST); // Right motor
    }
    else {
        // Moving straight – keep both motors at base speed
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_BASE_SPEED); // Left motor
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_BASE_SPEED); // Right motor
    }
}

void Maintain_Straight_Line_PI(double* yaw_angle, uint32_t* lastTick, double gyroZ_offset)
{
    // 1. ?????? ????? ????
    MPU6050_Read_All(&hi2c1, &MPU6050);
    uint32_t now = HAL_GetTick();
    double dt = (now - *lastTick) / 1000.0;
    *lastTick = now;

    double Gz_corrected = MPU6050.Gz - gyroZ_offset;
    *yaw_angle += Gz_corrected * dt;

    // 2. ?????? ??? (???????? ????? ??? ????)
    double error = *yaw_angle;  // ??? ?? ???? ????? ?? (yaw ????)? ???? ?? ????? ??

    // 3. ??????????? ??????? ???
    yaw_error_integral += error * dt;

    // 4. ?????? ????? ????? PI
    double control_output = Kp * error + Ki * yaw_error_integral;

    // 5. ????????? ????? ???? ??????? ?? ?????
    if (control_output > PWM_ADJUST) control_output = PWM_ADJUST;
    if (control_output < -PWM_ADJUST) control_output = -PWM_ADJUST;

    // 6. ????? ???? ???????
    int left_speed = PWM_BASE_SPEED + control_output;
    int right_speed = PWM_BASE_SPEED - control_output;

    // ????????? ????? ???? ????? ???? ?? ???? ???? ???
    if (left_speed < 0) left_speed = 0;
    if (right_speed < 0) right_speed = 0;
    if (left_speed > 1000) left_speed = 1000;
    if (right_speed > 1000) right_speed = 1000;

    // 7. ????? ???? ?? ???????
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, left_speed);  // Left motor
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right_speed); // Right motor
}

void PerformRotation_PI(double target_angle)
{
    yaw_angle = 0;
    yaw_error_integral = 0;

    double error, control_output;
    double Kp_rot = 6.0;
    double Ki_rot = 0.5;
    const int pwm_min = 200;
    const int pwm_max = 400;
    const double angle_tolerance = 1.0;

    uint32_t last_time = HAL_GetTick(); // ???? ?????? dt

    // --- ????? ???: ???? ?? PI ---
    while (fabs(target_angle - yaw_angle) > angle_tolerance)
    {
        Update_Yaw(&yaw_angle, &lastTick, gyroZ_offset);

        uint32_t now = HAL_GetTick();
        double dt = (now - last_time) / 1000.0;
        last_time = now;

        error = target_angle - yaw_angle;
        yaw_error_integral += error * dt;

        control_output = Kp_rot * error + Ki_rot * yaw_error_integral;

        // ????????? ?????
        if (control_output > pwm_max) control_output = pwm_max;
        else if (control_output < -pwm_max) control_output = -pwm_max;

        if (control_output > 0 && control_output < pwm_min) control_output = pwm_min;
        else if (control_output < 0 && control_output > -pwm_min) control_output = -pwm_min;

        // ????? ?? ???????
        if (control_output > 0) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, control_output); // ????? ?? ???
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, control_output); // ????? ???? ???
        } else {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, -control_output); // ????? ?? ???
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -control_output); // ????? ???? ???
        }

        HAL_Delay(10);
    }

    Motor_Stop();
    HAL_Delay(100);

    // --- ????? ???: ????? ????? ?? ???? over/under-shoot ---
    double correction_pwm = 200;
    double correction_tolerance = 2.0;

    if (yaw_angle > target_angle + correction_tolerance) {
        // ??? ?? ?? ?????? – ????? ?? ????
        while (yaw_angle > target_angle + correction_tolerance / 2.0) {
            Update_Yaw(&yaw_angle, &lastTick, gyroZ_offset);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, correction_pwm); // ?? ???
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, correction_pwm); // ???? ???
            HAL_Delay(10);
        }
    }
		else if (yaw_angle < target_angle - correction_tolerance) {
        // ???? ?????? – ????? ?? ??
        while (yaw_angle < target_angle - correction_tolerance / 2.0) {
            Update_Yaw(&yaw_angle, &lastTick, gyroZ_offset);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, correction_pwm); // ?? ???
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, correction_pwm); // ???? ???
            HAL_Delay(10);
        }
    }
    HAL_Delay(100);
    Motor_Stop();
    yaw_angle = 0;
}

void Maintain_Straight_Line_PI_Backward(double* yaw_angle, uint32_t* lastTick, double gyroZ_offset)
{
    // 1. ?????? ????? ????
    MPU6050_Read_All(&hi2c1, &MPU6050);
    uint32_t now = HAL_GetTick();
    double dt = (now - *lastTick) / 1000.0;
    *lastTick = now;

    double Gz_corrected = MPU6050.Gz - gyroZ_offset;
    *yaw_angle += Gz_corrected * dt;

    // 2. ?????? ??? (???????? ????? ??? ????)
    double error = *yaw_angle;  // ??? ?? ???? ????? ?? (yaw ????)? ???? ?? ????? ??

    // 3. ??????????? ??????? ???
    yaw_error_integral += error * dt;

    // 4. ?????? ????? ????? PI
    double control_output = Kp * error + Ki * yaw_error_integral;

    // 5. ????????? ????? ???? ??????? ?? ?????
    if (control_output > PWM_ADJUST) control_output = PWM_ADJUST;
    if (control_output < -PWM_ADJUST) control_output = -PWM_ADJUST;

    // 6. ????? ???? ???????
    int left_speed = PWM_BASE_SPEED + control_output;
    int right_speed = PWM_BASE_SPEED - control_output;

    // ????????? ????? ???? ????? ???? ?? ???? ???? ???
    if (left_speed < 0) left_speed = 0;
    if (right_speed < 0) right_speed = 0;
    if (left_speed > 1000) left_speed = 1000;
    if (right_speed > 1000) right_speed = 1000;

    // 7. ????? ???? ?? ???????
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, left_speed);  // Left motor
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, right_speed); // Right motor
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	  HAL_Delay(100);

  MFRC522_Init();
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  // HAL_UART_Transmit(&huart1, (uint8_t*)"helo,RC522 is ready. lets go!\r\n", 32, HAL_MAX_DELAY); // UART for RFID init message

  MPU6050_Init(&hi2c1);
  while (MPU6050_Init(&hi2c1) == 1);
  // HAL_UART_Transmit(&huart1, (uint8_t*)"STM32F103 Mifare RC522 RFID Card reader\r\n", 42, 500); // UART for MPU6050 init

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  typedef enum {
      NORMAL_STATE,
      ROTATING_RIGHT,
      ROTATING_LEFT,
      ROTATING_AROUND,
		  STOPPED_STATE
  } RobotState_t;

  RobotState_t RobotState = NORMAL_STATE;
	RobotState_t LastCommand = NORMAL_STATE;



  HAL_Delay(4000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,1);
	
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		Update_Yaw(&yaw_angle, &lastTick, gyroZ_offset);

    switch (RobotState)
    {
      case NORMAL_STATE:
        // HAL_UART_Transmit(&huart1, (uint8_t*)"everything is good\r\n", 22, HAL_MAX_DELAY); // UART normal state message
		  	Maintain_Straight_Line_PI(&yaw_angle, &lastTick, gyroZ_offset);
     //   Maintain_Straight_Line(&yaw_angle, &lastTick, gyroZ_offset);
      //     Motor_Forward();
        if (!MFRC522_Request(PICC_REQIDL, CardID))
        {
          if (!MFRC522_Anticoll(CardID))
          {
            if (memcmp(CardID, LastCardID, 4))
            {
              memcpy(LastCardID, CardID, 4);
							yaw_angle = 0;
						  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);
	            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);

              Motor_Stop();
              HAL_Delay(700);
						//	Maintain_Straight_Line_PI_Backward(&yaw_angle, &lastTick, gyroZ_offset);
              Motor_Backward();
              HAL_Delay(450);
              Motor_Stop();
							
              HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
	            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
              // HAL_UART_Transmit(&huart1, (uint8_t*)"stop!!!!\r\n", 11, HAL_MAX_DELAY); // UART stop message

              if (memcmp(LastCardID, Turn_Right, 4) == 0)
               {
                  LastCommand = ROTATING_RIGHT;
                  RobotState = ROTATING_RIGHT;
               }
              else if (memcmp(LastCardID, Turn_Left, 4) == 0)
              {
                  LastCommand = ROTATING_LEFT;
                  RobotState = ROTATING_LEFT;
              }
              else if (memcmp(LastCardID, Turn_Around, 4) == 0)
              {
                 LastCommand = ROTATING_AROUND;
                 RobotState = ROTATING_AROUND;
              }
              else if (memcmp(LastCardID, Repeat_Command_Tag, 4) == 0)
              {
                   RobotState = LastCommand; 
              }
							else if (memcmp(LastCardID, Stop_Command_Tag, 4) == 0)
              {
                   RobotState = STOPPED_STATE; 
              }

            }
          }
        }
        else {
          t++;
          if (t > 4) {
            memset(LastCardID, 0, 4);
            t = 0;
          }
        }
        break;

      case ROTATING_RIGHT:
        PerformRotation(-90.0, Motor_TurnRight, &htim3, TIM_CHANNEL_3, &htim2, TIM_CHANNEL_4);
			//  PerformRotation_PI(-90.0);
        RobotState = NORMAL_STATE;
        break;

      case ROTATING_LEFT:
        PerformRotation(90.0, Motor_TurnLeft, &htim3, TIM_CHANNEL_4, &htim2, TIM_CHANNEL_3);
		 // 	PerformRotation_PI(90.0);
        RobotState = NORMAL_STATE;
        break;

      case ROTATING_AROUND:
        PerformRotation(-180.0, Motor_TurnRight, &htim3, TIM_CHANNEL_3, &htim2, TIM_CHANNEL_4);
		//  	PerformRotation_PI(-180.0);
        RobotState = NORMAL_STATE;
        break;
			case STOPPED_STATE:
        Motor_Stop(); 
        break;

    }

    HAL_Delay(50);
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
