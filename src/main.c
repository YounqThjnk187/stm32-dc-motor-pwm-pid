/* USER CODE BEGIN Header */ 
/** 
***************************************************************************
 *** 
* @file           
: main.c 
* @brief          : Main program body 
***************************************************************************
 *** 
* @attention 
* 
* Copyright (c) 2025 STMicroelectronics. 
* All rights reserved. 
* 
* This software is licensed under terms that can be found in the LICENSE file 
* in the root directory of this software component. 
* If no LICENSE file comes with this software, it is provided AS-IS. 
* 
***************************************************************************
 *** 
*/ 
/* USER CODE END Header */ 
/* Includes ------------------------------------------------------------------*/ 
#include "main.h" 
/* Private includes ----------------------------------------------------------*/ 
/* USER CODE BEGIN Includes */ 
#include "i2c_lcd.h" 
#include <stdio.h> 
#include <string.h> 
#include <stdio.h> 
#include <math.h> 
/* USER CODE END Includes */ 
/* Private typedef -----------------------------------------------------------*/ 
/* USER CODE BEGIN PTD */ 
/* USER CODE END PTD */ 
/* Private define ------------------------------------------------------------*/ 
/* USER CODE BEGIN PD */ 
#define MAX_DUTY_CYCLE 100 
#define MIN_DUTY_CYCLE 0 
#define PULSES_PER_REV 20            // Số xung encoder mỗi vòng quay 
#define WHEEL_DIAMETER 0.026f        // Đường kính bánh xe (mét) 
#define PI 3.14159f 
#define PID_UPDATE_INTERVAL 100      // Thời gian cập nhật PID (ms) 
#define MAX_PID_OUTPUT MAX_DUTY_CYCLE 
#define MIN_PID_OUTPUT MIN_DUTY_CYCLE 
/* USER CODE END PD */ 
/* Private macro -------------------------------------------------------------*/ 
/* USER CODE BEGIN PM */ 
/* USER CODE END PM */ 
/* Private variables ---------------------------------------------------------*/ 
I2C_HandleTypeDef hi2c1; 
TIM_HandleTypeDef htim1; 
TIM_HandleTypeDef htim2; 
/* USER CODE BEGIN PV */ 
typedef struct { 
float Kp; 
float Ki; 
float Kd; 
float setpoint; 
float integral; 
float prev_error; 
float output; 
float prev_velocity; 
} PID_Controller; 
PID_Controller pid; 
int16_t encoder = 0; 
int16_t encoder_prev = 0; 
uint32_t time_now = 0; 
uint32_t time_prev = 0; 
float velocity = 0.0f; 
float target_velocity = 0.5f;  // Tốc độ đặt mặc định 0.5 m/s 
uint8_t current_duty_cycle = 0; 
I2C_LCD_HandleTypeDef my_lcd; 
char lcd_buffer[16]; 
// Biến lọc nhiễu 
#define FILTER_WINDOW_SIZE 5 
float velocity_history[FILTER_WINDOW_SIZE]; 
uint8_t history_index = 0; 
/* USER CODE END PV */ 
/* Private function prototypes -----------------------------------------------*/ 
void SystemClock_Config(void); 
static void MX_GPIO_Init(void); 
static void MX_TIM2_Init(void); 
static void MX_I2C1_Init(void); 
static void MX_TIM1_Init(void); 
void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd, float setpoint); 
void PID_Update(PID_Controller* pid, float actual_value, float dt); 
void set_motor_speed(uint8_t duty_cycle); 
void update_velocity_measurement(void); 
/* USER CODE BEGIN PFP */ 
/* USER CODE END PFP */ 
/* Private user code ---------------------------------------------------------*/ 
/* USER CODE BEGIN 0 */ 
/* PID Initialization */ 
void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd, float setpoint) { 
pid->Kp = Kp; 
pid->Ki = Ki; 
pid->Kd = Kd; 
pid->setpoint = setpoint; 
pid->integral = 0; 
pid->prev_error = 0; 
pid->output = 0; 
pid->prev_velocity = 0; 
// Khởi tạo bộ lọc 
for(int i=0; i<FILTER_WINDOW_SIZE; i++) { 
velocity_history[i] = 0; 
} 
} 
/* PID Update Calculation - Improved version */ 
/* Enhanced PID Update Calculation */ 
void PID_Update(PID_Controller* pid, float actual_value, float dt) { 
// Calculate error 
float error = pid->setpoint - actual_value; 
// Proportional term 
float proportional = pid->Kp * error; 
// Integral term with anti-windup 
pid->integral += error * dt; 
// Integral windup protection 
const float max_integral = (MAX_PID_OUTPUT * 0.8f) / pid->Ki; 
if (pid->integral > max_integral) pid->integral = max_integral; 
if (pid->integral < -max_integral) pid->integral = -max_integral; 
float integral = pid->Ki * pid->integral; 
// Derivative term with low-pass filtering 
float velocity_derivative = (actual_value - pid->prev_velocity) / dt; 
pid->prev_velocity = actual_value; 
// Low-pass filter for derivative 
static float filtered_derivative = 0; 
filtered_derivative = 0.3f * velocity_derivative + 0.7f * filtered_derivative; 
float derivative = pid->Kd * (-filtered_derivative); 
// Calculate PID output 
pid->output = proportional + integral + derivative; 
// Output clamping with anti-windup 
if (pid->output > MAX_PID_OUTPUT) { 
pid->output = MAX_PID_OUTPUT; 
pid->integral -= error * dt; // Anti-windup 
} 
else if (pid->output < MIN_PID_OUTPUT) { 
pid->output = MIN_PID_OUTPUT; 
pid->integral -= error * dt; // Anti-windup 
} 
pid->prev_error = error; 
} 
/* Lọc nhiễu tốc độ */ 
float filter_velocity(float new_velocity) { 
velocity_history[history_index] = new_velocity; 
history_index = (history_index + 1) % FILTER_WINDOW_SIZE; 
float sum = 0; 
for(int i=0; i<FILTER_WINDOW_SIZE; i++) { 
sum += velocity_history[i]; 
} 
return sum / FILTER_WINDOW_SIZE; 
} 
/* Set motor speed with PWM */ 
void set_motor_speed(uint8_t duty_cycle) { 
// Giới hạn duty cycle trong khoảng 0-100 
if(duty_cycle > 100) duty_cycle = 100; 
if(duty_cycle < 0) duty_cycle = 0; 
// Set PWM pulse width 
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle * 10); // Scale to 
0-1000 (period is 999) 
// Set direction based on sign (though in this case we're only using positive speeds) 
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, duty_cycle > 0 ? GPIO_PIN_SET : 
GPIO_PIN_RESET); 
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); 
current_duty_cycle = duty_cycle; 
} 
/* Update velocity measurement */ 
void update_velocity_measurement(void) { 
encoder = __HAL_TIM_GET_COUNTER(&htim2); 
time_now = HAL_GetTick(); 
uint32_t delta_time = time_now - time_prev; 
if (delta_time > 0) { 
int16_t delta_encoder = encoder - encoder_prev; 
float raw_velocity = ((float)delta_encoder / PULSES_PER_REV) / (delta_time / 1000.0f) * 
(PI * WHEEL_DIAMETER); 
// Lọc nhiễu tốc độ 
velocity = filter_velocity(raw_velocity); 
encoder_prev = encoder; 
time_prev = time_now; 
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
MX_TIM2_Init(); 
MX_I2C1_Init(); 
MX_TIM1_Init(); 
/* USER CODE BEGIN 2 */ 
HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); 
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
my_lcd.hi2c = &hi2c1; 
my_lcd.address = 0x27 << 1;  // địa chỉ LCD (nhân 2 vì HAL dùng 8-bit addr) 
lcd_init(&my_lcd); 
lcd_clear(&my_lcd); 
// Set initial motor direction 
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);   // IN1 = 1 
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // IN2 = 0 
/* Initialize PID controller */ 
PID_Init(&pid, 25.0f, 10.0f, 2.0f, target_velocity); // Kp, Ki, Kd, setpoint 
uint32_t last_pid_update = HAL_GetTick(); 
uint32_t last_display_update = HAL_GetTick(); 
/* USER CODE END 2 */ 
/* Infinite loop */ 
/* USER CODE BEGIN WHILE */ 
while (1) 
{ 
/* Update velocity measurement */ 
update_velocity_measurement(); 
/* Update PID and set motor speed */ 
if (HAL_GetTick() - last_pid_update >= PID_UPDATE_INTERVAL) { 
float dt = PID_UPDATE_INTERVAL / 1000.0f; 
PID_Update(&pid, velocity, dt); 
// Set motor speed based on PID output 
set_motor_speed((uint8_t)pid.output); 
last_pid_update = HAL_GetTick(); 
} 
/* Update display */ 
if (HAL_GetTick() - last_display_update >= 200) { 
lcd_gotoxy(&my_lcd, 0, 0); 
snprintf(lcd_buffer, sizeof(lcd_buffer), "Set: %.2f m/s ", pid.setpoint); 
lcd_puts(&my_lcd, lcd_buffer); 
lcd_gotoxy(&my_lcd, 0, 1); 
sizeof(lcd_buffer), 
"Act: %.2f m/s %2d%%", velocity, 
snprintf(lcd_buffer, 
current_duty_cycle); 
lcd_puts(&my_lcd, lcd_buffer); 
last_display_update = HAL_GetTick(); 
} 
HAL_Delay(10); 
/* USER CODE END WHILE */ 
/* USER CODE BEGIN 3 */ 
} 
/* USER CODE END 3 */ 
} 
// ... (Rest of the code remains the same, including all the hardware initialization functions) 
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
} 
Error_Handler(); 
/** Initializes the CPU, AHB and APB buses clocks 
*/ 
RCC_ClkInitStruct.ClockType = 
RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK 
|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; 
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; 
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; 
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; 
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; 
if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) 
{ 
} 
} 
Error_Handler(); 
/** 
* @brief I2C1 Initialization Function 
* @param None 
* @retval None 
*/ 
static void MX_I2C1_Init(void) 
{ 
/* USER CODE BEGIN I2C1_Init 0 */ 
/* USER CODE END I2C1_Init 0 */ 
/* USER CODE BEGIN I2C1_Init 1 */ 
/* USER CODE END I2C1_Init 1 */ 
hi2c1.Instance = I2C1; 
hi2c1.Init.ClockSpeed = 100000; 
hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2; 
hi2c1.Init.OwnAddress1 = 0; 
hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; 
hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; 
hi2c1.Init.OwnAddress2 = 0; 
hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; 
hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; 
if (HAL_I2C_Init(&hi2c1) != HAL_OK) 
{ 
} 
Error_Handler(); 
/* USER CODE BEGIN I2C1_Init 2 */ 
/* USER CODE END I2C1_Init 2 */ 
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
TIM_OC_InitTypeDef sConfigOC = {0}; 
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0}; 
/* USER CODE BEGIN TIM1_Init 1 */ 
/* USER CODE END TIM1_Init 1 */ 
htim1.Instance = TIM1; 
htim1.Init.Prescaler = 71; 
htim1.Init.CounterMode = TIM_COUNTERMODE_UP; 
htim1.Init.Period = 999; 
htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; 
htim1.Init.RepetitionCounter = 0; 
htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; 
if (HAL_TIM_Base_Init(&htim1) != HAL_OK) 
{ 
} 
Error_Handler(); 
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; 
if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) 
{ 
} 
Error_Handler(); 
if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) 
{ 
} 
Error_Handler(); 
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; 
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; 
if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) 
{ 
} 
Error_Handler(); 
sConfigOC.OCMode = TIM_OCMODE_PWM1; 
sConfigOC.Pulse = 0; 
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; 
sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; 
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; 
sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET; 
sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; 
if 
(HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != 
HAL_OK) 
{ 
Error_Handler(); 
} 
sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE; 
sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE; 
sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF; 
sBreakDeadTimeConfig.DeadTime = 0; 
sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE; 
sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH; 
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
htim2.Init.Prescaler = 0; 
htim2.Init.CounterMode = TIM_COUNTERMODE_UP; 
htim2.Init.Period = 65535; 
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
} 
Error_Handler(); 
/* USER CODE BEGIN TIM2_Init 2 */ 
/* USER CODE END TIM2_Init 2 */ 
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
__HAL_RCC_GPIOD_CLK_ENABLE(); 
__HAL_RCC_GPIOA_CLK_ENABLE(); 
__HAL_RCC_GPIOB_CLK_ENABLE(); 
/*Configure GPIO pin Output Level */ 
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET); 
/*Configure GPIO pins : PA9 PA10 */ 
GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10; 
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
GPIO_InitStruct.Pull = GPIO_NOPULL; 
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; 
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
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
