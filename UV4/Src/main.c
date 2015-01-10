/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 06/01/2015 17:24:27
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "MPU6050.h"
#include <math.h>

#define start_angle                 0.1     // degree
#define limit_angle                 30    // degree
#define limit_speed                 100    // cm/s
#define limit_compensate_angle      15    // degree

#define beta                        0.1f     
#define ACCELEROMETER_SENSITIVITY   16384.0f  
#define GYROSCOPE_SENSITIVITY       0.007629510948f    // 1/131.07f  
#define M_PI                        3.14159265359f	    

#define clock_cnt                   100000.0f
#define dt                          0.005f
#define sampling                    200

#define inv_cm2pulse                0.006675883837f

#define data2voltage                0.00332992f
#define deg2rad                     0.0174532925f
#define rad2deg                     57.29577951f
#define gx_diff                     34    // x axis gyro diff
#define gy_diff                     41    // y axis gyro diff
#define gz_diff                     156   // z axis gyro diff

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

float debug;

float Battery_voltage = 12;   // assume full battery
float front_distance = 100;   // assume no obstacle
float rear_distance = 100;    // assume no obstacle
uint8_t sr_04_channel = 0;    // 0 = front, 1 = rear
uint8_t echo_status = 0;   // prevent tim17 over flow  


int8_t raw_data_uart[10] = {0};
int8_t ch1, ch2, ch3, ch4;
int8_t uart_watchdog = 0;

int16_t AccelGyro[6] = {0};                              
float q1=1, q2=0, q3=0, q4=0;
float L_R_ref_filted ;

// state//
float angle = 0;    
float angle_dot = 0;
float velocity = 0;    
float position = 0;  
float force = 0;  

// PID variable //

float posi_ref = 0;

float velo_ref = 0;
float L_R_ref = 0;

float error_posi = 0; 
float error_posi_dot = 0;

float error_angle = 0; 
float error_angle_dot = 0;

float error_velo = 0; 
float error_velo_dot = 0;

float kp_angle = 25;
float kd_angle = 0.56;

//float kp_velo = 1.8;
//float ki_velo = 0.05;
//float kd_velo = 1.1;
//float ki_velo_dyna = 0.2;

float kp_velo = 0.1;
float ki_velo = 0.001;
float kd_velo = 0;
float ki_velo_dyna = 7;

static float error_velo_sum;
static float error_velo_sum_dynamic;
static float error_velo_sum_output; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

void Initial_MPU6050(void);
void read_mpu6050(void);
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
void MPU6050_GetRawAccelGyro(int16_t* AccelGyro);
void Ahrs(void);
void A4988_driver_state(FunctionalState tmp);
void A4988_driver_output(float velocity_mL_tmp, float velocity_mR_tmp);
float Smooth_filter(float alfa, float new_data, float prev_data);
void SR_04_measuring(void);
void Mesuaring_batt(void);
void Sampling_isr(void);
void Print_BLE(void);
void Print_Debug(void);



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  
  A4988_driver_state(DISABLE);    // disable step motor driver
  
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim14);

  HAL_TIM_Base_Start(&htim17);   // start iuput capture for SR-04 module
  
  Initial_MPU6050();    // initial mpu6050 with DLPF 98 Hz
  
  A4988_driver_state(DISABLE);    // disable step motor driver

  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
  
  HAL_TIM_Base_Start_IT(&htim16);   // start controller sampling
  
  /* Infinite loop */
  while (1)
  {
    SR_04_measuring();    // mesuaring disrance 
    Mesuaring_batt();   // mesuaring battary voltage
    if (uart_watchdog != 0)
    {
      velo_ref = (float)ch2 * 0.4f;
      L_R_ref = (float)ch1 * 0.04f;
    }
    else
    {
      velo_ref = 0;
      L_R_ref = 0;
    }
    
    if (uart_watchdog > 0) uart_watchdog --;    
    HAL_UART_Receive_IT(&huart1, (uint8_t *)raw_data_uart, 10);   // receive data
    HAL_Delay(20);    // dely for 50Hz
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __SYSCFG_CLK_ENABLE();

}

/* ADC init function */
void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc.Init.Resolution = ADC_RESOLUTION12b;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = OVR_DATA_PRESERVED;
  HAL_ADC_Init(&hadc);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 239;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);
}

/* TIM14 init function */
void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 239;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim14);

}

/* TIM16 init function */
void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 479;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 499;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim16);

}

/* TIM17 init function */
void MX_TIM17_Init(void)
{

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 47;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 0xffff;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim17);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 9600;    // for ble
  huart1.Init.BaudRate = 115200;    // for debug
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

void Initial_MPU6050(void)
{
  HAL_Delay(100); // for stability
  //    Reset to defalt 
  MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, ENABLE);

  //	  SetClockSource(MPU6050_CLOCK_PLL_XGYRO)
  MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);	

  //    SetFullScaleGyroRange(MPU6050_GYRO_FS_250)
  MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);

  //    SetFullScaleAccelRange(MPU6050_ACCEL_FS_2)
  MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_ACCEL_FS_2);

  //    interupt(Enable)
  MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, ENABLE);

  //		SetDLPF(MPU6050_DLPF_BW_5)
  MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42);
 
  //    SetSleepModeStatus(DISABLE)
  MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, DISABLE);
  HAL_Delay(100); // for stability
}

void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp;
		HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    HAL_I2C_Mem_Write(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
}

void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t tmp;
		HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    HAL_I2C_Mem_Write(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
}

void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp;
		HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}

void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t tmp;
		HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    *data = tmp & (1 << bitNum);
}
void MPU6050_GetRawAccelGyro(int16_t* AccelGyro)
{
    uint8_t tmpBuffer[14];
	  HAL_I2C_Mem_Read(&hi2c1,MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,1,tmpBuffer,14,1);
    /* Get acceleration */
    for (int i = 0; i < 3; i++)
        AccelGyro[i] = ((int16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
    /* Get Angular rate */
    for (int i = 4; i < 7; i++)
        AccelGyro[i - 1] = ((int16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);

}

void Ahrs(void)
{
	// quaternion base process 

	float Norm;
	float ax = AccelGyro[0];
	float ay = AccelGyro[1];
	float az = AccelGyro[2];
	float gx =((AccelGyro[3]-gx_diff) * GYROSCOPE_SENSITIVITY ) * deg2rad;
	float gy =((AccelGyro[4]-gy_diff) * GYROSCOPE_SENSITIVITY ) * deg2rad;
	float gz =((AccelGyro[5]-gz_diff) * GYROSCOPE_SENSITIVITY ) * deg2rad;
	
	float q1_dot = 0.5 * (-q2 * gx - q3 * gy - q4 * gz);
	float q2_dot = 0.5 * ( q1 * gx + q3 * gz - q4 * gy);
	float q3_dot = 0.5 * ( q1 * gy - q2 * gz + q4 * gx);
	float q4_dot = 0.5 * ( q1 * gz + q2 * gy - q3 * gx);

	if(!((ax == 0) && (ay == 0) && (az == 0))) {
		// Normalise 
		Norm = sqrtf(ax * ax + ay * ay + az * az);
		ax /= Norm;
		ay /= Norm;
		az /= Norm;   

		float _2q1 = 2 * q1;
		float _2q2 = 2 * q2;
		float _2q3 = 2 * q3;
		float _2q4 = 2 * q4;
		float _4q1 = 4 * q1;
		float _4q2 = 4 * q2;
		float _4q3 = 4 * q3;
		float _8q2 = 8 * q2;
		float _8q3 = 8 * q3;
		float q1q1 = q1 * q1;
		float q2q2 = q2 * q2;
		float q3q3 = q3 * q3;
		float q4q4 = q4 * q4;
		// Gradient decent 
		float s1 = _4q1 * q3q3 + _2q4 * ax + _4q1 * q2q2 - _2q2 * ay;
		float s2 = _4q2 * q4q4 - _2q4 * ax + 4 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
		float s3 = 4 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
		float s4 = 4 * q2q2 * q4 - _2q2 * ax + 4 * q3q3 * q4 - _2q3 * ay;
		// Normalise 
		Norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
		s1 /= Norm;
		s2 /= Norm;
		s3 /= Norm;
		s4 /= Norm;
		// compensate acc
		q1_dot -= (beta * s1);
		q2_dot -= (beta * s2);
		q3_dot -= (beta * s3);
		q4_dot -= (beta * s4);
	}
  
  // angular velocity
  

  angle_dot = asinf(2*(q2_dot*q4_dot - q1_dot*q3_dot)) * rad2deg;   //pitch
  
	// Integrate 
	q1 += q1_dot * dt;
	q2 += q2_dot * dt;
	q3 += q3_dot * dt;
	q4 += q4_dot * dt;

	// Normalise
	Norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4 );
	q1 /= Norm;
	q2 /= Norm;
	q3 /= Norm;
	q4 /= Norm;
  
	// convert to euler
//  float x=  2*(0.5 - q2*q2 - q3*q3);
//	y_roll =  2*(q3*q4 + q1*q2);
//  q_roll = atan2f(y_roll,x) * rad2deg;  //roll

  angle = asinf(2*(q2*q4 - q1*q3)) * rad2deg;   //pitch
     
}
void A4988_driver_state(FunctionalState tmp)
{
  if (tmp)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  }
}
float Smooth_filter(float alfa, float new_data, float prev_data)
{
  float output = prev_data + (alfa * (new_data - prev_data));
  return output;
}

void SR_04_measuring(void)
{
  sr_04_channel = 1 - sr_04_channel;    // switch direction 0front & 1rear 
  
  echo_status = 0;    // Reset echo input capture ready
  __disable_irq();    // disable interrupt 10 uS
  if (sr_04_channel)
    // rear mesauring     
    {
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
    }
    else
    // front mesauring        
    {
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
    }
    
    uint16_t count_delay = 76;
    while (count_delay > 0) count_delay--;    // dalay 10uS
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
    __enable_irq();   // resume interrupt  
}

void Mesuaring_batt(void)
{
  HAL_ADC_Start(&hadc);
  float tmp_adc = (float)HAL_ADC_GetValue(&hadc) * data2voltage;
  Battery_voltage = Smooth_filter(0.3f, (float)tmp_adc, Battery_voltage);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (echo_status)
    {
      float tmp_distance = TIM17->CNT;
      
      // convert to centimeter
      tmp_distance = tmp_distance / 58.0f;     
      if (sr_04_channel)
      {
//        rear_distance = tmp_distance;
        rear_distance = Smooth_filter(0.1f, tmp_distance, rear_distance);
      }
      else
      {
//        front_distance = tmp_distance;
        front_distance = Smooth_filter(0.1f, tmp_distance, front_distance);
      }
    }
    else
    {
      TIM17->CNT = 0;
      echo_status = 1;
    }
}


void A4988_driver_output(float velocity_mL_tmp, float velocity_mR_tmp)
{
  // write direction pin left motor
  if (velocity_mL_tmp > 0)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    velocity_mL_tmp = -velocity_mL_tmp;
  }
  
  // write direction pin right motor
  if (velocity_mR_tmp > 0)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    velocity_mR_tmp = -velocity_mR_tmp;
  }
  
  // calculate period * 149.7927 pulse / 1 cm *
  
  float period_L = (float)clock_cnt * (float)inv_cm2pulse / (float)velocity_mL_tmp;
  float period_R = (float)clock_cnt * (float)inv_cm2pulse / (float)velocity_mR_tmp;
  
  if (period_L > 65535) period_L = 65535;
  if (period_R > 65535) period_R = 65535; 
  
  // Motor L
    TIM14 -> ARR = period_L;
    if ( TIM14 -> CNT > TIM14 -> ARR) TIM14 -> CNT = period_L * 0.9f;   //reset counter register tim14
  
  // Motor R
    TIM3 -> ARR = period_R;
    if ( TIM3 -> CNT > TIM3 -> ARR) TIM3 -> CNT = period_L * 0.9f;    //reset counter register tim3

}

void Sampling_isr(void)
{
  static int8_t is_1st_start;
  static int8_t count_velo;
  static float angle_from_velo; 
  static float velo_from_force;
  static float velocity_mL;
  static float velocity_mR;
  static int32_t count_un_integrate = 0;
  
  // call at 200 Hz
  MPU6050_GetRawAccelGyro(AccelGyro);
  Ahrs();

  if ((angle < start_angle) && (angle > -start_angle))
  {
    if (!is_1st_start)
    {
      is_1st_start = 1;
      error_velo_sum = 0;
      velo_from_force = 0;
      A4988_driver_state(ENABLE);
    }
  }
  if ((angle > limit_angle) || (angle < -limit_angle))
  {
    is_1st_start = 0;
    A4988_driver_state(DISABLE);
  }

  /*888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888*/
  
  ////////////////////////////////////
  // PI controller for Velocity at 50 Hz
  count_velo --;
  if (count_velo <= 0)
  {
    //88888888888888888888 Velocity 888888888888888888888//

    float error_velo_prev = error_velo;
    error_velo = velo_ref - velo_from_force; 
    error_velo_dot = (error_velo - error_velo_prev) * 50.0f ;    // differential of velocity == force
    

      if (velo_ref == 0)
      {
        error_velo_sum += error_velo * 0.02f;   // 4 * dt = 4 * 0.005 = 0.02
        error_velo_sum_dynamic = 0;
      }
      else
      {
        error_velo_sum_dynamic += error_velo * 0.02f;
      }
    }
    
    if (count_un_integrate > 0) count_un_integrate --;
    
    
    error_velo_sum_output = error_velo_sum * ki_velo + error_velo_sum_dynamic * ki_velo_dyna; 
    
    if (error_velo_sum_output > limit_compensate_angle) error_velo_sum_output = limit_compensate_angle;    // prevent wildup
    if (error_velo_sum_output < -limit_compensate_angle) error_velo_sum_output = -limit_compensate_angle;    

    //888888888888888888888888888888888888888888888888888// 

    angle_from_velo = (error_velo_dot * kp_velo) + (error_velo_sum_output); 
   
    if (angle_from_velo > 10) angle_from_velo = 10;
    if (angle_from_velo < -10) angle_from_velo = -10;

  ////////////////////////////////////
  
  /*======================================================================================================*/
//  angle_from_velo = 0;  // test velocity control
  /*======================================================================================================*/ 
  
  if (Battery_voltage < 10) angle_from_velo = 0;    // battery protection
  if (front_distance  < 30 && rear_distance > 30 && uart_watchdog > 0) 
  {
//    angle_from_velo = -2;
//    count_un_integrate = 500;   // 800 == 8 sec
  }
  if (rear_distance  < 30 && front_distance > 30 && uart_watchdog > 0)
  {
//    angle_from_velo = 2;
//    count_un_integrate = 500;
  }
  
  ///////////////////////////////////
  // PD controller for angle at 200 Hz
  
  float error_angle_prev = error_angle ;

  error_angle = angle - angle_from_velo - 2.7f;    // 2.7f from unbalance mass
  error_angle_dot = (error_angle - error_angle_prev) ;
  
  force = ((error_angle * kp_angle * dt) + (error_angle_dot * kd_angle));
  
  velo_from_force += force;   // velocity == integreted of force
  
  ///////////////////////////////////
  
  if (velo_from_force > limit_speed) velo_from_force = limit_speed;
  if (velo_from_force < -limit_speed) velo_from_force = -limit_speed; 
  
  L_R_ref_filted = Smooth_filter(0.1, L_R_ref, L_R_ref_filted);
  
  if (velo_from_force >= 0) 
  {
    velocity_mL = velo_from_force - L_R_ref_filted;
    velocity_mR = velo_from_force + L_R_ref_filted;
  }
  else
  {
    velocity_mL = velo_from_force + L_R_ref_filted;
    velocity_mR = velo_from_force - L_R_ref_filted;
  }
  
//    velocity_mL = velo_from_force - L_R_ref_filted;
//    velocity_mR = velo_from_force + L_R_ref_filted;
  
  velocity = velo_from_force;
  position +=  velo_from_force * dt;
  
  A4988_driver_output(velocity_mL, velocity_mR);
  
  
//  Print_Debug();
  ////////////////////////////////////  
  /*======================================================================================================*/
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  int8_t i=0;
  while(raw_data_uart[i++] != 125 )
  {
    if (i>5)
    {
      return;
    }
  }
  
  ch1 = raw_data_uart[i];
  i++ ; 
  ch2 = raw_data_uart[i];   
  i++ ; 
  ch3 = raw_data_uart[i];  
  i++ ; 
  ch4 = raw_data_uart[i];
  
  uart_watchdog = 25;     // Reset watchdog timer
}
void Print_BLE(void)
{
  
}

void Print_Debug(void)
{
  uint8_t header[2] = {0x7e, 0x7e};
  uint8_t terminator[2] = {0xe7, 0xe7};
  HAL_UART_Transmit(&huart1, header, 2, 1);    // sent header
  HAL_UART_Transmit(&huart1, (uint8_t *)&angle_dot, 4, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)&angle, 4, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)&force, 4, 1);
  HAL_UART_Transmit(&huart1, terminator, 2, 1);    // sent header
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
