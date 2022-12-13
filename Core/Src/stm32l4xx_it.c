/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kamami_l496_mems.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
typedef enum
{
	acc_x,
	acc_y,
	acc_z
} Accelerometer_State_Type;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SEG_A  GPIO_PIN_0
#define SEG_B  GPIO_PIN_1
#define SEG_C  GPIO_PIN_2
#define SEG_D  GPIO_PIN_3
#define SEG_E  GPIO_PIN_4
#define SEG_F  GPIO_PIN_5
#define SEG_G GPIO_PIN_6
#define SEG_DP GPIO_PIN_9
#define ALL_SEGMENTS SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G | SEG_DP

#define DIG_1 GPIO_PIN_2
#define DIG_2 GPIO_PIN_3
#define DIG_3 GPIO_PIN_4
#define DIG_4 GPIO_PIN_5

#define MAX_NUMBER_OF_DIGIT 4U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
const uint8_t segments[] = {
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,//0
  SEG_B | SEG_C,// 1
	SEG_A | SEG_E | SEG_B | SEG_D | SEG_G,// 2
	SEG_A | SEG_B | SEG_G | SEG_C | SEG_D,// 3
	SEG_F | SEG_G | SEG_B | SEG_C,// 4
	SEG_A | SEG_F | SEG_G | SEG_C | SEG_D,// 5
  SEG_A | SEG_F | SEG_G | SEG_E | SEG_C | SEG_D,// 6
	SEG_A | SEG_B | SEG_C,// 7
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,// 8
	SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,// 9
  SEG_A | SEG_F | SEG_G | SEG_E | SEG_C,//10 - x
  SEG_F | SEG_G | SEG_B | SEG_C,//11 - y
  SEG_A | SEG_B | SEG_G | SEG_E | SEG_D//12 - z
};
bool is_button_pressed = false;
static Accelerometer_State_Type acc_state = acc_x;
struct mems_xyz_res acc_res_t = {0, 0, 0};
uint8_t currentDigitposition = 1U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void clear_lcd()
{
  HAL_GPIO_WritePin(GPIOG, ALL_SEGMENTS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, DIG_1 | DIG_2 | DIG_3 | DIG_4, GPIO_PIN_RESET);
}

void send_digit_to_LED(const float* value, const uint16_t DIG_number, const uint8_t position, bool isDot)
{
  uint8_t digit = 0U;
  if (value == NULL && position == 1)
  {
    if (acc_state == acc_x)
    {
      digit = 10;
    }
    else if (acc_state == acc_y)
    {
      digit = 11;
    }
    else if (acc_state == acc_z)
    {
      digit = 12;
    }
  }
  else if (position == 2)
  {
    digit = (uint8_t)(*value);
    if (digit > 9)
    {
      digit = 9;
    }
  }
  else if (position == 3)
  {
    digit = ((uint8_t)(10.0F * (*value)) % 10);
  }
  else if (position == 4)
  {
    digit = ((uint8_t)(100.0F * (*value)) % 10);
  }

  clear_lcd();
  if(isDot == true)
  {
    HAL_GPIO_WritePin(GPIOG, segments[digit] | SEG_DP, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOG, segments[digit], GPIO_PIN_SET);
  }
  HAL_GPIO_WritePin(GPIOB, DIG_number, GPIO_PIN_SET);
}

void update_led_display(const float frequency, struct mems_xyz_res* res)
{
  float res_f = 0.0F;
  static uint32_t counter = 0U;
  static uint32_t max_counter = 0U;
  max_counter = (uint32_t)((float)1000/frequency);

  if (acc_state == acc_x)
  {
    res_f = (float)(acc_res_t.x) * 2.0F / MEMS_ACC_MAXVAL;
  }
  else if (acc_state == acc_y)
  {
    res_f = (float)(acc_res_t.y) * 2.0F / MEMS_ACC_MAXVAL;
  }
  else if (acc_state == acc_z)
  {
    res_f = (float)(acc_res_t.z) * 2.0F / MEMS_ACC_MAXVAL;
  }

  counter++;
  if(counter >= max_counter)
  {
    if (currentDigitposition == 1)
    {
      send_digit_to_LED(NULL, DIG_1, 1, true);
    }
    else if (currentDigitposition == 2) 
    {
      send_digit_to_LED(&res_f, DIG_2, 2, true);
    }
    else if (currentDigitposition == 3)
    {
      send_digit_to_LED(&res_f, DIG_3, 3, false);
    }
    else if (currentDigitposition == 4)
    {
      send_digit_to_LED(&res_f, DIG_4, 4, false);
    }

    currentDigitposition++;
    if(currentDigitposition > MAX_NUMBER_OF_DIGIT)
    {
    currentDigitposition = 1U;
    }
    counter = 0;
  }
}

void set_accelerometer_state_type()
{
  if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_SET)
	{
		is_button_pressed = false;
	}

	//change accelerometer state to z by right button
	if(is_button_pressed == false && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET)
	{
    acc_state = acc_z;
    is_button_pressed = true;
	}
	//change accelerometer state to x by left button
	if(is_button_pressed == false && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET)
	{
		acc_state = acc_z;
    is_button_pressed = true;
	}
	//change accelerometer state to y by up button
	if(is_button_pressed == false && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
	{
    acc_state = acc_z;
		is_button_pressed = true;
	}
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  mems_acc_read_xyz(&acc_res_t);
  set_accelerometer_state_type(&acc_state);
  update_led_display(1000, &acc_res_t);
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
