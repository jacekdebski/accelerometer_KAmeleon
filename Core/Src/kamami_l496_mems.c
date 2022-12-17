// Functions for easy use of LSM303C MEMS accelerometer on Kamami KAMeLeon STM32L496 board.
// jk 11'2017
// The initialization code was generated in STM32CubeMX.

#include "kamami_l496_mems.h"
#include "stm32l4xx_hal.h"

// MEMS accelerometer I2C address
#define	KAMAMI_L496_MEMS_ACC_ADDR	0x1D

#define MEMS_ACC_AXIS_X				1
#define MEMS_ACC_AXIS_Y				2
#define MEMS_ACC_AXIS_Z				4
#define MEMS_ACC_AXIS_XYZ			8

#define MEMS_I2C3_SCL_Pin GPIO_PIN_7
#define MEMS_I2C3_SCL_GPIO_Port GPIOG
#define MEMS_I2C3_SDA_Pin GPIO_PIN_8
#define MEMS_I2C3_SDA_GPIO_Port GPIOG
#define MEMS_RDY_MAG_Pin GPIO_PIN_10
#define MEMS_RDY_MAG_GPIO_Port GPIOG
#define MEMS_INT_AXL_Pin GPIO_PIN_11
#define MEMS_INT_AXL_GPIO_Port GPIOG
#define MEMS_INT_MAG_Pin GPIO_PIN_12
#define MEMS_INT_MAG_GPIO_Port GPIOG

#define	MEMS_ACC_ID_REG		0xf
#define	MEMS_ACC_ID			0x41
#define	MEMS_ACC_CTRL1_REG	0x20
#define	MEMS_ACC_CTRL4_REG	0x23
#define	MEMS_ACC_X_REG		0x28
#define	MEMS_ACC_Y_REG		0x2a
#define	MEMS_ACC_Z_REG		0x2c

#define	MEMS_MAG_ID_REG		0xf
#define	MEMS_MAG_ID     	0x3d
#define	MEMS_MAG_CTRL1_REG	0x20
#define	MEMS_MAG_CTRL2_REG	0x21
#define	MEMS_MAG_CTRL3_REG	0x22
#define	MEMS_MAG_CTRL4_REG	0x23
#define	MEMS_MAG_X_REG		0x28
#define	MEMS_MAG_Y_REG		0x2a
#define	MEMS_MAG_Z_REG		0x2c

#define	I2C_MRW				0x80	// multiple read/write

I2C_HandleTypeDef hi2c3;

static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void LM75_Error_Handler(char *file, int line);

uint8_t mems_init(uint8_t acc_datarate, uint8_t acc_fullscale) {
	RCC_PeriphCLKInitTypeDef PeriphClkInit;
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
	PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}

	MX_GPIO_Init();
	MX_I2C3_Init();

	uint8_t acc_axis = 7; // all axis 
	uint8_t r[4];

	if (HAL_I2C_Mem_Read(&hi2c3, KAMAMI_L496_MEMS_ACC_ADDR << 1,
			MEMS_ACC_ID_REG, 1, r, 1, 1000) != HAL_OK)
		LM75_Error_Handler(__FILE__, __LINE__);
	if (r[0] != MEMS_ACC_ID)
		return 1;

	if (acc_axis == MEMS_ACC_AXIS_XYZ)
		acc_axis = MEMS_ACC_AXIS_X | MEMS_ACC_AXIS_Y | MEMS_ACC_AXIS_Z; // each axis is enabled individually
	if (acc_datarate == 0 || acc_datarate >= 7 || acc_axis > 7
			|| (acc_fullscale & 0xfc) != 0 || acc_fullscale == 1)
		return 1;

	r[0] = (acc_datarate << 4) | acc_axis;
	if (HAL_I2C_Mem_Write(&hi2c3, KAMAMI_L496_MEMS_ACC_ADDR << 1,
	MEMS_ACC_CTRL1_REG, 1, r, 1, 1000) != HAL_OK)
		LM75_Error_Handler(__FILE__, __LINE__);

	r[0] = (acc_fullscale << 4) | 4; // set fullscale, turn address autoincrement on
	if (HAL_I2C_Mem_Write(&hi2c3, KAMAMI_L496_MEMS_ACC_ADDR << 1,
	MEMS_ACC_CTRL4_REG, 1, r, 1, 1000) != HAL_OK)
		LM75_Error_Handler(__FILE__, __LINE__);

	return 0;
}

void mems_acc_read_xyz(struct mems_xyz_res *res) {
	if (HAL_I2C_Mem_Read(&hi2c3, KAMAMI_L496_MEMS_ACC_ADDR << 1,
	MEMS_ACC_X_REG | I2C_MRW, 1, (uint8_t*) res, 6, 1000) != HAL_OK)
		LM75_Error_Handler(__FILE__, __LINE__);
}

int16_t mems_acc_read_x(void) {
	int16_t res;
	if (HAL_I2C_Mem_Read(&hi2c3, KAMAMI_L496_MEMS_ACC_ADDR << 1,
	MEMS_ACC_X_REG | I2C_MRW, 1, (uint8_t*) &res, 2, 1000) != HAL_OK)
		LM75_Error_Handler(__FILE__, __LINE__);
	return res;
}

int16_t mems_acc_read_y(void) {
	int16_t res;
	if (HAL_I2C_Mem_Read(&hi2c3, KAMAMI_L496_MEMS_ACC_ADDR << 1,
	MEMS_ACC_Y_REG | I2C_MRW, 1, (uint8_t*) &res, 2, 1000) != HAL_OK)
		LM75_Error_Handler(__FILE__, __LINE__);
	return res;
}

int16_t mems_acc_read_z(void) {
	int16_t res;
	if (HAL_I2C_Mem_Read(&hi2c3, KAMAMI_L496_MEMS_ACC_ADDR << 1,
	MEMS_ACC_Z_REG | I2C_MRW, 1, (uint8_t*) &res, 2, 1000) != HAL_OK)
		LM75_Error_Handler(__FILE__, __LINE__);
	return res;
}

/* I2C3 init function */
static void MX_I2C3_Init(void) {

	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x10909CEC;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}

	/**Configure Analogue filter 
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/**Configure Digital filter 
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}

}

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOG_CLK_ENABLE();
	HAL_PWREx_EnableVddIO2();

	/*Configure GPIO pin : MEMS_RDY_MAG_Pin */
	GPIO_InitStruct.Pin = MEMS_RDY_MAG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MEMS_RDY_MAG_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MEMS_INT_AXL_Pin MEMS_INT_MAG_Pin */
	GPIO_InitStruct.Pin = MEMS_INT_AXL_Pin | MEMS_INT_MAG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void LM75_Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
//  while(1)
//  {
//  }
	/* USER CODE END Error_Handler_Debug */
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {

	GPIO_InitTypeDef GPIO_InitStruct;
	if (hi2c->Instance == I2C3) {
		/* USER CODE BEGIN I2C3_MspInit 0 */

		/* USER CODE END I2C3_MspInit 0 */

		/**I2C3 GPIO Configuration    
		 PG7     ------> I2C3_SCL
		 PG8     ------> I2C3_SDA 
		 */
		GPIO_InitStruct.Pin = MEMS_I2C3_SCL_Pin | MEMS_I2C3_SDA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
		HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C3_CLK_ENABLE();
		/* USER CODE BEGIN I2C3_MspInit 1 */

		/* USER CODE END I2C3_MspInit 1 */
	}

}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c) {

	if (hi2c->Instance == I2C3) {
		/* USER CODE BEGIN I2C3_MspDeInit 0 */

		/* USER CODE END I2C3_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_I2C3_CLK_DISABLE();

		/**I2C3 GPIO Configuration    
		 PG7     ------> I2C3_SCL
		 PG8     ------> I2C3_SDA 
		 */
		HAL_GPIO_DeInit(GPIOG, MEMS_I2C3_SCL_Pin | MEMS_I2C3_SDA_Pin);

		/* USER CODE BEGIN I2C3_MspDeInit 1 */

		/* USER CODE END I2C3_MspDeInit 1 */
	}

}
