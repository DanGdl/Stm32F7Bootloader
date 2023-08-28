/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_B6_Pin GPIO_PIN_5
#define LCD_B6_GPIO_Port GPIOK
#define LCD_B5_Pin GPIO_PIN_4
#define LCD_B5_GPIO_Port GPIOK
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define LCD_B3_Pin GPIO_PIN_15
#define LCD_B3_GPIO_Port GPIOJ
#define FDCAN1_STBY_Pin GPIO_PIN_3
#define FDCAN1_STBY_GPIO_Port GPIOD
#define PDM1_CLK_Pin GPIO_PIN_2
#define PDM1_CLK_GPIO_Port GPIOE
#define LCD_B7_Pin GPIO_PIN_6
#define LCD_B7_GPIO_Port GPIOK
#define LCD_B4_Pin GPIO_PIN_3
#define LCD_B4_GPIO_Port GPIOK
#define RMII_TXD1_Pin GPIO_PIN_12
#define RMII_TXD1_GPIO_Port GPIOG
#define SAI1_SCKA_Pin GPIO_PIN_5
#define SAI1_SCKA_GPIO_Port GPIOE
#define SAI1_FSA_Pin GPIO_PIN_4
#define SAI1_FSA_GPIO_Port GPIOE
#define SAI1_SDB_Pin GPIO_PIN_3
#define SAI1_SDB_GPIO_Port GPIOE
#define LCD_DE_Pin GPIO_PIN_7
#define LCD_DE_GPIO_Port GPIOK
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define LCD_B2_Pin GPIO_PIN_14
#define LCD_B2_GPIO_Port GPIOJ
#define LCD_B0_Pin GPIO_PIN_12
#define LCD_B0_GPIO_Port GPIOJ
#define MFX_IRQOUT_Pin GPIO_PIN_8
#define MFX_IRQOUT_GPIO_Port GPIOI
#define SAI1_SDA_Pin GPIO_PIN_6
#define SAI1_SDA_GPIO_Port GPIOE
#define LCD_B1_Pin GPIO_PIN_13
#define LCD_B1_GPIO_Port GPIOJ
#define MCO_Pin GPIO_PIN_8
#define MCO_GPIO_Port GPIOA
#define SAI1_MCLKA_Pin GPIO_PIN_7
#define SAI1_MCLKA_GPIO_Port GPIOG
#define LCD_HSYNC_Pin GPIO_PIN_12
#define LCD_HSYNC_GPIO_Port GPIOI
#define LCD_VSYNC_Pin GPIO_PIN_13
#define LCD_VSYNC_GPIO_Port GPIOI
#define LCD_CLK_Pin GPIO_PIN_14
#define LCD_CLK_GPIO_Port GPIOI
#define LCD_G7_Pin GPIO_PIN_2
#define LCD_G7_GPIO_Port GPIOK
#define LCD_G5_Pin GPIO_PIN_0
#define LCD_G5_GPIO_Port GPIOK
#define LCD_G6_Pin GPIO_PIN_1
#define LCD_G6_GPIO_Port GPIOK
#define LCD_G4_Pin GPIO_PIN_11
#define LCD_G4_GPIO_Port GPIOJ
#define LED1_RGB_Pin GPIO_PIN_10
#define LED1_RGB_GPIO_Port GPIOF
#define LCd_G3_Pin GPIO_PIN_10
#define LCd_G3_GPIO_Port GPIOJ
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define LCD_G2_Pin GPIO_PIN_9
#define LCD_G2_GPIO_Port GPIOJ
#define ETH_MDIO_Pin GPIO_PIN_2
#define ETH_MDIO_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define LCD_R1_Pin GPIO_PIN_0
#define LCD_R1_GPIO_Port GPIOJ
#define LCD_G1_Pin GPIO_PIN_8
#define LCD_G1_GPIO_Port GPIOJ
#define LCD_G0_Pin GPIO_PIN_7
#define LCD_G0_GPIO_Port GPIOJ
#define LCD_R7_Pin GPIO_PIN_6
#define LCD_R7_GPIO_Port GPIOJ
#define LCD_R0_Pin GPIO_PIN_15
#define LCD_R0_GPIO_Port GPIOI
#define LCD_R2_Pin GPIO_PIN_1
#define LCD_R2_GPIO_Port GPIOJ
#define LCD_BL_CTRL_Pin GPIO_PIN_6
#define LCD_BL_CTRL_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define LCD_R6_Pin GPIO_PIN_5
#define LCD_R6_GPIO_Port GPIOJ
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define LCD_R3_Pin GPIO_PIN_2
#define LCD_R3_GPIO_Port GPIOJ
#define RS_232RX_Pin GPIO_PIN_15
#define RS_232RX_GPIO_Port GPIOB
#define LED3_RGB_Pin GPIO_PIN_4
#define LED3_RGB_GPIO_Port GPIOA
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LCD_R4_Pin GPIO_PIN_3
#define LCD_R4_GPIO_Port GPIOJ
#define LCD_R5_Pin GPIO_PIN_4
#define LCD_R5_GPIO_Port GPIOJ
#define RS232_TX_Pin GPIO_PIN_14
#define RS232_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
