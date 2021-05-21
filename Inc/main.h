/* USER CODE BEGIN Header */
/****************************************************************************
* Title                 :   DFM - Electrical fence firmware test
* ProductLink           :   https://docs.google.com/spreadsheets/d/163NVGYAAz6Q9rcFRIo8FAkWSXMXqbl9w7QIAKOVKirg/edit#gid=984419045
* Filename              :   main.h
* Author                :   ItachiThuan
* Origin Date           :   May 21, 2021
* Version               :   1.0.0
* Target                :   STM32WB55 with STM32CubeIDE
* Notes                 :
*****************************************************************************/
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
* Preprocessor
*******************************************************************************/
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "stdbool.h"


/******************************************************************************
* Configuration Constants
*******************************************************************************/
/* USER CODE BEGIN Private defines */
#define RX_DMABUF_LEN 100
#define RX_BUF_LEN 100
#define WK_ACL_Pin GPIO_PIN_0
#define WK_ACL_GPIO_Port GPIOA
#define WK_ACL_EXTI_IRQn EXTI0_IRQn
#define EEPROM_EN_Pin GPIO_PIN_1
#define EEPROM_EN_GPIO_Port GPIOA
#define GPS_EN_Pin GPIO_PIN_4
#define GPS_EN_GPIO_Port GPIOA
#define EN_BATT_Pin GPIO_PIN_5
#define EN_BATT_GPIO_Port GPIOA
#define ADC_BATT_Pin GPIO_PIN_6
#define ADC_BATT_GPIO_Port GPIOA
#define D1_Pin GPIO_PIN_7
#define D1_GPIO_Port GPIOA
#define D2_Pin GPIO_PIN_8
#define D2_GPIO_Port GPIOA
#define SW_DIS_Pin GPIO_PIN_9
#define SW_DIS_GPIO_Port GPIOA
#define SW_DIS_EXTI_IRQn EXTI9_5_IRQn
#define DISTANCE_EN_Pin GPIO_PIN_0
#define DISTANCE_EN_GPIO_Port GPIOB
#define RAK_EN_Pin GPIO_PIN_15
#define RAK_EN_GPIO_Port GPIOA
#define PWM_CABLE_Pin GPIO_PIN_3
#define PWM_CABLE_GPIO_Port GPIOB
#define PWM_CABLE_EXTI_IRQn EXTI3_IRQn
#define TRIGGER_CABLE_Pin GPIO_PIN_4
#define TRIGGER_CABLE_GPIO_Port GPIOB
#define SHUTD_Pin GPIO_PIN_5
#define SHUTD_GPIO_Port GPIOB

#define RELAY_EN_Pin GPIO_PIN_2
#define RELAY_EN_Port GPIOB

#define LED_G_Pin GPIO_PIN_1
#define LED_G_Port GPIOB

#define LED_R_Pin GPIO_PIN_3
#define LED_R_Port GPIOB

#define BUZZER_Pin GPIO_PIN_10
#define BUZZER_Port GPIOA


#define PROBE1				GPIO_PIN_11
#define PROBE2				GPIO_PIN_12
#define PROBE_PORT			GPIOA
/* USER CODE END Private defines */


/******************************************************************************
* Macro
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/
/* USER CODE BEGIN EFP */
void UART_GetDataDMA(uint8_t* pui8buffer);
void Error_Handler(void);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
