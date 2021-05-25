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

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "app_conf.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
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
void UART_GetDataDMA(uint8_t* pui8buffer);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RX_DMABUF_LEN 100
#define RX_BUF_LEN 100
#define WK_ACL_Pin GPIO_PIN_0
#define WK_ACL_GPIO_Port GPIOA
#define WK_ACL_EXTI_IRQn EXTI0_IRQn
#define EEPROM_EN_Pin GPIO_PIN_1
#define EEPROM_EN_GPIO_Port GPIOA
#define ADC_POSITIVE_Pin GPIO_PIN_4
#define ADC_POSITIVE_GPIO_Port GPIOA
#define EN_BATT_Pin GPIO_PIN_5
#define EN_BATT_GPIO_Port GPIOA
#define ADC_BATT_Pin GPIO_PIN_6
#define ADC_BATT_GPIO_Port GPIOA
#define ADC_12V_Pin GPIO_PIN_7
#define ADC_12V_GPIO_Port GPIOA
#define ADC_NEGATIVE_Pin GPIO_PIN_8
#define ADC_NEGATIVE_GPIO_Port GPIOA
#define SW_DIS_Pin GPIO_PIN_9
#define SW_DIS_GPIO_Port GPIOA
#define SW_DIS_EXTI_IRQn EXTI9_5_IRQn
#define RELAY_EN_Pin GPIO_PIN_2
#define RELAY_EN_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_1
#define LED_G_GPIO_Port GPIOB
#define LED_TEST_Pin GPIO_PIN_4
#define LED_TEST_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_10
#define BUZZER_GPIO_Port GPIOA
#define RAK_EN_Pin GPIO_PIN_15
#define RAK_EN_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_3
#define LED_R_GPIO_Port GPIOB
#define TRIGGER_CABLE_Pin GPIO_PIN_4
#define TRIGGER_CABLE_GPIO_Port GPIOB
#define SHUTD_Pin GPIO_PIN_5
#define SHUTD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define USED								1
#define NOT_USED							0
#define FW_ELECFENCE

#define ENDLESS_LOOP_ACL					NOT_USED
#define ENDLESS_LOOP_DYP					NOT_USED
#define ENDLESS_BATT_MEASURING				NOT_USED
#define ENDLESS_ADC_MEASURING				NOT_USED
#define DEBUG_LORA_AT_UART					NOT_USED

#define FW_TEST								USED
#define EEPROM_TEST 						USED
#define	ACL_TEST							USED
#define GPS_TEST							NOT_USED

#ifdef FW_ELECFENCE
#define ADC_ELECFENCE_TEST					USED
#else
#define ADC_TEST							USED
#endif  /* End of ifdef FW_ELECFENCE */

#define LORA_TEST							NOT_USED
#define CHANGE_DEVEUI						USED
#define BLE_TEST							NOT_USED

#define TEST_DOWNLINK						USED
#define	DISABLE_ACL_IRQ						NOT_USED


#define DEBUG_CONSOLE						USED
#define DEBUG_UART							NOT_USED

#define PROBE1				GPIO_PIN_11
#define PROBE2				GPIO_PIN_12
#define PROBE_PORT			GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
