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
#include "app_entry.h"
#include "app_common.h"

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
#define EEPROM_EN_Pin GPIO_PIN_1
#define EEPROM_EN_GPIO_Port GPIOA
#define GPS_EN_Pin GPIO_PIN_4
#define GPS_EN_GPIO_Port GPIOA
#define ADC_BATT_Pin GPIO_PIN_6
#define ADC_BATT_GPIO_Port GPIOA
#define SW_DIS_Pin GPIO_PIN_9
#define SW_DIS_GPIO_Port GPIOA
#define SW_DIS_EXTI_IRQn EXTI9_5_IRQn
#define RAK_EN_Pin GPIO_PIN_2
#define RAK_EN_GPIO_Port GPIOB
#define RELAY_EN_Pin GPIO_PIN_11
#define RELAY_EN_GPIO_Port GPIOA
#define WAKEUP_Pin GPIO_PIN_0
#define WAKEUP_GPIO_Port GPIOB
#define WAKEUP_EXTI_IRQn EXTI0_IRQn
#define VREF_EN_Pin GPIO_PIN_1
#define VREF_EN_GPIO_Port GPIOB
#define SHUTD_EN_Pin GPIO_PIN_4
#define SHUTD_EN_GPIO_Port GPIOE
#define EXT_IO_EN_Pin GPIO_PIN_10
#define EXT_IO_EN_GPIO_Port GPIOA
#define OPA_SW_Pin GPIO_PIN_12
#define OPA_SW_GPIO_Port GPIOA
#define HV_POS_CHANNEL_Pin GPIO_PIN_0
#define HV_POS_CHANNEL_GPIO_Port GPIOA
#define HV_NEG_CHANNEL_Pin GPIO_PIN_5
#define HV_NEG_CHANNEL_GPIO_Port GPIOA
#define V12V_CHANNEL_Pin GPIO_PIN_7
#define V12V_CHANNEL_GPIO_Port GPIOA
#define NTC_CHANNEL_Pin GPIO_PIN_8
#define NTC_CHANNEL_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define USED								1
#define NOT_USED							0
#define FW_ELECFENCE

#define ADC_ELECFENCE_TEST					NOT_USED
#define PLOT_HV_ADC							NOT_USED
#define HV_MAX_OF_BOTH_ADCS					USED	/* Used DMA to find max ADC value of both channel */
#define HV_MAX_EACH_ADC_CHANNEL				NOT_USED	/* Used interrupt to find max ADC value of each channel */


#define ENDLESS_LOOP_ACL					NOT_USED
#define ENDLESS_LOOP_DYP					NOT_USED
#define ENDLESS_BATT_MEASURING				NOT_USED
#define ENDLESS_ADC_HV_MEASURING			NOT_USED
#define ENDLESS_HV_MEASURING				NOT_USED
#define DEBUG_LORA_AT_UART					NOT_USED


#define FW_TEST								USED
#define HW_SYSTEST							USED

#define EEPROM_TEST 						NOT_USED
#define	ACL_TEST							USED
#define GPS_TEST							USED

#if ADC_ELECFENCE_TEST
#define ADC_TEST							NOT_USED
#else
#define ADC_TEST							USED
#endif  /* End of ADC_ELECFENCE_TEST */

#define LORA_TEST							NOT_USED
#define TEST_SLEEPLORA						NOT_USED
#if LORA_TEST
#define LORAWAN_TEST						NOT_USED
#define CHANGE_DEVEUI						USED
#define TEST_DOWNLINK						USED
#endif  /* End of LORA_TEST */

#define BLE_TEST							USED
#define	DISABLE_ACL_IRQ						USED


#define DEBUG_CONSOLE						USED
#define DEBUG_UART							USED
#if (DEBUG_CONSOLE) && (!DEBUG_UART)
#define DEBUG_ITM							USED
#define DEBUG_UART							NOT_USED
#endif

#define DEV_SLEEP							USED
#define DEBUG_LPOWER						NOT_USED

#define PROBE1								GPIO_PIN_11
#define PROBE2								GPIO_PIN_12
#define PROBE_PORT							GPIOA

#define HV_POS_CHANNEL						ADC_CHANNEL_5
#define HV_NEG_CHANNEL						ADC_CHANNEL_10
#define BATT_ADC_CHANNEL					ADC_CHANNEL_11
#define V12V_ADC_CHANNEL					ADC_CHANNEL_12
#define NTC_TEMP_CHANNEL					ADC_CHANNEL_15

#define TOGGLE_PIN(PIN)						UNSED(PIN)		//TODO: Implement
#define OPA_SW_SET(STATE)					UNSED(STATE)	//TODO: Implement


#define PORTA_OUTPUT_PINS					(EEPROM_EN_Pin| GPS_EN_Pin| EXT_IO_EN_Pin| RELAY_EN_Pin| OPA_SW_Pin)
#define PORTB_OUTPUT_PINS					(VREF_EN_Pin| RAK_EN_Pin)



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
