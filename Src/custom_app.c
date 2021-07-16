/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : custom_app.c
 * Description        : Custom Example Application (Server)
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
	/* ELECTRICFENCE */
	uint8_t               Hv_Notification_Status;
	uint8_t               Vbat_Notification_Status;
	/* USER CODE BEGIN CUSTOM_APP_Context_t */

	/* USER CODE END CUSTOM_APP_Context_t */

	uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

PLACE_IN_SECTION("BLE_APP_CONTEXT") static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

/* USER CODE BEGIN PV */
uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];
uint8_t SecureReadData;


ELFence_Batt_Data_t Batt_Data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* ELECTRICFENCE */
static void Custom_Hv_Update_Char(void);
static void Custom_Hv_Send_Notification(void);
static void Custom_Vbat_Update_Char(void);
//static void Custom_Vbat_Send_Notification(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/

/********************************************************* APP layer Handle GATT Events *********************************************************/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
	/* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

	/* USER CODE END CUSTOM_STM_App_Notification_1 */
	switch(pNotification->Custom_Evt_Opcode)
	{
		/* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */
		/* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

		/* ELECTRICFENCE */
		case CUSTOM_STM_HV_READ_EVT:
			/* USER CODE BEGIN CUSTOM_STM_HV_READ_EVT */
			/* USER CODE END CUSTOM_STM_HV_READ_EVT */
		break;

		case CUSTOM_STM_HV_NOTIFY_ENABLED_EVT:
			/* USER CODE BEGIN CUSTOM_STM_HV_NOTIFY_ENABLED_EVT */
			APP_DBG_MSG("\r\n\r** HIGH VOLTAGE NOTIFY ENABLED \n");

	//		Custom
			/* USER CODE END CUSTOM_STM_HV_NOTIFY_ENABLED_EVT */
		break;

		case CUSTOM_STM_HV_NOTIFY_DISABLED_EVT:
			/* USER CODE BEGIN CUSTOM_STM_HV_NOTIFY_DISABLED_EVT */
			APP_DBG_MSG("\r\n\r** HIGH VOLTAGE NOTIFY DISABLED \n");
			/* USER CODE END CUSTOM_STM_HV_NOTIFY_DISABLED_EVT */
		break;

		case CUSTOM_STM_VBAT_READ_EVT:
			/* USER CODE BEGIN CUSTOM_STM_VBAT_READ_EVT */

			/* USER CODE END CUSTOM_STM_VBAT_READ_EVT */
		break;

		case CUSTOM_STM_VBAT_NOTIFY_ENABLED_EVT:
			/* USER CODE BEGIN CUSTOM_STM_VBAT_NOTIFY_ENABLED_EVT */
			Batt_Data.Is_NotifyEn_B = true;
			APP_DBG_MSG("\r\n\r** BATTERY NOTIFY ENABLED \n");
			/* USER CODE END CUSTOM_STM_VBAT_NOTIFY_ENABLED_EVT */
		break;

		case CUSTOM_STM_VBAT_NOTIFY_DISABLED_EVT:
			/* USER CODE BEGIN CUSTOM_STM_VBAT_NOTIFY_DISABLED_EVT */
			Batt_Data.Is_NotifyEn_B = false;
			APP_DBG_MSG("\r\n\r** BATTERY NOTIFY DISABLED \n");
			/* USER CODE END CUSTOM_STM_VBAT_NOTIFY_DISABLED_EVT */
		break;

		default:
			/* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

			/* USER CODE END CUSTOM_STM_App_Notification_default */
		break;
	}
	/* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

	/* USER CODE END CUSTOM_STM_App_Notification_2 */
	return;
}


/********************************************************* App layer - Handle GAP Events *********************************************************/
void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
	/* USER CODE BEGIN CUSTOM_APP_Notification_1 */

	/* USER CODE END CUSTOM_APP_Notification_1 */

	switch(pNotification->Custom_Evt_Opcode)
	{
	/* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

	/* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
	case CUSTOM_CONN_HANDLE_EVT :
		/* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		APP_DBG_MSG("\r\n\r** COMPLETE CONNECTED EVENT WITH CLIENT \n");
		/* USER CODE END CUSTOM_CONN_HANDLE_EVT */
		break;

	case CUSTOM_DISCON_HANDLE_EVT :
		/* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
		APP_DBG_MSG("\r\n\r** DISCONNECTION EVENT WITH CLIENT \n");
		for (uint8_t count = 0; count < 10; ++count)	/* Blink then shut down Buzzer */
		{
			HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
			HAL_Delay(100);
		}
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		/* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
		break;

	default:
		/* USER CODE BEGIN CUSTOM_APP_Notification_default */

		/* USER CODE END CUSTOM_APP_Notification_default */
		break;
	}

	/* USER CODE BEGIN CUSTOM_APP_Notification_2 */

	/* USER CODE END CUSTOM_APP_Notification_2 */

	return;
}

void Custom_APP_Init(void)
{
	/* USER CODE BEGIN CUSTOM_APP_Init */

	/* USER CODE END CUSTOM_APP_Init */
	return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* ELECTRICFENCE */
void Custom_Hv_Update_Char(void) /* Property Read */
{
	Custom_STM_App_Update_Char(CUSTOM_STM_HV, (uint8_t *)UpdateCharData);
	/* USER CODE BEGIN Hv_UC*/

	/* USER CODE END Hv_UC*/
	return;
}



void Custom_Hv_Send_Notification(void) /* Property Notification */
{
	if(Custom_App_Context.Hv_Notification_Status)
	{
		Custom_STM_App_Update_Char(CUSTOM_STM_HV, (uint8_t *)NotifyCharData);
		/* USER CODE BEGIN Hv_NS*/

		/* USER CODE END Hv_NS*/
	}
	else
	{
		APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
	}
	return;
}

void Custom_Vbat_Update_Char(void) /* Property Read */
{
	Custom_STM_App_Update_Char(CUSTOM_STM_VBAT, (uint8_t *)UpdateCharData);
	/* USER CODE BEGIN Vbat_UC*/

	/* USER CODE END Vbat_UC*/
	return;
}

void Custom_Vbat_Send_Notification(void) /* Property Notification */
 {
  if(Custom_App_Context.Vbat_Notification_Status)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_VBAT, (uint8_t *)NotifyCharData);
    /* USER CODE BEGIN Vbat_NS*/

    /* USER CODE END Vbat_NS*/
  }
  else
  {
    APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
  }
  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
