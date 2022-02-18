/**
  ******************************************************************************
  * @file    custom_nfc04a1.c
  * @author  MMY Application Team
  * @version $Revision: 3351 $
  * @date    $Date: 2017-01-25 17:28:08 +0100 (Wed, 25 Jan 2017) $
  * @brief   This file provides nfc04a1 specific functions
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 STMicroelectronics</center></h2>
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

/** @addtogroup CUSTOM
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/** @defgroup X_NUCLEO_NFC04A1_Global_Variables
 * @{
 */
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "custom_nfc04a1.h"

__weak void BSP_GPO_Callback(void);

EXTI_HandleTypeDef GPO_EXTI={.Line=ST25DV_INT_PIN_GPO_EXTI_LINE};

/**
  * @brief  This function initialize the GPIO to manage the NFCTAG GPO pin
  * @param  None
  * @return Status
  */
int32_t CUSTOM_GPO_Init( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin   = ST25DV_INT_PIN_GPO_EXTI_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init( ST25DV_INT_PIN_GPO_EXTI_PORT, &GPIO_InitStruct );

  (void)HAL_EXTI_GetHandle(&GPO_EXTI, ST25DV_INT_PIN_GPO_EXTI_LINE);
  (void)HAL_EXTI_RegisterCallback(&GPO_EXTI,  HAL_EXTI_COMMON_CB_ID, BSP_GPO_Callback);

  /* Enable interruption */
  HAL_NVIC_SetPriority( ST25DV_INT_PIN_GPO_EXTI_IRQn, CUSTOM_NFCTAG_GPO_PRIORITY, 0 );
  HAL_NVIC_EnableIRQ( ST25DV_INT_PIN_GPO_EXTI_IRQn );

  return BSP_ERROR_NONE;

}

/**
  * @brief  DeInit GPO.
  * @param  None.
  * @return Status
  * @note GPO DeInit does not disable the GPIO clock nor disable the Mfx
  */
int32_t CUSTOM_GPO_DeInit( void )
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* DeInit the GPIO_GPO pin */
  gpio_init_structure.Pin = ST25DV_INT_PIN_GPO_EXTI_PIN;
  HAL_GPIO_DeInit( ST25DV_INT_PIN_GPO_EXTI_PORT, gpio_init_structure.Pin);

  return BSP_ERROR_NONE;

}

/**
  * @brief  This function get the GPO value through GPIO
  * @param  None
  * @retval GPIO pin status
  */
int32_t CUSTOM_GPO_ReadPin( void )
{
  return (int32_t)HAL_GPIO_ReadPin( ST25DV_INT_PIN_GPO_EXTI_PORT, ST25DV_INT_PIN_GPO_EXTI_PIN );
}

/**
  * @brief  This function initialize the GPIO to manage the NFCTAG LPD pin
  * @param  None
  * @return Status
  */
int32_t CUSTOM_LPD_Init( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin   = ST25DV_LPD_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init( ST25DV_LPD_PIN_PORT, &GPIO_InitStruct );

  HAL_GPIO_WritePin( ST25DV_LPD_PIN_PORT, ST25DV_LPD_PIN, GPIO_PIN_RESET );

  return BSP_ERROR_NONE;
}

/**
  * @brief  DeInit LPD.
  * @param  None.
  * @return Status
  * @note LPD DeInit does not disable the GPIO clock nor disable the Mfx
  */
int32_t CUSTOM_LPD_DeInit( void )
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* DeInit the GPIO_GPO pin */
  gpio_init_structure.Pin = ST25DV_LPD_PIN;
  HAL_GPIO_DeInit( ST25DV_LPD_PIN_PORT, gpio_init_structure.Pin);

  return BSP_ERROR_NONE;
}

/**
  * @brief  This function get the LPD state
  * @param  None
  * @retval GPIO pin status
  */
int32_t CUSTOM_LPD_ReadPin( void )
{
  return (int32_t)HAL_GPIO_ReadPin( ST25DV_LPD_PIN_PORT, ST25DV_LPD_PIN );
}

/**
  * @brief  This function sets the LPD GPIO
  * @param  None
  * @return Status
  */
int32_t CUSTOM_LPD_On( void )
{
  HAL_GPIO_WritePin( ST25DV_LPD_PIN_PORT, ST25DV_LPD_PIN, GPIO_PIN_SET );

  return BSP_ERROR_NONE;

}

/**
  * @brief  This function resets the LPD GPIO
  * @param  None
  * @return Status
  */
int32_t CUSTOM_LPD_Off( void )
{
  HAL_GPIO_WritePin( ST25DV_LPD_PIN_PORT, ST25DV_LPD_PIN, GPIO_PIN_RESET );

  return BSP_ERROR_NONE;

}

/**
  * @brief  This function toggles the LPD GPIO
  * @param  None
  * @return Status
  */
int32_t CUSTOM_LPD_Toggle( void )
{
  HAL_GPIO_TogglePin( ST25DV_LPD_PIN_PORT, ST25DV_LPD_PIN );

  return BSP_ERROR_NONE;

}

/**
  * @brief  BSP GPO callback
  * @retval None.
  */
__weak void BSP_GPO_Callback(void)
{
  /* Prevent unused argument(s) compilation warning */

  /* This function should be implemented by the user application.
     It is called into this driver when an event on Button is triggered. */
}

void BSP_GPO_IRQHandler(void)
{
  HAL_EXTI_IRQHandler(&GPO_EXTI);
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
