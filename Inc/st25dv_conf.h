/**
 ******************************************************************************
 * @file    st25dv_conf.h
 * @author  SRA Application Team
 * @version V0.0.1
 * @date    28-Nov-2018
 * @brief   This file contains definitions for the ST25DV bus interfaces
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ST25DV_CONF_H__
#define __ST25DV_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wbxx_hal.h"
#include "custom_bus.h"
#include "custom_errno.h"
#include "stm32wbxx_hal_exti.h"

#define CUSTOM_ST25DV_I2C_Init         BSP_I2C1_Init
#define CUSTOM_ST25DV_I2C_DeInit       BSP_I2C1_DeInit
#define CUSTOM_ST25DV_I2C_ReadReg16    BSP_I2C1_ReadReg16
#define CUSTOM_ST25DV_I2C_WriteReg16   BSP_I2C1_WriteReg16
#define CUSTOM_ST25DV_I2C_Recv         BSP_I2C1_Recv
#define CUSTOM_ST25DV_I2C_IsReady      BSP_I2C1_IsReady

#define CUSTOM_ST25DV_GetTick HAL_GetTick

#define ST25DV_LPD_PIN_PORT GPIOB
#define ST25DV_LPD_PIN GPIO_PIN_1
#define ST25DV_INT_PIN_GPO_EXTI_PORT GPIOA
#define ST25DV_INT_PIN_GPO_EXTI_PIN GPIO_PIN_0
#define ST25DV_INT_PIN_GPO_EXTI_LINE EXTI_LINE_0
#define ST25DV_INT_PIN_GPO_EXTI_IRQn EXTI0_IRQn
extern EXTI_HandleTypeDef GPO_EXTI;
#define H_EXTI_0  GPO_EXTI

#define CUSTOM_NFCTAG_INSTANCE         (0)
#define CUSTOM_NFCTAG_GPO_PRIORITY     (0)
#ifdef __cplusplus
}
#endif

#endif /* __ST25DV_CONF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

