/**
  ******************************************************************************
  * @file    custom_nfc04a1.h
  * @author  MMY Application Team
  * @version $Revision: 3351 $
  * @date    $Date: 2017-01-25 17:28:08 +0100 (Wed, 25 Jan 2017) $
  * @brief   This file contains definitions for the bsp_nfc04a1.c
  *          board specific functions.
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
#ifndef __CUSTOM_NFC04A1_H__
#define __CUSTOM_NFC04A1_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "st25dv_conf.h"

#include "st25dv.h"

/** @addtogroup CUSTOM
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief  NFC04A1 Ack Nack enumerator definition
 */
typedef enum
{
  I2CANSW_ACK = 0,
  I2CANSW_NACK
}CUSTOM_I2CANSW_E;

/* External variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/** @defgroup ST25DV_NUCLEO_Exported_Functions
  * @{
  */
int32_t CUSTOM_GPO_Init( void );
int32_t CUSTOM_GPO_DeInit( void );
int32_t CUSTOM_GPO_ReadPin( void );
int32_t CUSTOM_LPD_Init( void );
int32_t CUSTOM_LPD_DeInit( void );
int32_t CUSTOM_LPD_ReadPin( void );
int32_t CUSTOM_LPD_On( void );
int32_t CUSTOM_LPD_Off( void );
int32_t CUSTOM_LPD_Toggle( void );
void BSP_GPO_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_NFC04A1_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
