/**
  ******************************************************************************
  * @file    py32f030xx_bsp_kbd.c
  * @author  MCU Application Team
  * @brief   This file provides set of firmware functions to manage Leds, 
  *          push-button available on Start Kit.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx_bsp_kbd.h"

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT };
const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN };
const uint8_t BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn };
const uint32_t BUTTON_EXTI_LINE[BUTTONn] = {USER_BUTTON_EXTI_LINE };

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability
  * @retval None
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);

  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_PULL_NO);
  /* LL_GPIO_SetPinSpeed(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_SPEED_FREQ_HIGH); */

  if(ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    LL_EXTI_EnableIT(BUTTON_EXTI_LINE[Button]);
    LL_EXTI_EnableFallingTrig(BUTTON_EXTI_LINE[Button]);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F);
    NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

/**
  * @brief  Push Button DeInit.
  * @param  Button: Button to be configured
  *   This parameter should be: BUTTON_USER
  * @note PB DeInit does not disable the GPIO clock
  * @retval None
  */
void BSP_PB_DeInit(Button_TypeDef Button)
{
  NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  LL_GPIO_SetPinMode(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_MODE_ANALOG);
  /* LL_GPIO_SetPinSpeed(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_SPEED_FREQ_LOW); */
  /* LL_GPIO_SetPinPull(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_PULL_NO);         */
  /* LL_GPIO_SetAFPin_8_15(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_AF_0);         */
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_USER
  * @retval Button state.
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return LL_GPIO_IsInputPinSet(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
