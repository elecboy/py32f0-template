/***
 * Demo: ADC With DMA Triggered by TIM1
 * 
 * PY32          
 * PA4      ------> Input voltage between 0V ~ 3.3V
 * 
 * PA2(TX)  ------> RX
 * PA3(RX)  ------> TX
 */
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"
#include "adc_drv.h"
#include "I2C_Libary.h"


int main(void)
{
  BSP_RCC_HSI_PLL48MConfig();

  BSP_USART_Config(115200);
  printf("I2C and ADC Timer Trigger DMA Demo\r\nClock: %ld\r\n", SystemCoreClock);

  i2c_config();
  test_at24cxx();
  APP_DMAConfig();
  APP_ADCConfig();
  // Start ADC regular conversion and wait for next external trigger
  LL_ADC_REG_StartConversion(ADC1);

  APP_TimerInit();

  while (1);
}

void APP_ErrorHandler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1);
}
#endif /* USE_FULL_ASSERT */
