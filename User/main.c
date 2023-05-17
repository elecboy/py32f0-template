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

#define VDDA_APPLI                       ((uint32_t)3300)

__IO uint16_t uhADCxConvertedData_Voltage_mVolt = 0;

uint32_t ADCxConvertedDatas[4];

int main(void)
{
  BSP_RCC_HSI_PLL48MConfig();

  BSP_USART_Config(115200);
  printf("ADC Timer Trigger DMA Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_DMAConfig();
  APP_ADCConfig();
  // Start ADC regular conversion and wait for next external trigger
  LL_ADC_REG_StartConversion(ADC1);

  APP_TimerInit();

  while (1);
}

void APP_TransferCompleteCallback(void)
{
  /* Convert the adc value to voltage value */
  uhADCxConvertedData_Voltage_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, ADCxConvertedDatas[0], LL_ADC_RESOLUTION_12B);
  printf("Channel1 voltage %d mV\r\n", uhADCxConvertedData_Voltage_mVolt);

  uhADCxConvertedData_Voltage_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, ADCxConvertedDatas[1], LL_ADC_RESOLUTION_12B);
  printf("Channel4 voltage %d mV\r\n", uhADCxConvertedData_Voltage_mVolt);

  uhADCxConvertedData_Voltage_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, ADCxConvertedDatas[2], LL_ADC_RESOLUTION_12B);
  printf("Channel Vref voltage %d mV\r\n", uhADCxConvertedData_Voltage_mVolt);

  uhADCxConvertedData_Voltage_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, ADCxConvertedDatas[3], LL_ADC_RESOLUTION_12B);
  printf("Channel TEMP voltage %d mV\r\n\r\n", uhADCxConvertedData_Voltage_mVolt);
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
