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

#define VDDA_APPLI                       ((uint32_t)3300)

__IO uint16_t uhADCxConvertedData_Voltage_mVolt = 0;

static uint32_t ADCxConvertedDatas[4];

static void APP_ADCConfig(void);
static void APP_TimerInit(void);
static void APP_DMAConfig(void);

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

static void APP_TimerInit(void)
{
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
  LL_TIM_SetPrescaler(TIM1, (SystemCoreClock / 6000) - 1);
  LL_TIM_SetAutoReload(TIM1, 6000 - 1);
  /* Triggered by update */
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);

  LL_TIM_EnableCounter(TIM1);
}

static void APP_ADCConfig(void)
{
  __IO uint32_t backup_setting_adc_dma_transfer = 0;

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  LL_ADC_Reset(ADC1);
  // Calibrate start
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Backup current settings */
    backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
    /* Turn off DMA when calibrating */
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
    LL_ADC_StartCalibration(ADC1);

    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);

    /* Delay 1ms(>= 4 ADC clocks) before re-enable ADC */
    LL_mDelay(1);
    /* Apply saved settings */
    LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);
  }
  // Calibrate end

  /* PA4 as ADC input */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1 | LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
  /* Set ADC channel and clock source when ADEN=0, set other configurations when ADSTART=0 */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_PATH_INTERNAL_TEMPSENSOR);

  LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV2);
  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
  LL_ADC_SetResolution(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
  LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);

  /* Set TIM1 as trigger source */
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);

  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
  LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
  LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_1 | LL_ADC_CHANNEL_4| LL_ADC_CHANNEL_VREFINT| LL_ADC_CHANNEL_TEMPSENSOR);

  LL_ADC_Enable(ADC1);
}

static void APP_DMAConfig(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

  // Remap ADC to LL_DMA_CHANNEL_1
  LL_SYSCFG_SetDMARemap_CH1(LL_SYSCFG_DMA_MAP_ADC);

  /* DMA通道1初始化 */
  LL_DMA_InitTypeDef DMA_InitStruct;
  DMA_InitStruct.PeriphOrM2MSrcAddress  = (uint32_t)&ADC1->DR;
  DMA_InitStruct.MemoryOrM2MDstAddress  = (uint32_t)&ADCxConvertedDatas;
  DMA_InitStruct.Direction              = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStruct.Mode                   = LL_DMA_MODE_CIRCULAR;
  DMA_InitStruct.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStruct.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  DMA_InitStruct.NbData                 = 0x00000004U;
  DMA_InitStruct.Priority               = LL_DMA_PRIORITY_HIGH;
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &DMA_InitStruct);
  // Enable DMA channel 1
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  // Enable transfer-complete interrupt
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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
