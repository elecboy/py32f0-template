#include <string.h>
#include "bsp_i2c.h"
#include "I2C_Libary.h"
#include "py32f0xx_bsp_printf.h"

#define MAX_ERROR_NUM 8

#define I2C1_PORT GPIOF
#define I2C1_SCL LL_GPIO_PIN_1
#define I2C1_SDA LL_GPIO_PIN_0
#define I2C1_OWN_ADDRESS7      0x82
#define MASTER_ADDRESS I2C1_OWN_ADDRESS7

void delay_us(uint32_t nus,uint32_t fac_us);
/**
  * @brief  us级延时
  * @param  nus = 延时的us量,
  *         fac_us=系统时钟/1000000
  * @retval 无
  */
void delay_us(uint32_t nus,uint32_t fac_us)
{
  LL_SYSTICK_DisableIT();
  uint32_t temp;
  SysTick->LOAD=nus*fac_us;
  SysTick->VAL=0x00;
  SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
  do
  {
    temp=SysTick->CTRL;
  }
  while((temp&0x01)&&!(temp&(1<<16)));
  SysTick->CTRL=0x00;
  SysTick->VAL =0x00;
}

static void APP_CheckEndOfTransfer(void);
static uint8_t APP_Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength);

// uint16_t Current_Slave_Addr = 0;
// I2C_Error_TypeDef Error = {0};

void I2C_WriteByte(uint32_t I2Cx,uint8_t Slave_Addr,uint8_t Data)
{
  BSP_I2C_MasterTransmit(Slave_Addr, &Data, 1, 1000);
}

void I2C_WriteRegBytes(uint32_t I2Cx , uint8_t Slave_Addr,uint8_t Reg,uint8_t* Data,uint8_t Length)
{
    BSP_I2C_Transmit(Slave_Addr, Reg, Data, Length, 1000, I2C_MEMADD_SIZE_16BIT);
}

void I2C_ReadBytes(uint32_t I2Cx ,uint8_t Slave_Addr,uint8_t Register,uint8_t Length,uint8_t* Array,enum LMSB Word)
{
    BSP_I2C_Receive(Slave_Addr, Register, Array, Length, 1000, I2C_MEMADD_SIZE_16BIT);
}

int32_t i2c_read(void *handle, uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t size)
{
    I2C_ReadBytes(1, address<<1, reg, size, buffer, LSB);
    return 1;
}

int32_t i2c_write(void *handle, uint8_t address, uint8_t reg, const uint8_t *buffer, uint16_t size)
{
    I2C_WriteRegBytes(1, address<<1, reg, (uint8_t* )buffer, size);
    return 1;
}

void i2c_bus_reset(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);

  // PF1 SCL/PF0 SDA 
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  delay_us(10, 48);
  LL_GPIO_SetOutputPin(GPIOF, LL_GPIO_PIN_0 | LL_GPIO_PIN_1);

  for(uint8_t i=0; i<9; i++)
  {
    LL_GPIO_SetOutputPin(GPIOF, LL_GPIO_PIN_1);
    delay_us(10, 48);
    LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_1);
    delay_us(10, 48);
  }
  LL_GPIO_SetOutputPin(GPIOF, LL_GPIO_PIN_1);

}

void i2c_config(void)
{
  // i2c_bus_reset();
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  // PF1 SCL
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  // PF0 SDA
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);

  BSP_I2C_Config();
  //BSP_I2C_Scan();
}

void enable_i2c_int(void)
{
    /* enable the I2C1 interrupt */
    // i2c_interrupt_enable(I2C1, I2C_INT_ERR);
    // i2c_interrupt_enable(I2C1, I2C_INT_EV);
    // i2c_interrupt_enable(I2C1, I2C_INT_BUF);

}

#define I2C_ADDRESS        0xA0 >> 1     /* 本机\从机地址 */
#define I2C_MEMADD_SIZE_8BIT            0x00000001U /* 从机内部地址大小为8位 */
#define I2C_MEMADD_SIZE_16BIT           0x00000010U /* 从机内部地址大小为16位 */
uint8_t aTxBuffer[24] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
uint8_t aRxBuffer[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t         *pBuffPtr   = 0;
__IO uint16_t   XferCount   = 0;
__IO uint32_t   Devaddress  = 0;

/**
  * @brief  校验数据函数
  * @param  无
  * @retval 无
  */
static void APP_CheckEndOfTransfer(void)
{
  /* 比较发送数据和接收数据 */
  if(APP_Buffercmp8((uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, sizeof(aRxBuffer)))
  {
    /* 错误处理 */
    printf("I2C EEPROM Write & Read Test Error!\r\n");
  }
  else
  {
    printf("I2C EEPROM Write & Read test Good!\r\n");
  }
}

/**
  * @brief  字符比较函数
  * @param  pBuffer1：待比较缓冲区1
  * @param  pBuffer2：待比较缓冲区2
  * @param  BufferLength：待比较字符的个数
  * @retval 0：比较值相同；1：比较值不同
  */
static uint8_t APP_Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return 1;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}


void test_at24cxx(void)
{
    LL_mDelay(10);
    i2c_write(NULL, I2C_ADDRESS, 0x0, (uint8_t *)aTxBuffer, sizeof(aTxBuffer));

    LL_mDelay(10);

    i2c_read(NULL, I2C_ADDRESS, 0x0, (uint8_t *)aRxBuffer, sizeof(aRxBuffer));
    /* 检查接收到的数据 */
    APP_CheckEndOfTransfer();

}