#include <string.h>
#include "bsp_i2c.h"
#include "I2C_Libary.h"

#define MAX_ERROR_NUM 8

#define I2C1_PORT GPIOF
#define I2C1_SCL LL_GPIO_PIN_1
#define I2C1_SDA LL_GPIO_PIN_0
#define I2C1_OWN_ADDRESS7      0x82
#define MASTER_ADDRESS I2C1_OWN_ADDRESS7


uint16_t Current_Slave_Addr = 0;
I2C_Error_TypeDef Error = {0};

void I2C_WriteByte(uint32_t I2Cx,uint8_t Slave_Addr,uint8_t Data)
{
}

void I2C_WriteRegBytes(uint32_t I2Cx , uint8_t Slave_Addr,uint8_t Reg,uint8_t* Data,uint8_t Length)
{
    BSP_I2C_Transmit(Slave_Addr, Reg, Data, Length, 1000);
}

void I2C_ReadBytes(uint32_t I2Cx ,uint8_t Slave_Addr,uint8_t Register,uint8_t Length,uint8_t* Array,enum LMSB Word)
{
    BSP_I2C_Receive(Slave_Addr, Register, Array, Length, 1000);
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

void i2c_config(void)
{
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
  BSP_I2C_Scan();
}

void enable_i2c_int(void)
{
    /* enable the I2C1 interrupt */
    // i2c_interrupt_enable(I2C1, I2C_INT_ERR);
    // i2c_interrupt_enable(I2C1, I2C_INT_EV);
    // i2c_interrupt_enable(I2C1, I2C_INT_BUF);

}