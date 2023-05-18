#ifndef I2C_Lib
#define I2C_Lib

#include <stdint.h>
#include "py32f0xx_ll_bus.h"
#include "py32f0xx_ll_cortex.h"
#include "py32f0xx_ll_dma.h"
#include "py32f0xx_ll_exti.h"
#include "py32f0xx_ll_gpio.h"
#include "py32f0xx_ll_pwr.h"
#include "py32f0xx_ll_rcc.h"
#include "py32f0xx_ll_system.h"
#include "py32f0xx_ll_tim.h"
#include "py32f0xx_ll_i2c.h"
#include "py32f0xx_ll_utils.h"

#define ENABLE_I2C_SLAVE 1
#define I2C_SLAVE_ADDRESS7 0x72

#define USE_I2C I2C1

enum LMSB {MSB,LSB};
struct I2C_Error
{
	uint16_t Slave_Address;
	uint32_t Blocked_I2C_FLAG;
	struct I2C_Error* Next;
};
typedef struct I2C_Error I2C_Error_TypeDef;
void i2c_config(void);
void enable_i2c_int(void);
void I2C_WriteByte(uint32_t I2Cx ,uint8_t Slave_Addr, uint8_t Data);
void I2C_WriteRegBytes(uint32_t I2Cx , uint8_t Slave_Addr,uint8_t Reg,uint8_t* Data,uint8_t Length);
void I2C_ReadBytes(uint32_t I2Cx ,uint8_t Slave_Addr,uint8_t Register,uint8_t Length,uint8_t* Array,enum LMSB Word);

int32_t i2c_read(void *handle, uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t size);
int32_t i2c_write(void *handle, uint8_t address, uint8_t reg, const uint8_t *buffer, uint16_t size);
#endif
