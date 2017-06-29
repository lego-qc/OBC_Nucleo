#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stdlib.h"
#include "stdbool.h"

extern I2C_HandleTypeDef hi2c1; // initialize in the main.c


int8_t I2C_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);

int8_t I2C_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);

int8_t I2C_readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);

int8_t I2C_readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);

bool I2C_writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data);

bool I2C_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);

bool I2C_writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);

bool I2C_writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);

bool I2C_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
