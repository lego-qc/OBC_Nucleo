#include "iic.h"


/* i2cdev functions (plus parameter: i2c handler have to be in a global variable called hi2c1 */

int8_t I2C_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data){
	uint8_t registerAddr = regAddr;
	I2C_HandleTypeDef* Handle = &hi2c1;

	if(HAL_I2C_Master_Transmit(Handle,  devAddr, &registerAddr, 1, 1000) != HAL_OK){
		return -1;
	}

	if(HAL_I2C_Master_Receive(Handle, devAddr, data, length, 1000) != HAL_OK){
		return -1;
	}

	return length; //if we return here, we are sure that we got the full data
}



int8_t I2C_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t* data) {
    return I2C_readBytes(devAddr, regAddr, 1, data);
}

int8_t I2C_readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = I2C_readByte(devAddr, regAddr, &b)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

int8_t I2C_readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t* data) {
    uint8_t b;
    uint8_t count = I2C_readByte(devAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}










bool I2C_writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
	I2C_HandleTypeDef* Handle = &hi2c1;
	uint8_t* d;
	int i;
	d = malloc(length + 1);

	if(d == 0)
		return false;

	d[0] = regAddr;

	for(i = 1; i < (length + 1); i++){
		d[i] = data[i-1];
	}


	if(HAL_I2C_Master_Transmit(Handle,  devAddr, d, length + 1, 1000) != HAL_OK){
		return false;
	}

	free(d);


	return true; //success
}

bool I2C_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return I2C_writeBytes(devAddr, regAddr, 1, &data);
}

bool I2C_writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (I2C_readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return I2C_writeByte(devAddr, regAddr, b);
    } else {
        return false;
    }
}

bool I2C_writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    I2C_readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return I2C_writeByte(devAddr, regAddr, b);
}

bool I2C_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data){
	I2C_HandleTypeDef* Handle = &hi2c1;
	uint8_t d[3];
	d[0] = regAddr;
	d[1] = (data >> 8); //MSB
	d[2] = (data); //LSB
	if(HAL_I2C_Master_Transmit(Handle,  devAddr, d, 2, 1000) != HAL_OK){
		return false;
	}
	return true;


}

/*end of i2cdev functions */

