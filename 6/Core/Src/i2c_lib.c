#include "i2c_lib.h"

I2C_HandleTypeDef hi2c1;

void setDuty(uint16_t Duty) {
	uint8_t devID = 0x80;
	uint8_t TxBuff[2];
	Duty = ((4096 * Duty) / 100) - 1;

	TxBuff[0] = 0x00;
	TxBuff[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuff, 2, 1000);

	TxBuff[0] = 0xFA;
	TxBuff[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuff, 2, 1000);

	TxBuff[0] = 0xFB;
	TxBuff[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuff, 2, 1000);

	TxBuff[0] = 0xFC;
	TxBuff[1] = Duty % 0x100;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuff, 2, 1000);

	TxBuff[0] = 0xFD;
	TxBuff[1] = Duty / 0x100;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuff, 2, 1000);
}
void turnOff(void) {
	uint8_t devID = 0x80;
	uint8_t TxBuff[2];

	TxBuff[0] = 0x00;
	TxBuff[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuff, 2, 1000);

	TxBuff[0] = 0xFA;
	TxBuff[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuff, 2, 1000);

	TxBuff[0] = 0xFB;
	TxBuff[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuff, 2, 1000);

	TxBuff[0] = 0xFC;
	TxBuff[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuff, 2, 1000);

	TxBuff[0] = 0xFD;
	TxBuff[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuff, 2, 1000);
}
void setFreq(uint16_t PWMFreq) {
	uint8_t devID = 0x80;
	uint8_t TxBuff[2];

	uint16_t Prescale = (16000000 / (4096 * PWMFreq)) - 1;

	TxBuff[0] = 0x00;
	TxBuff[1] = 0x10;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuff, 2, 1000);

	TxBuff[0] = 0xFE;
	TxBuff[1] = Prescale;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuff, 2, 1000);

	TxBuff[0] = 0x00;
	TxBuff[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuff, 2, 1000);
}
