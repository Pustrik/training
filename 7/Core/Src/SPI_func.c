#include "SPI_func.h"
#include "SPI_var.h"
#include <string.h>
#include <stdio.h>
#include "main.h"

void CS_State(uint8_t state)
{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, state);
}

void LvlProtection(uint8_t lvl)
{
	uint8_t buffer[2] = {WRSR, lvl};
	CS_State(CS_ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &buffer, 2, 100);
	CS_State(CS_OFF);
}

void WriteStatusRegister(void)
{
	CS_State(CS_ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EWSR, 1, 100);
	CS_State(CS_OFF);
	LvlProtection(NONE);
}

void WriteEnable(void)
{
	CS_State(CS_ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &WREN, 1, 100);
	CS_State(CS_OFF);
}

void WriteDisable(void)
{
	CS_State(CS_ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &WRDI, 1, 100);
	CS_State(CS_OFF);
}

void AAIEnable(void)
{
	CS_State(CS_ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EBSY, 1, 100);
	CS_State(CS_OFF);
}

void AAIDisable(void)
{
	CS_State(CS_ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &DBSY, 1, 100);
	CS_State(CS_OFF);
}

uint8_t ReadStatusRegister(void)
{
	uint8_t status[2];
	uint8_t buffer[2] = {RDSR, '0x00'};
	CS_State(CS_ON);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &buffer, (uint8_t*) &status, 2, 100);
	CS_State(CS_OFF);
	return status[1];
}

uint8_t AddressByte(uint8_t byte, uint32_t adr)
{
	uint8_t temp;
	switch(byte)
	{
		case 1:
			temp = (uint8_t)(adr >> 16);
			return temp;
			break;
		case 2:
			temp = (uint8_t)(adr >> 8);
			return temp;
			break;
		case 3:
			temp = (uint8_t)adr;
			return temp;
			break;
	}
	return 0;
}

void ChipErase(void)
{
	CS_State(CS_OFF);
	WriteStatusRegister();
	WriteEnable();
	CS_State(CS_ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &CHER, 1, 100);
	CS_State(CS_OFF);
	WriteDisable();
	while(ReadStatusRegister() & B_BYTE) {};
}

void WaitForBuisyBit(void)
{
	CS_State(CS_ON);
	while (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4));
	CS_State(CS_OFF);
}

void WriteStringAAI(uint32_t adr, uint8_t string[], uint16_t lenght)
{
	WriteStatusRegister();
	AAIEnable();
	WriteEnable();

	CS_State(CS_ON);
	uint8_t transmit[] = { AAI, AddressByte(1, adr), AddressByte(2, adr), AddressByte(3, adr), string[0], string[1]};
	HAL_SPI_Transmit(&hspi1, transmit, sizeof(transmit), 100);
	CS_State(CS_OFF);
	WaitForBuisyBit();

	for (uint8_t i = 2; i <= lenght; i += 2)
	{
		CS_State(CS_ON);
		uint8_t transmit[] = { AAI, string[i], string[i+1]};
		HAL_SPI_Transmit(&hspi1, (uint8_t*) &transmit, sizeof(transmit), 100);
		CS_State(CS_OFF);
		WaitForBuisyBit();
	}
	WriteDisable();
	AAIDisable();
}

void WriteTimeCapsule(uint8_t timeCps[STR_NUM][STR_LEN])
{

	for(uint8_t CurrentStringCounter = 0; CurrentStringCounter < STR_NUM; CurrentStringCounter++)
		{
			uint8_t CurrentStringBuffer[STR_LEN] = {0};
			strcpy(CurrentStringBuffer, &timeCps[CurrentStringCounter][0]);
			WriteStringAAI(StartAddress, &CurrentStringBuffer, STR_LEN);
			StartAddress += SECTOR;
		}
	StartAddress = START_ADR;
}

void ReadTimeCapsule(void)
{

	uint8_t RDBuff[STR_LEN] = {0};
	for(uint8_t StrCounter = 0; StrCounter < STR_NUM; StrCounter++)
	{
		uint8_t ReadCommand[4] = {READ, AddressByte(1, StartAddress), AddressByte(2, StartAddress), AddressByte(3, StartAddress)};
		CS_State(CS_ON);
		HAL_SPI_Transmit(&hspi1, ReadCommand, sizeof(ReadCommand), 100);
		HAL_SPI_Receive(&hspi1, RDBuff, sizeof(RDBuff), 100);
		CS_State(CS_OFF);
		while (ReadStatusRegister() & 0x01) {};
		for(uint8_t j = 0; j < STR_LEN; j++)
			if(RDBuff[j] == 0xFF)
				RDBuff[j] = 0x00;
		HAL_UART_Transmit(&huart3, RDBuff, sizeof(RDBuff), 100);
		StartAddress += SECTOR;
	}
	StartAddress = START_ADR;
}

void ShowInfo()
{
	uint8_t Info[] = {"1 - Erase all memory\n\r2 - Write time capsule\n\r3 - Read time capsule from flash mamory\n\r"};
	HAL_UART_Transmit(&huart3, Info, sizeof(Info), 100);
}
