
const uint8_t WRITE  = 0x02;   // Write one data byte to Memory instruction
const uint8_t WREN   = 0x06;   // Write enable instruction
const uint8_t WRDI   = 0x04;   // Write disable instruction
const uint8_t RDSR   = 0x05;   // Read Status Register instruction
const uint8_t READ   = 0x03;   // 25Mhz Read from Memory instruction
const uint8_t HREAD  = 0x0B;   // 50Mhz Read from Memory instruction
const uint8_t WRSR   = 0x01;   // Write Status Register instruction
const uint8_t CHER   = 0x60;   // Bulk Erase instruction
const uint8_t EWSR	 = 0x50;   // Enable write status register
const uint8_t D_BYTE = 0xFF;   // Dummy byte
const uint8_t EBSY	 = 0x70;   // Enable SO to output RY/BY# status during AAI programming
const uint8_t DBSY	 = 0x80;   // Disable SO as RY/BY# status during AAI programming
const uint8_t AAI	 = 0xAD;   // Auto Address Increment Programming
const uint8_t B_BYTE = 0x01;   // Busy bute

#define STR_NUM	  16    	   // Time capsule amount of strings
#define STR_LEN	  100    	   // Time capsule string length
#define SECTOR    4096    	   // One sector size
#define START_ADR 0x00		   // Start address

#define NONE     0x00  // None
#define P1_32    0x01  // 1/32
#define P1_16    0x02  // 1/16
#define P1_8	 0x03  // 1/8
#define P1_4     0x04  // 1/4
#define P1_2	 0x05  // 1/2
#define ALL 	 0x06  // All

enum
{
	CS_ON,
	CS_OFF
};

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart3;
uint32_t StartAddress = START_ADR;
