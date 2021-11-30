#include "main.h"

#define STR_LEN	 100

void CS_State(uint8_t state);     // CE# signal
void LvlProtection(uint8_t lvl);  // Setting memory protection ALL-...-NONE
void WriteStatusRegister(void);   // Set enable to write command in Status Register
void WriteEnable(void); 		  // Set write enable in Status Register
void WriteDisable(void);          // Set write disable in Status Register
void AAIEnable(void);			  // Set enable SO to output RY/BY# while AAI writing
void AAIDisable(void);			  // Set disable SO to output RY/BY# after AAI writing
uint8_t ReadStatusRegister(void); // Reading status register Busy bit to know is during operation still going
uint8_t AddressByte(uint8_t byte, uint32_t adr); 					  // Returns 1 - 3 byte of 3 byte address
void ChipErase(void);			  // Full chip memory erase
void WaitForBuisyBit(void);		  // Waiting till AAI writing ends (signal RY/BY# from SO)
void WriteStringAAI(uint32_t adr, uint8_t string[], uint16_t lenght); // AAI writing one string on address
void WriteTimeCapsule(uint8_t timeCps[][STR_LEN]);					  // Write entire Time Capsule
void ReadTimeCapsule(void);		  // Read Time Capsule and show it
void ShowInfo(void);			  // Show info
