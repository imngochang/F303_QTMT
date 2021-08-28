#include "stdint.h"
#include "string.h"

#define F4 4
#define F3 3
#define F1 1

#define SECTOR	1
#define	PAGE	0

/*Change your MCU line here*/
//#define _STM32CHIP_ F4
#define _STM32CHIP_ F3
//#define _STM32CHIP_ F1

//#define _FLASHTYPE_ SECTOR
#define _FLASHTYPE_ PAGE

#if _STM32CHIP_ == F4
	#include "stm32f4xx_hal.h"
#elif _STM32CHIP_ == F3
	#include "stm32f3xx_hal.h"
#else
	#include "stm32f1xx_hal.h"
#endif

#if _STM32CHIP_ == F4
	#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
	#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
	#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
	#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
	#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
	#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
	#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
	#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#endif

#if _FLASHTYPE_ == SECTOR
#define SIZE_OF_SECTOR 131072
#else
#define SIZE_OF_PAGE   2048
#endif
void 	 Flash_delBuff(char* data, uint16_t len);
void 	 Flash_Lock(void);
void 	 Flash_Unlock(void);
#if _FLASHTYPE_ == SECTOR
uint32_t Flash_GetSector(uint32_t Address);
#endif
void 	 Flash_Erase(uint32_t addr);
void 	 Flash_WriteIntType(uint32_t addr, uint32_t data, uint32_t typeprogram);
uint32_t Flash_ReadIntType(uint32_t addr);
void 	 Flash_WriteCharType(uint32_t addr, char* data, uint8_t typeprogram);
void 	 Flash_ProgramPage(char* dataIn, uint32_t addr);
uint32_t Flash_ReadCharType(char* dataOut, uint32_t addr, uint8_t typeprogram);
