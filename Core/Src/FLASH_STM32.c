#include "FLASH_STM32.h"
#include "stdio.h"
uint8_t lengthPage;

void Flash_delBuff(char* data, uint16_t len)
{
	for(uint8_t i = 0; i < len; i++)
	{
		data[i] = 0;
	}
}

void Flash_Lock()
{
	HAL_FLASH_Lock();
}

void Flash_Unlock()
{
	HAL_FLASH_Unlock();
}

#if _FLASHTYPE_ == SECTOR
uint32_t Flash_GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7))*/
  {
    sector = FLASH_SECTOR_7;
  }
  return sector;
}
#endif

void Flash_Erase(uint32_t addr)
{
	#if _FLASHTYPE_ == SECTOR
		uint32_t FirstSector, SectorError = 0;
		FirstSector = Flash_GetSector(addr);
		static FLASH_EraseInitTypeDef EraseInitStruct;
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		EraseInitStruct.Sector = FirstSector;
		EraseInitStruct.NbSectors = 1;
		if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
		{
			//print log here
			__NOP();
		}
	#else
		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t PageError = 0;
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = addr;
		EraseInitStruct.NbPages     = 1;
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
		{
			//print log here
			__NOP();
		}
	#endif
}

void Flash_WriteIntType(uint32_t addr, uint32_t data, uint32_t typeprogram)
{
	HAL_FLASH_Program(typeprogram, addr, data);
}

uint32_t Flash_ReadIntType(uint32_t addr)
{
	uint32_t* val = (uint32_t *)addr;
	return *val;
}

void Flash_WriteCharType(uint32_t addr, char* data, uint8_t typeprogram)
{
	uint16_t i;
	FLASH->CR |= FLASH_CR_PG;
	int var = 0;
	lengthPage = strlen(data);
	for(i=0; i<lengthPage; i+=1)
	{
		while((FLASH->SR&FLASH_SR_BSY)){};
		var = (int)data[i];
		#if _STM32CHIP_ != F3
		if(typeprogram == FLASH_TYPEPROGRAM_BYTE)
		{
			HAL_FLASH_Program(typeprogram, (addr + i), var);
		}
		else if(typeprogram == FLASH_TYPEPROGRAM_HALFWORD)
		{
			HAL_FLASH_Program(typeprogram, (addr + i*2), var);
		}
		else
		{
			//Print log here
			break;
		}
		#else
		{
			HAL_FLASH_Program(typeprogram, (addr + i*2), var);
		}
		#endif
	}
	while((FLASH->SR&FLASH_SR_BSY)){};
	FLASH->CR &= ~FLASH_CR_PG;
}

uint32_t Flash_ReadCharType(char* dataOut, uint32_t addr, uint8_t typeprogram)
{
	Flash_delBuff(dataOut, strlen(dataOut));
	uint32_t ind = 0;
	uint32_t count = 0;
	uint32_t sizearea = 0;
	#if _FLASHTYPE_ == SECTOR
		sizearea = SIZE_OF_SECTOR;
	#else
		sizearea = SIZE_OF_PAGE;
	#endif
	for(ind = addr; ind < addr+sizearea; )
	{
		if((unsigned char)Flash_ReadIntType(ind) == 0xFF) break;
		else
		{
			dataOut[count++] = Flash_ReadIntType(ind);
		}
		#if _STM32CHIP_ != F3
		if(typeprogram == FLASH_TYPEPROGRAM_BYTE)
		{
			ind=ind+0x01;
		}
		else
		{
			ind=ind+0x02;
		}
		#else
		{
			ind=ind+0x02;
		}
		#endif
	}
	return count;
}
