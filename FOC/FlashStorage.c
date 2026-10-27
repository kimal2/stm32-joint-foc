#include "FlashStorage.h"
#include <string.h>

/*
 * STM32F103C8T6: 64KB flash, 1KB per page.
 * Original sample used pages 30/31 (0x08007800/0x08007C00), but the ported
 * FOC image is already larger than 30KB, so those pages now contain program
 * code. Store the parameters in the last two pages instead.
 * IMPORTANT: keep the firmware image below 0x0800F800 (62KB).
 */
#define FLASH_STORAGE_PARAM_ADDR 0x0800F800 //第62页：电机结构参数
#define FLASH_STORAGE_ID_ADDR    0x0800FC00 //第63页：电机ID
#define FLASH_STORAGE_AVAIL_FLAG 0xA5A5A5A5 //Flash数据有效标志

//擦除电机结构参数
void Flash_EraseMotorParam()
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.PageAddress = FLASH_STORAGE_PARAM_ADDR;
	erase.NbPages = 1;
	uint32_t error = 0;
	HAL_FLASHEx_Erase(&erase, &error);
	HAL_FLASH_Lock();
}

//将电机结构参数存入Flash
void Flash_SaveMotorParam(int poles, float zero_elec_angle, int dir)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.PageAddress = FLASH_STORAGE_PARAM_ADDR;
	erase.NbPages = 1;
	uint32_t error = 0;
	HAL_FLASHEx_Erase(&erase, &error);
	
	uint32_t buf[5];
	buf[0] = FLASH_STORAGE_AVAIL_FLAG;
	buf[1] = *((uint32_t*)&poles);
	buf[2] = *((uint32_t*)&zero_elec_angle);
	buf[3] = *((uint32_t*)&dir);
	for(uint8_t i=0; i<4; i++)
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_PARAM_ADDR+i*4, buf[i]);
	
	HAL_FLASH_Lock();
}

//从Flash中读取电机结构参数
int Flash_ReadMotorParam(int* poles, float* zero_elec_angle, int* dir)
{
	uint32_t *buf = (uint32_t*)FLASH_STORAGE_PARAM_ADDR;
	if(buf[0] != FLASH_STORAGE_AVAIL_FLAG)
		return -1;
	*poles = *((int*)&buf[1]);
	*zero_elec_angle = *((float*)&buf[2]);
	*dir = *((int*)&buf[3]);
	return 0;
}

//将电机ID存入Flash
void Flash_SaveMotorID(uint8_t id)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.PageAddress = FLASH_STORAGE_ID_ADDR;
	erase.NbPages = 1;
	uint32_t error = 0;
	HAL_FLASHEx_Erase(&erase, &error);
	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ID_ADDR, FLASH_STORAGE_AVAIL_FLAG);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ID_ADDR+4, id);
	
	HAL_FLASH_Lock();
}

//从Flash中读取电机ID
uint8_t Flash_ReadMotorID()
{
	uint32_t *buf = (uint32_t*)FLASH_STORAGE_ID_ADDR;
	if(buf[0] != FLASH_STORAGE_AVAIL_FLAG)
		return 0;
	return buf[1];
}
