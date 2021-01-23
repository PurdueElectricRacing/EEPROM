/*
 * eeprom.h
 *
 *  Created on: Dec 6, 2020
 *      Author: Luke Oxley
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include "main.h"

//#define DEVICE_ADDR 0x50 //Before Bit Shifting

//writing
#define WRITE_ENABLE 0x00
#define WRITE_TIMEOUT 1000

#define PAGE_SIZE 32

//reading
#define READ_ENABLE 0x01

#define NAME_SIZE 3

//Header management

struct HeaderNode
{
  char name[NAME_SIZE];
  uint8_t version;
  uint16_t size;
  uint16_t eAddress;
  void* ptr;
  struct HeaderNode* next;
};

#define HEADER_SIZE 8 //bytes per header

#define MAX_HEADER_COUNT 20 //8*20 160/4000 number of entries worth of space allocated

#define OVERWRITE_LOC 7
#define OVERWRITE_MASK 0b01111111
#define MAX_VERSION 127

//errors
enum EEPROM_ERROR{COM_TIMEOUT, COM_ERROR, MAX_HEADER, MAX_MEM};

//macros
#define SET_ADDRESS(address, write_en) (address << 1) | write_en

void eDump(UART_HandleTypeDef huart);
void eWipe();
void eLinkStruct(void* ptr, uint16_t size, char name[], uint8_t version, uint8_t overwrite);
void eInitialize(I2C_HandleTypeDef* i2c, uint16_t eepromSpace, uint8_t address);
void eCleanHeaders();
uint8_t eLoadStruct(char name[]);
uint8_t eSaveStruct(char name[]);

#endif
