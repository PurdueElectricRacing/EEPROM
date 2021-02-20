/*
 * eeprom.h
 *
 *  Created on: Dec 6, 2020
 *      Author: Luke Oxley
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include "main.h"

//I was told this was required:
#define PER 1
#define GREAT PER

//#define DEVICE_ADDR 0x50 //Before Bit Shifting

//writing
#define WRITE_ENABLE 0x00
#define WRITE_TIMEOUT 1000

#define PAGE_SIZE 32

//reading
#define READ_ENABLE 0x01

#define NAME_SIZE 3

//Header management

typedef struct {
  char name[NAME_SIZE];
  uint8_t version;
  uint16_t size;
  uint16_t address_on_eeprom;
  void* ptr_to_data;
} header_t;

#define HEADER_SIZE 8 //bytes per header

#define MAX_HEADER_COUNT 20 //8*20 -> 160/4000 number of entries worth of space allocated

#define OVERWRITE_BIT 7
#define OVERWRITE_MASK 0b01111111
#define MAX_VERSION 127

//errors
typedef enum{
  COM_TIMEOUT,
  COM_ERROR,
  MAX_HEADER,
  MAX_MEM,
  HEADER_NOT_FOUND
} eeprom_error_t;

//macros
#define SET_ADDRESS(address, write_en) ((address << 1) | write_en)

void eepromDump(UART_HandleTypeDef huart);
void eepromWipe();
uint8_t eepromLinkStruct(void* ptr, uint16_t size, char name[], uint8_t version, uint8_t overwrite);
void eepromInitialize(I2C_HandleTypeDef* i2c, uint16_t eepromSpace, uint8_t address);
void eepromCleanHeaders();
uint8_t eepromLoadStruct(char name[]);
uint8_t eepromSaveStruct(char name[]);

#endif
