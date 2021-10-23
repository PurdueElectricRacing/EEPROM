/*
 * eeprom.h
 *
 *  Created on: Dec 6, 2020
 *      Author: Luke Oxley
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include "common/phal_L4/i2c/i2c.h"

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

#define E_DELAY 5 //ms (time delay between I2C operations)

typedef struct {
  uint32_t last_write_time;
  uint16_t next_boundary;
  uint16_t end_loc;
  uint16_t current_addr;
  uint8_t *from;
  struct {
    uint8_t has_write_started:1;
    uint8_t is_write_complete:1;
  };
} eeprom_write_status_t;

extern eeprom_write_status_t e_wr_stat; // eeprom periodic write status

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

//void eepromDump(UART_HandleTypeDef huart);
/**
 * @brief Set all addresses in eeprom to zero
 */
void eepromWipe();

/**
 * @brief           links a data structure in memory to one in eeprom
 *                  if not existing, initializes one in eeprom memory
 * 
 * @param ptr       pointer to location of the data
 * @param size      size in bytes of the data
 * @param name      3 character variable name
 * @param version   version number of the data (change when structure changed!) 
 * @param overwrite set to 1 to prevent clean headers from removing if not linked
 * @return uint8_t  returns 1 if the structure was not found to exist, signifying
 *                  the creation of a new one
 */
uint8_t eepromLinkStruct(void* ptr, uint16_t size, char name[], uint8_t version, uint8_t overwrite);

/**
 * @brief             Used to initialize the eeprom, loads existing header information
 *                    off of the eeprom
 * 
 * @param eepromSpace byte capacity of eeprom
 * @param address     I2C address
 */
void eepromInitialize(uint16_t eepromSpace, uint8_t address);

/**
 * @brief call after linking structs, removes old structs from memory with 
 *        overwrite protection turned off
 */
void eepromCleanHeaders();

/**
 * @brief          Loads what data was in eeprom and writes it to the 
 *                 linked address
 * 
 * @param name     3 character data label 
 * @return uint8_t returns 1 if the label is unrecognized 
 */
uint8_t eepromLoadStruct(char name[]);

/**
 * @brief          Writes current data at the linked address to eeprom,
 *                 blocks for 5 ms between each write command
 * 
 * @param name     3 character data label 
 * @return uint8_t returns 1 if the label is unrecognized 
 */
uint8_t eepromSaveStructBlocking(char name[]);

/**
 * @brief          Writes current data at the linked address to eeprom,
 *                 call periodically at a minimum period of 5 ms until 
 *                 the eeprom write status is_write_complete bit is 
 *                 set to high
 * 
 * @param name     3 character data label 
 * @return uint8_t returns 1 if the label is unrecognized 
 */
uint8_t eepromSaveStructPeriodic(char name[]);


#endif
