#include "eeprom.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

uint8_t g_write_data[4];
HAL_StatusTypeDef ret;

header_t g_headers[MAX_HEADER_COUNT] = {NULL};

uint8_t g_numStructs; //number of entries in header

I2C_HandleTypeDef *i2c01;
uint16_t g_eeprom_size;
uint8_t g_device_addr;

void setAddress(uint16_t addr);
uint8_t readByte(uint16_t addr);
void downloadChunk(uint16_t from_addr, void *to_addr, uint16_t size);
void uploadByte(uint16_t addr, uint8_t val);
void uploadChunk(void *from_addr, uint16_t to_addr, uint16_t size);
header_t *findHeader(char name[]);
void addHeaderEntry(header_t *newHeader);
void updateHeaderEntry(header_t *header);
void sortHeaders();
uint16_t spaceAvailable(uint16_t address);
uint16_t eepromMalloc(uint16_t size);
void removeFromEeprom(char name[]);
void splitVersion(uint8_t *version, uint8_t *overwrite);
void combineVersion(uint8_t *version, uint8_t *overwrite);
void errorFound(eeprom_error_t error);
void loadHeaderEntries();

//sets 'cursor' in eeprom
void setAddress(uint16_t addr)
{
  uint8_t timeout = 0;

  g_write_data[0] = addr >> 8;
  g_write_data[1] = addr & 0xFF;
  HAL_Delay(10);
  ret = HAL_I2C_Master_Transmit(i2c01, SET_ADDRESS(g_device_addr, WRITE_ENABLE), g_write_data, 2, HAL_MAX_DELAY);

  if (ret != HAL_OK)
  {
    errorFound(COM_ERROR);
  }

  for (timeout = 0; i2c01->State != HAL_I2C_STATE_READY && timeout < WRITE_TIMEOUT; timeout++)
  {
    //Wait for the send to stop
  }

  if (timeout > WRITE_TIMEOUT)
  {
    errorFound(COM_TIMEOUT);
  }
}

//reads single byte
uint8_t readByte(uint16_t addr)
{
  uint8_t value;

  setAddress(addr);

  ret = HAL_I2C_Master_Receive(i2c01, SET_ADDRESS(g_device_addr, READ_ENABLE), &value, 1, HAL_MAX_DELAY);

  if (ret != HAL_OK)
  {
    errorFound(COM_ERROR);
  }

  HAL_Delay(5);
  return value;
}

//reads chunk of data
void downloadChunk(uint16_t from_addr, void *to_addr, uint16_t size)
{
  setAddress(from_addr);
  HAL_Delay(5);
  ret = HAL_I2C_Master_Receive(i2c01, SET_ADDRESS(g_device_addr, READ_ENABLE), to_addr, size, HAL_MAX_DELAY);

  if (ret != HAL_OK)
  {
    errorFound(COM_ERROR);
  }
}

//writes single byte
void uploadByte(uint16_t addr, uint8_t val)
{
  g_write_data[0] = addr >> 8;
  g_write_data[1] = addr & 0xFF;
  g_write_data[2] = val;

  uint8_t timeout = 0;
  HAL_Delay(5); //Was not working without a delay...
  ret = HAL_I2C_Master_Transmit(i2c01, SET_ADDRESS(g_device_addr, WRITE_ENABLE), g_write_data, 3, HAL_MAX_DELAY);

  if (ret != HAL_OK)
  {
    errorFound(COM_ERROR);
  }
  for (timeout = 0; i2c01->State != HAL_I2C_STATE_READY && timeout < WRITE_TIMEOUT; timeout++)
  {
    //Wait for the send to stop
  }
  if (timeout > WRITE_TIMEOUT)
  {
    errorFound(COM_TIMEOUT);
  }
}

//uploads chunk ignoring page breaks
void eUploadRaw(void *from_addr, uint16_t to_addr, uint16_t size)
{
  HAL_Delay(5);
  //ret = HAL_I2C_Master_Transmit(i2c01, SET_ADDRESS(g_device_addr, WRITE_ENABLE), from_addr, size, HAL_MAX_DELAY);
  ret = HAL_I2C_Mem_Write(i2c01, SET_ADDRESS(g_device_addr, WRITE_ENABLE), to_addr, I2C_MEMADD_SIZE_16BIT, from_addr, size, HAL_MAX_DELAY);

  if (ret != HAL_OK)
  {
    errorFound(COM_ERROR);
  }

  uint8_t timeout = 0;
  for (timeout = 0; i2c01->State != HAL_I2C_STATE_READY && timeout < WRITE_TIMEOUT; timeout++)
  {
    //Wait for the send to stop
  }

  if (timeout > WRITE_TIMEOUT)
  {
    errorFound(COM_TIMEOUT);
  }

}

//breaks data into chunks to prevent crossing page boundary
void uploadChunk(void *from_addr, uint16_t to_addr, uint16_t size)
{
  uint16_t next_boundary = (to_addr / PAGE_SIZE + 1) * PAGE_SIZE;
  uint16_t current_addr = to_addr;
  uint16_t end_loc = to_addr + (size - 1);
  uint8_t *from = from_addr;
  uint8_t chunkSize; //number of bytes copying from mem

  do
  {
    //send from current to boundary or end loc, whichever is less
    if (end_loc - current_addr < next_boundary - current_addr)
    {
      chunkSize = end_loc - current_addr + 1;
    }
    else
    {
      chunkSize = next_boundary - current_addr;
    }

    eUploadRaw(from + (current_addr - to_addr), current_addr, chunkSize);
    HAL_Delay(5);
    current_addr += chunkSize;
    next_boundary = (current_addr / PAGE_SIZE + 1) * PAGE_SIZE;
  } while (current_addr < end_loc);
}

//transfers all values to given huart
void eepromDump(UART_HandleTypeDef huart)
{
  uint8_t MSG[PAGE_SIZE + 1] = {0};
  for (uint16_t i = 0; i < g_eeprom_size; i += PAGE_SIZE)
  {
    downloadChunk(i, MSG, PAGE_SIZE);
    HAL_UART_Transmit(&huart, MSG, sizeof(MSG) - 1, 100);
    HAL_Delay(10);
  }
}

//Sets all addresses to 0
void eepromWipe()
{
  uint8_t data[PAGE_SIZE] = {0};

  for (uint16_t i = 0; i < g_eeprom_size; i += PAGE_SIZE)
  {
    uploadChunk(data, i, 32);
  }
}

//returns null if none
header_t *findHeader(char name[])
{

  //search through headers until name match
  for (int i = 0; i < g_numStructs; i++)
  {
    if (strncmp(name, g_headers[i].name, NAME_SIZE) == 0)
    {
      return &g_headers[i];
    }
  }

  return NULL;
}

//adds Header to eeprom
void addHeaderEntry(header_t *new_header)
{

  new_header->address_on_eeprom = eepromMalloc(new_header->size);

  g_numStructs += 1;
  uploadByte(0, g_numStructs); //increment struct num by 1

  if (g_numStructs > MAX_HEADER_COUNT)
  {
    errorFound(MAX_HEADER);
  }

  uploadChunk(new_header, (g_numStructs - 1) * HEADER_SIZE + 1, HEADER_SIZE);

  sortHeaders(); //added new item, put it in place
}

//finds location of header in eeprom and updates it
void updateHeaderEntry(header_t *header)
{
  //somehow find where its located
  //current process is slower due to searching through actual eeprom mem
  char name_found[NAME_SIZE];

  //converting allows for pointer addition
  uint8_t *header_loc = (uint8_t*) header;

  for (int i = 0; i < g_numStructs; i++)
  {
    downloadChunk(i * HEADER_SIZE + 1, &name_found, NAME_SIZE);
    if (strncmp(name_found, header->name, NAME_SIZE) == 0)
    {
      //found the correct header to update
      uploadChunk(header_loc + NAME_SIZE, i * HEADER_SIZE + 1 + NAME_SIZE, HEADER_SIZE - NAME_SIZE);
      return;
    }
  }
  //header not found, should never reach this point...
  errorFound(HEADER_NOT_FOUND);
}

//links struct ptr with a header from eeprom, overwrite protect active high
//returns 1 if it was not found to currently exist
uint8_t eepromLinkStruct(void *ptr, uint16_t size, char name[], uint8_t version, uint8_t overwrite_protection)
{
  uint8_t is_new_struct = 0;

  header_t *a_header = NULL;

  a_header = findHeader(name);

  if (version > MAX_VERSION)
  {
    version = MAX_VERSION;
  }

  uint8_t overwrite_previous;
  //if node found, extract overwrite bit from version
  if (a_header != NULL)
  {
    splitVersion(&(a_header->version), &overwrite_previous);
  }

  if (overwrite_protection != 0)
  {
    overwrite_protection = 1;
  }

  if (a_header == NULL)
  {
    //struct not in eeprom in any form
    is_new_struct = 1;

    a_header = &g_headers[g_numStructs]; //0 based list, no +1
    strcpy(a_header->name, name);

    combineVersion(&version, &overwrite_protection);
    a_header->version = version;
    a_header->size = size;

    a_header->ptr_to_data = ptr; //link :D

    addHeaderEntry(a_header); //update eAddress too
    uploadChunk(a_header->ptr_to_data, a_header->address_on_eeprom, a_header->size);
  }
  else if (a_header->size != size || a_header->version != version)
  {
    //overwrite and header change

    if (spaceAvailable(a_header->address_on_eeprom) < a_header->size)
    {
      //can't place struct here, move
      a_header->address_on_eeprom = eepromMalloc(a_header->size);

      //change of address, sort g_headers
      sortHeaders();
    }

    combineVersion(&version, &overwrite_protection);
    a_header->version = version;
    a_header->size = size;

    a_header->ptr_to_data = ptr; //link :D

    updateHeaderEntry(a_header);
    uploadChunk(a_header->ptr_to_data, a_header->address_on_eeprom, a_header->size);
  }
  else if (overwrite_previous != overwrite_protection)
  {
    combineVersion(&version, &overwrite_protection);
    a_header->version = version;
    a_header->ptr_to_data = ptr; //link :D
    updateHeaderEntry(a_header);
  }
  else
  {
    //struct info matches that in eeprom
    a_header->ptr_to_data = ptr; //link :D
  }

  return is_new_struct;
}

//populate linked list with header info from eeprom
void loadHeaderEntries()
{
  for (int i = 0; i < g_numStructs; i++)
  {
    downloadChunk(i * HEADER_SIZE + 1, &g_headers[i], HEADER_SIZE);
  }
}

//sort headers by increasing eaddress
void sortHeaders()
{
  header_t temp; //temporary buffer

  for (int i = 0; i < g_numStructs; i++)
  {
    for (int j = 0; j < g_numStructs - i - 1; j++)
    {
      if (g_headers[j].address_on_eeprom > g_headers[j + 1].address_on_eeprom)
      {
        temp = g_headers[j + 1];
        g_headers[j + 1] = g_headers[j];
        g_headers[j] = temp;
      }
    }
  }
}

//returns available space to use at an address
uint16_t spaceAvailable(uint16_t address)
{
  if (address > g_eeprom_size)
  {
    return 0;
  }

  //find header with first address greater than eAddress
  for (int i = 0; i < g_numStructs; i++)
  {
    if (g_headers[i].address_on_eeprom > address)
    {
      return g_headers[i].address_on_eeprom - address;
    }
  }
  //no headers with address after said address
  return g_eeprom_size - address;
}

/*returns eeprom address with space for set size
null if not available, relies on the fact that
the linked list is sorted by increasing
eaddress*/
uint16_t eepromMalloc(uint16_t size)
{
  if (g_numStructs > 0)
  {
//    header_t *current = g_headers;

    //check between end of headers and first node
    if (g_headers->address_on_eeprom - (MAX_HEADER_COUNT * HEADER_SIZE + 1) >= size)
    {
      return MAX_HEADER_COUNT * HEADER_SIZE + 1;
    }

    //check between individual nodes
    for (int i = 0; i < g_numStructs - 1; i++)
    {
      if (g_headers[i + 1].address_on_eeprom - (g_headers[i].address_on_eeprom + g_headers[i].size) >= size)
      {
        return g_headers[i].address_on_eeprom + g_headers[i].size;
      }
    }
    //reached last entry, check is space between last and end of eeprom

    if (g_eeprom_size - g_headers[g_numStructs].address_on_eeprom + g_headers[g_numStructs].size >= size)
    {
      return g_headers[g_numStructs - 1].address_on_eeprom + g_headers[g_numStructs - 1].size;
    }

    errorFound(MAX_MEM);
    return NULL; //no space available
  }
  else
  {
    return MAX_HEADER_COUNT * HEADER_SIZE + 1;
  }
}

//remove header from eeprom
void removeFromEeprom(char name[])
{
  // This function finds the last header entry in
  // eeprom and overwrites the one to be deleted

  uint8_t header_buffer[HEADER_SIZE]; //stores last header entry in eeprom
  char name_buffer[NAME_SIZE];

  // copy last header info into header_buffer
  downloadChunk((g_numStructs - 1) * HEADER_SIZE + 1, header_buffer, HEADER_SIZE);

  // find unused header pos and overwrite
  for (int i = 0; i < g_numStructs; i++)
  {
    downloadChunk(i * HEADER_SIZE + 1, &name_buffer, NAME_SIZE);
    if (strncmp(name_buffer, name, NAME_SIZE) == 0)
    {
      // found the correct header to update
      uploadChunk(header_buffer, i * HEADER_SIZE + 1, HEADER_SIZE);
      i = g_numStructs; // exit loop
    }
  }

  // decrement num g_headers
  g_numStructs -= 1;
  uploadByte(0, g_numStructs);
}

//removes unused headers (those without a linked pointer) from eeprom
void eepromCleanHeaders()
{

  for (int i = 0; i < g_numStructs; i++)
  {
    if (g_headers[i].ptr_to_data == NULL && !(g_headers[i].version >> OVERWRITE_BIT))
    {
      //unused header and no overwrite
      //DECREMENTS G_NUM_STRUCTS
      removeFromEeprom(g_headers[i].name);

      //move headers back one to fill gap
      for (int j = i; j < g_numStructs; j++)
      {
        g_headers[j] = g_headers[j + 1]; //intended to reach 1+
      }
      i-=1; //indexes all shifted back one now
    }
  }
}

//loads current header info
void eepromInitialize(I2C_HandleTypeDef *i2c, uint16_t eepromSpace, uint8_t address)
{
  i2c01 = i2c;
  g_eeprom_size = eepromSpace;
  g_device_addr = address;

  g_numStructs = readByte(0);

  loadHeaderEntries();
  sortHeaders();

  //eepromWipe();
}

//loads struct from mem, returns 1 if unknown struct
uint8_t eepromLoadStruct(char name[])
{

  for (int i = 0; i < g_numStructs; i++)
  {
    if (strncmp(name, g_headers[i].name, NAME_SIZE) == 0)
    {
      //found desired node
      downloadChunk(g_headers[i].address_on_eeprom, g_headers[i].ptr_to_data, g_headers[i].size);
      return 0;
    }
  }

  return 1;
}

//saves struct to mem, returns 1 if unknown struct
uint8_t eepromSaveStruct(char name[])
{

  for (int i = 0; i < g_numStructs; i++)
  {
    if (strncmp(name, g_headers[i].name, NAME_SIZE) == 0)
    {
      //found desired node
      uploadChunk(g_headers[i].ptr_to_data, g_headers[i].address_on_eeprom, g_headers[i].size);
      return 0;
    }
  }

  return 1;
}

//splits version into overwrite and version
void splitVersion(uint8_t *version, uint8_t *overwrite)
{
  *overwrite = *version >> OVERWRITE_BIT;
  *version = *version & OVERWRITE_MASK;
}

//combines overwrite with version
void combineVersion(uint8_t *version, uint8_t *overwrite)
{
  *version = *version | (*overwrite << OVERWRITE_BIT);
}

void errorFound(eeprom_error_t error)
{
  switch (error)
  {
  case COM_TIMEOUT:
  case COM_ERROR:
  case MAX_HEADER:
  case MAX_MEM:
  case HEADER_NOT_FOUND:
    while (PER == GREAT)
    {
    }
    break;
  }
}
