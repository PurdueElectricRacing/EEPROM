#include "common/eeprom/eeprom.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

header_t g_headers[MAX_HEADER_COUNT] = {0};
eeprom_write_status_t e_wr_stat = {.has_write_started=0, .is_write_complete=0};

uint8_t g_numStructs; //number of entries in header

uint16_t g_eeprom_size;
uint8_t g_device_addr;
I2C_TypeDef *g_eeprom_i2c;

void downloadChunk(uint16_t from_addr, void *to_addr, uint16_t size);
void uploadByte(uint16_t addr, uint8_t val);
void uploadChunkBlocking(void *from_addr, uint16_t to_addr, uint16_t size);
uint8_t uploadChunkPeriodic(void *from_addr, uint16_t to_addr, uint16_t size);
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
void delay(uint8_t ms);


//reads chunk of data
void downloadChunk(uint16_t from_addr, void *to_addr, uint16_t size)
{
  uint8_t ret = 0;

  // set cursor
  ret = PHAL_I2C_gen_start(g_eeprom_i2c, SET_ADDRESS(g_device_addr, WRITE_ENABLE), 2, PHAL_I2C_MODE_TX);
  if (!ret) errorFound(COM_ERROR);
  ret = PHAL_I2C_write(g_eeprom_i2c, from_addr >> 8);   // High
  if (!ret) errorFound(COM_ERROR);
  ret = PHAL_I2C_write(g_eeprom_i2c, from_addr & 0xFF); // Low
  if (!ret) errorFound(COM_ERROR);
  ret = PHAL_I2C_gen_stop(g_eeprom_i2c);
  if (!ret) errorFound(COM_ERROR);

  while(size > 0xFF) // can only receive 255 at a time
  {
    ret = PHAL_I2C_gen_start(g_eeprom_i2c, SET_ADDRESS(g_device_addr, READ_ENABLE), 0xFF, PHAL_I2C_MODE_RX);
    if (!ret) errorFound(COM_ERROR);
    ret = PHAL_I2C_read_multi(g_eeprom_i2c, to_addr, 0xFF);
    if (!ret) errorFound(COM_ERROR);
    ret = PHAL_I2C_gen_stop(g_eeprom_i2c);
    if (!ret) errorFound(COM_ERROR);
    size -= 0xFF;
    to_addr += 0xFF;
  }

  ret = PHAL_I2C_gen_start(g_eeprom_i2c, SET_ADDRESS(g_device_addr, READ_ENABLE), (uint8_t) size, PHAL_I2C_MODE_RX);
  if (!ret) errorFound(COM_ERROR);
  ret = PHAL_I2C_read_multi(g_eeprom_i2c, to_addr, (uint8_t) size);
  if (!ret) errorFound(COM_ERROR);
  ret = PHAL_I2C_gen_stop(g_eeprom_i2c);
  if (!ret) errorFound(COM_ERROR);

}

//writes single byte
void uploadByte(uint16_t addr, uint8_t val)
{
  uint8_t ret = 0;
  ret = PHAL_I2C_gen_start(g_eeprom_i2c, SET_ADDRESS(g_device_addr, WRITE_ENABLE), 3, PHAL_I2C_MODE_TX);
  if (!ret) errorFound(COM_ERROR);
  ret = PHAL_I2C_write(g_eeprom_i2c, addr >> 8);   // High Addr
  if (!ret) errorFound(COM_ERROR);
  ret = PHAL_I2C_write(g_eeprom_i2c, addr & 0xFF); // Low Addr
  if (!ret) errorFound(COM_ERROR);
  ret = PHAL_I2C_write(g_eeprom_i2c, val);         // Data
  if (!ret) errorFound(COM_ERROR);
  ret = PHAL_I2C_gen_stop(g_eeprom_i2c);

}

//uploads chunk ignoring page breaks
void eUploadRaw(void *from_addr, uint16_t to_addr, uint16_t size)
{
  uint8_t ret = 0;

  ret = PHAL_I2C_gen_start(g_eeprom_i2c, SET_ADDRESS(g_device_addr, WRITE_ENABLE), 2 + size, PHAL_I2C_MODE_TX);
  if (!ret) errorFound(COM_ERROR);
  ret = PHAL_I2C_write(g_eeprom_i2c, to_addr >> 8);          // High Addr
  if (!ret) errorFound(COM_ERROR);
  ret = PHAL_I2C_write(g_eeprom_i2c, to_addr & 0xFF);        // Low Addr
  if (!ret) errorFound(COM_ERROR);
  ret = PHAL_I2C_write_multi(g_eeprom_i2c, from_addr, size); // Data
  if (!ret) errorFound(COM_ERROR);
  ret = PHAL_I2C_gen_stop(g_eeprom_i2c);
  if (!ret) errorFound(COM_ERROR);

}

//breaks data into chunks to prevent crossing page boundary
void uploadChunkBlocking(void *from_addr, uint16_t to_addr, uint16_t size)
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

    delay(E_DELAY);
    eUploadRaw(from + (current_addr - to_addr), current_addr, chunkSize);

    current_addr += chunkSize;
    next_boundary = (current_addr / PAGE_SIZE + 1) * PAGE_SIZE;
  } while (current_addr < end_loc);
}

//breaks data into chunks to prevent crossing page boundary
uint8_t uploadChunkPeriodic(void *from_addr, uint16_t to_addr, uint16_t size)
{
  if (!e_wr_stat.has_write_started)
  {
    e_wr_stat.has_write_started = 1;
    e_wr_stat.is_write_complete = 0;
    e_wr_stat.next_boundary = (to_addr / PAGE_SIZE + 1) * PAGE_SIZE;
    e_wr_stat.current_addr = to_addr;
    e_wr_stat.end_loc = to_addr + (size - 1);
    e_wr_stat.from = from_addr;
  }

  uint8_t chunkSize; //number of bytes copying from mem

  //send from current to boundary or end loc, whichever is less
  if (e_wr_stat.end_loc - e_wr_stat.current_addr < 
      e_wr_stat.next_boundary - e_wr_stat.current_addr)
  {
    chunkSize = e_wr_stat.end_loc - e_wr_stat.current_addr + 1;
  }
  else
  {
    chunkSize = e_wr_stat.next_boundary - e_wr_stat.current_addr;
  }

  eUploadRaw(e_wr_stat.from + (e_wr_stat.current_addr - to_addr), e_wr_stat.current_addr, chunkSize);

  e_wr_stat.current_addr += chunkSize;
  e_wr_stat.next_boundary = (e_wr_stat.current_addr / PAGE_SIZE + 1) * PAGE_SIZE;

  if (e_wr_stat.current_addr < e_wr_stat.end_loc)
  {
    return false;
  }
  else
  {
    e_wr_stat.is_write_complete = 1;
    e_wr_stat.has_write_started = 0;
    return true;
  }
}

//transfers all values to given huart
// TODO: PHAL UART Library
// void eepromDump(UART_HandleTypeDef huart)
// {
//   uint8_t MSG[PAGE_SIZE + 1] = {0};
//   for (uint16_t i = 0; i < g_eeprom_size; i += PAGE_SIZE)
//   {
//     downloadChunk(i, MSG, PAGE_SIZE);
//     //HAL_UART_Transmit(&huart, MSG, sizeof(MSG) - 1, 100);
//     HAL_Delay(10);
//   }
// }

//Sets all addresses to 0
void eepromWipe()
{
  uint8_t data[PAGE_SIZE] = {0};

  for (uint16_t i = 0; i < g_eeprom_size; i += PAGE_SIZE)
  {
    uploadChunkBlocking(data, i, 32);
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
  delay(E_DELAY); // minimum 5 ms between writing
  uploadByte(0, g_numStructs); //increment struct num by 1

  if (g_numStructs > MAX_HEADER_COUNT)
  {
    errorFound(MAX_HEADER);
  }

  uploadChunkBlocking(new_header, (g_numStructs - 1) * HEADER_SIZE + 1, HEADER_SIZE);

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
      uploadChunkBlocking(header_loc + NAME_SIZE, i * HEADER_SIZE + 1 + NAME_SIZE, HEADER_SIZE - NAME_SIZE);
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
    uploadChunkBlocking(a_header->ptr_to_data, a_header->address_on_eeprom, a_header->size);
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
    uploadChunkBlocking(a_header->ptr_to_data, a_header->address_on_eeprom, a_header->size);
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
    return 0; //no space available
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
      uploadChunkBlocking(header_buffer, i * HEADER_SIZE + 1, HEADER_SIZE);
      i = g_numStructs; // exit loop
    }
  }

  // decrement num g_headers
  g_numStructs -= 1;
  delay(E_DELAY); // minimum 5 ms delay between write cycles
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
void eepromInitialize(uint16_t eepromSpace, uint8_t address, I2C_TypeDef *i2c)
{
  g_eeprom_size = eepromSpace;
  g_device_addr = address;
  g_eeprom_i2c = i2c;

  downloadChunk(0x00, &g_numStructs, 1);

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
uint8_t eepromSaveStructBlocking(char name[])
{

  for (int i = 0; i < g_numStructs; i++)
  {
    if (strncmp(name, g_headers[i].name, NAME_SIZE) == 0)
    {
      //found desired node
      uploadChunkBlocking(g_headers[i].ptr_to_data, g_headers[i].address_on_eeprom, g_headers[i].size);
      return 0;
    }
  }

  return 1;
}

//saves struct to mem, returns 1 if unknown struct, call until eeprom is_write_complete = 1
uint8_t eepromSaveStructPeriodic(char name[])
{

  for (int i = 0; i < g_numStructs; i++)
  {
    if (strncmp(name, g_headers[i].name, NAME_SIZE) == 0)
    {
      //found desired node
      uploadChunkPeriodic(g_headers[i].ptr_to_data, g_headers[i].address_on_eeprom, g_headers[i].size);
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

void delay(uint8_t ms)
{
  uint32_t ticks = (SystemCoreClock / 1000) * ms / 4;
  for(int i = 0; i < ticks; i++)
  {
    __asm__("nop");
  }
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
    while (1)
    {
      __asm__("nop");
    }
    break;
  }
}
