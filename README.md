# Eeprom24AA32AF
EEPROM memory management for a STM Nucleo Board

Designed to save values of structs for data tracking. Each struct is stored with a corresponding name, version, and size. If a struct is added with a same name, but different version, the existing struct in memory will be overwritten. If the new struct is too large for the memory allocated, more space will be made to fit the larger struct.

Includes the dumping of all eeprom contents through UART. The uart is connected through an Arduino's tx and rx ports. Data is received and logged through PUTTY at 115200 bits/s. Python script within logs converts this data to a table with each address mapped to its value. Currently only maps 8 bit values.