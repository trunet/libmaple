#ifndef EEPROM_h
#define EEPROM_h

#include "wirish.h"
#include "i2c.h"

void writeEEPROM(unsigned int address, byte data );
void readEEPROM(unsigned int address, uint8 *rdata );

#endif
