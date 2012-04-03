#include "EEPROM.h"

void writeEEPROM(unsigned int address, byte data) {
	uint8 deviceaddress = 0x50;
	i2c_msg msgWrite1[1];
	uint8 bufWrite1[3];
	
	bufWrite1[0] = (uint8)((int)(address >> 8));
	bufWrite1[1] = (uint8)((int)(address & 0xff));
	bufWrite1[2] = data;
	
	msgWrite1[0].addr = deviceaddress;
	msgWrite1[0].flags = 0;
	msgWrite1[0].length = 3;
	msgWrite1[0].data = bufWrite1;
	
	i2c_master_xfer(I2C1, msgWrite1, 1, 0);
	delay(5);
}

void readEEPROM(unsigned int address, uint8 *rdata) {
	uint8 deviceaddress = 0x50;
	i2c_msg msgsRead[2];
	uint8 bufReadAddress[2];
	
	bufReadAddress[0] = (uint8)((int)(address >> 8));
	bufReadAddress[1] = (uint8)((int)(address & 0xff));
	
	msgsRead[0].addr = deviceaddress;
	msgsRead[0].flags = 0;
	msgsRead[0].length = 2;
	msgsRead[0].data = bufReadAddress;
	
	msgsRead[1].addr = deviceaddress;
	msgsRead[1].flags = I2C_MSG_READ;
	msgsRead[1].length = 1;
	msgsRead[1].data = rdata;
	
	i2c_master_xfer(I2C1, msgsRead, 2, 0);
	delay(5);
}
