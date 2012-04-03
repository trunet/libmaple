#include "BMA020.h"

void ACC_init() {
	i2c_msg msgacc[2];
	uint8 data[2];
	
	data[0] = 0x15; // set SPI4 bit
	data[1] = 0x80; // set SPI4 bit
	msgacc[0].addr = 0x38;
	msgacc[0].flags = 0;
	msgacc[0].length = 2;
	msgacc[0].data = data;
	i2c_master_xfer(I2C1, msgacc, 1, 0);
	delay(5);
	
	data[0] = 0x14;
	msgacc[0].addr = 0x38;
	msgacc[0].flags = 0;
	msgacc[0].length = 1;
	msgacc[0].data = data;
	msgacc[1].addr = 0x38;
	msgacc[1].flags = I2C_MSG_READ;
	msgacc[1].length = 1;
	msgacc[1].data = data;
	i2c_master_xfer(I2C1, msgacc, 2, 0);
	delay(5);
	
	uint8_t control = data[0];
	control = control & 0xE0;        // save bits 7,6,5
	control = control | (0x02 << 3); // Range 8G (10)
	control = control | 0x00;        // Bandwidth 25 Hz 000
	
	data[0] = 0x14;
	data[1] = control;
	msgacc[0].addr = 0x38;
	msgacc[0].flags = 0;
	msgacc[0].length = 2;
	msgacc[0].data = data;
	i2c_master_xfer(I2C1, msgacc, 1, 0);
	delay(5);
  
	//acc_1G = 63;
}

void ACC_getADC(int &acc_x, int &acc_y, int &acc_z) {
	i2c_msg msgacc[2];
	uint8 buffer[6], data[2];
	
	data[0] = 0x02;
	msgacc[0].addr = 0x38;
	msgacc[0].flags = 0;
	msgacc[0].length = 1;
	msgacc[0].data = data;
	msgacc[1].addr = 0x38;
	msgacc[1].flags = I2C_MSG_READ;
	msgacc[1].length = 6;
	msgacc[1].data = buffer;
	i2c_master_xfer(I2C1, msgacc, 2, 0);
	delay(5);

	acc_x = (int)((buffer[1]<<2) | buffer[0]>>6);
	acc_y = (int)((buffer[3]<<2) | buffer[2]>>6);
	acc_z = (int)((buffer[5]<<2) | buffer[4]>>6);
#ifdef SERIAL_DEBUG
	Serial2.print("X:");
	Serial2.print((int)((buffer[1]<<2) | buffer[0]>>6));
	Serial2.print(", ");
	Serial2.print("Y:");
	Serial2.print((int)((buffer[3]<<2) | buffer[2]>>6));
	Serial2.print(", ");
	Serial2.print("Z:");
	Serial2.print((int)((buffer[5]<<2) | buffer[4]>>6));
	Serial2.println();
#endif
}
