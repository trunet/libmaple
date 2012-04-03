#include "wirish.h"

#include "i2c.h"

#include <APM_BMP085.h> // APM BMP085 Library
#include <EEPROM.h>

APM_BMP085_Class APM_BMP085;
unsigned long timer_apm_bmp085;

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

void ACC_getADC() {
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
	
	Serial2.print("X:");
	Serial2.print((buffer[1]<<8) | buffer[0]);
	Serial2.print(", ");
	Serial2.print("Y:");
	Serial2.print((buffer[3]<<8) | buffer[2]);
	Serial2.print(", ");
	Serial2.print("Z:");
	Serial2.print((buffer[5]<<8) | buffer[4]);
	Serial2.println();
	/*
	Serial2.print("X:");
	Serial2.print((uint8)(buffer[1]<<2) | buffer[0]>>6);
	Serial2.print(", ");
	Serial2.print("Y:");
	Serial2.print((uint8)(buffer[3]<<2) | buffer[2]>>6);
	Serial2.print(", ");
	Serial2.print("Z:");
	Serial2.print((uint8)(buffer[5]<<2) | buffer[4]>>6);
	Serial2.println();
	*/
}

void setup() {
	Serial2.begin(9600);
    Serial2.println("TrunetCopter v0.1");

	pinMode(BOARD_LED_PIN, OUTPUT);

	i2c_master_enable(I2C1, I2C_FAST_MODE);

	APM_BMP085.Init();   // APM BMP init
	
	ACC_init(); // APM ACC init

	/*
	EEPROM write test
	*/
	/*
	uint8 buffer[256] = {0};
	for (uint16 i=0; i<256; i++) {
		buffer[i]=i;
		writeEEPROM(i, buffer[i]);
	}
	*/
	
	delay(1000);
	
	timer_apm_bmp085 = millis();
}

void loop() {
	float tmp_float;
	float Altitude;
	
	if((millis() - timer_apm_bmp085) > 200) {
		timer_apm_bmp085 = millis();
		APM_BMP085.Read();
		Serial2.print("Pressure:");
		Serial2.print(APM_BMP085.Press);
	    Serial2.print(" Temperature:");
	    Serial2.print(APM_BMP085.Temp/10.0);
	    Serial2.print(" Altitude:");
	    tmp_float = (APM_BMP085.Press/101325.0);
	    tmp_float = pow(tmp_float,0.190295);
	    Altitude = 44330*(1.0-tmp_float);
		Serial2.print(Altitude);
		Serial2.println();
		
		ACC_getADC();
		
		/*
		EEPROM read test
		*/
		/*
		for (uint16 i=0; i<256; i++) {
			Serial2.print("|");
			Serial2.print(i);
			Serial2.print(":");
			readEEPROM(i, rdata);
			Serial2.print(rdata[0]);
			Serial2.print("|,");
		}
		Serial2.println();
		*/
		
		toggleLED();
	}
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    setup();

    while (true) {
        loop();
    }

    return 0;
}
