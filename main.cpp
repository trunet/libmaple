#include "wirish.h"

#include "i2c.h"

#include <APM_BMP085.h> // APM BMP085 Library
#include <BMA020.h>
#include <EEPROM.h>

APM_BMP085_Class APM_BMP085;
unsigned long timer_apm_bmp085;

void setup() {
	Serial2.begin(9600);
	Serial2.println("TrunetCopter v0.1");

	pinMode(BOARD_LED_PIN, OUTPUT);

	i2c_master_enable(I2C1, I2C_FAST_MODE);

	APM_BMP085.Init();   // BMP085 init
	
	ACC_init(); // ACC BMA020 init

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
		
		int acc_x, acc_y, acc_z;
		ACC_getADC(acc_x, acc_y, acc_z);
		Serial2.print("ACC X:");
                Serial2.print(acc_x);
		Serial2.print(", Y:");
		Serial2.print(acc_y);
		Serial2.print(", Z:");
		Serial2.print(acc_z);
		Serial2.println();
		
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
