#include "wirish.h"

#include "i2c.h"

#include <APM_BMP085.h> // APM BMP085 Library
#include <BMA020.h>
#include <EEPROM.h>

APM_BMP085_Class APM_BMP085;
unsigned long timer_apm_bmp085;

//TXRX
#define PPM_SUM 18

#define CHANNELS 6
#define MINCHECK 900
#define MAXCHECK 2100

//Channel mapping
// 1 = Aileron
// 2 = Profundor
// 3 = Throttle
// 4 = Rudder
// 5 = Gear
// 6 = Flap

//static uint8_t rcChannel[12] = {PPM_ORDER};
volatile uint16_t rcValue[CHANNELS] = {1500,1500,1500,1500,1500,1500}; // interval [1000;2000]
volatile uint16_t rcTmpValue[CHANNELS] = {1500,1500,1500,1500,1500,1500}; // interval [1000;2000]

volatile unsigned char radio_status_rc=0;
volatile unsigned char sync=0;
volatile unsigned int currentChannel = 0;
static unsigned int last = 0;

unsigned int uiRcErrCnt1 = 0;

typedef struct {
  byte edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int  lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[9];

void rxIntPPMSUM(void) {
	volatile unsigned int now;
	volatile unsigned int diff;
	int i;

	now = micros();
	diff = now - last;
	last = now;
	if (diff > 4000 && diff < 21000) { // Sincro del frame
		currentChannel = 0;
		radio_status_rc = 0;
		if (uiRcErrCnt1 == 0) { // if the frame is error free, copy it to rcValue Array
			for (i=0;i<CHANNELS;i++) {
				rcValue[i] = rcTmpValue[i]; // THE PPMSUM VALUE START FROM 10 ' STANDARD PPM channel < 10
			}
		}
		sync=1;
		uiRcErrCnt1=0; // Reset Error counter
	} else {
		if ((diff>2200) || (diff<550)) { // the signal from my jeti receiver goes around 740 to 1550 ms, with <650 or >2000 bad data will be recorded
			uiRcErrCnt1++;
		}
	}
	
	if (sync==1) {
		if (currentChannel > 0) {
			rcTmpValue[currentChannel-1] = diff;
			if (diff<=MAXCHECK && diff>=MINCHECK) radio_status_rc++;
		}
		currentChannel++;
	}
	
	if (currentChannel>CHANNELS) {
		sync=0;
		radio_status_rc=0;
	}
}

uint16_t inputCh(unsigned char ch) {
	uint16_t data;
  	data = rcValue[ch];
	return data; // We return the value correctly copied when the IRQ's where disabled
}

void setup() {
	Serial2.begin(9600);
	Serial2.println("TrunetCopter v0.1");

	pinMode(BOARD_LED_PIN, OUTPUT);

	//TXRX
	pinMode(PPM_SUM, INPUT);
	attachInterrupt(PPM_SUM, rxIntPPMSUM, RISING);

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
	
	if((millis() - timer_apm_bmp085) > 500) {
		Serial2.println("==================================");
		Serial2.print("CH1:");
		Serial2.print(inputCh(0));
		Serial2.print(", CH2:");
		Serial2.print(inputCh(1));
		Serial2.print(", CH3:");
		Serial2.print(inputCh(2));
		Serial2.print(", CH4:");
		Serial2.print(inputCh(3));
		Serial2.print(", CH5:");
		Serial2.print(inputCh(4));
		Serial2.print(", CH6:");
		Serial2.print(inputCh(5));
		Serial2.println();
		
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
