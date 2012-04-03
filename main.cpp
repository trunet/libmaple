// Sample main.cpp file. Blinks the built-in LED, sends a message out
// USART2, and turns on PWM on pin 2.

#include "wirish.h"

#include <APM_BMP085.h> // APM BMP085 Library

APM_BMP085_Class APM_BMP085;

void setup() {
    pinMode(BOARD_LED_PIN, OUTPUT);

    Serial2.begin(9600);
    Serial2.println("TrunetCopter v0.1");

	APM_BMP085.Init();   // APM ADC initialization
}

void loop() {
	float tmp_float;
	float Altitude;
	
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
	
	toggleLED();
	delay(1000);
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
