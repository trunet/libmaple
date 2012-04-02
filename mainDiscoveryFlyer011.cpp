/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 ridgebackred.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *****************************************************************************/
/******************************************************************************
DiscoveryFlyer011.cpp  This one is a keeper.
                       Current development version

Flown: Gyro mode is way good with short props.

       Height hold works good, droops w/ battery. Throttle still active.
       Will keep this as default behavior for height hold for safety reasons.

       Reintroducing auto-level...
       Try Fabio's 6dof routines again. Works well with smoothing.

Not released: 9DOF test bed, Current Development - Flys excellent in gyro only

A libmaple program to fly a tri-copter.
http://leaflabs.com/docs/libmaple.html

Uses Anton's libmaple branch for Discovery.
https://github.com/anton19286/libmaple

Thanks to Mike Barton for the KKmulticopter AVR assembly to C port.
XXcontroller was used as a known good reference for my initial STM32 port.

For the STM32VLDISCOVERY evaluation board.         (USD $11.85 Mouser)
RX is Turnigy 9X 2.4GHz 8Ch Receiver (V2)          (USD  $8.99 HobbyKing)
9DOF sensor stick, Atmel ATAVRSBIN1                (USD $54.00 Mouser)
Altimeter is MS5611-01BA (10cm resolution)         (USD  $9.60 ServoFlo)

Created  May 17, 2011                                By ridgebackred
******************************************************************************/
#include "wirish.h"
#include "i2c.h"
#include <C:\libmaple\discovery3\libraries\Servo\Servo.h>

//Uncomment the next line for serial debugging output
//#define Use_SERIAL     TRUE

 // Declare the interrupt handler routines
void handler_CH1 (void);
void handler_CH2 (void);
void handler_CH3 (void);
void handler_CH4 (void);
void handler_CH5 (void);

 // Declare called routines
void InitI2C2(void);
void InitITG3200(void);
void ReadITG3200(uint8 *buf);
void CalibrateGyro(void);
void InitBMA150(void);
void ReadBMA150(uint8 *buf);
void CalibrateAccel(void);
void InitAK8975(void);
void StartConversionAK8975(void);
void ReadAK8975(uint8 *buf);
void InitMS5611(void);
void ReadMS5611_Prom(void);
void StartMS5611_mbar(void);
void ReadMS5611_mbar(void);
void StartMS5611_temp(void);
void ReadMS5611_temp(void);
void serialPrintFloatArr(float * arr, int length);
void serialFloatPrint(float f);
void rawGyroToDegsec(int * raw, float * gyro_ds);
void getInclination(void);
void rawAccToG(int * raw, float * RwAcc);
void normalize3DVec(float * vector);

 // Create servo object to control the yaw servo
Servo yawservo;

 //I2C2, uses pin 29 SCL (PB10) and pin 30 SDA (PB11)
 //All of the sensors are on the i2c bus at 400khz.

 // I/O pin assignments
const int Chan1Pin = 3;         // PA0 Input from RC receiver Pitch
const int Chan2Pin = 2;         // PA1 Input from RC receiver Roll
const int Chan3Pin = 1;         // PA2 Input from RC receiver Throttle
const int Chan4Pin = 0;         // PA3 Input from RC receiver Yaw
const int Chan5Pin = 10;        // PA4 Input from RC receiver Chan5 (gear)
const int Esc1Pin = 5;          // PB6 Timer4,1 Front left motor, ESC #1
const int Esc2Pin = 9;          // PB7 Timer4,2 Front right motor, ESC #2
const int Esc3Pin = 14;         // PB8 Timer4,3 Rear motor ESC, #3
const int YawServoPin = 6;      // PA8 Output for yaw servo
const int BlueLedPin = 37;      // PC8
const int GreenLedPin = 38;     // PC9
const int RedGrnNavLedPin = 35; // PC6
const int WhtNavLedPin = 36;    // PC7
const int LandingLedPin = 34;   // PB15 5.6 amps of hi brightness leds

 // Global variables for use in interrupt routines
volatile unsigned int Chan1begin = 0;
volatile unsigned int Chan1end = 0;
volatile unsigned int RxInPitch = 0;
volatile unsigned int Chan1prev = 0;
volatile unsigned int Chan2begin = 0;
volatile unsigned int Chan2end = 0;
volatile unsigned int RxInRoll = 0;
volatile unsigned int Chan2prev = 0;
volatile unsigned int Chan3begin = 0;
volatile unsigned int Chan3end = 0;
volatile unsigned int RxInThrottle = 0;
volatile unsigned int Chan3prev = 0;
volatile unsigned int Chan4begin = 0;
volatile unsigned int Chan4end = 0;
volatile unsigned int RxInYaw = 0;
volatile unsigned int Chan4prev = 0;
volatile unsigned int Chan5begin = 0;
volatile unsigned int Chan5end = 0;
volatile unsigned int RxInChan5 = 0;
volatile unsigned int Chan5prev = 0;

// Global variables
// This group is adjusted according to flying style and taste.
int PitchGyroScale = 42;        // 31,37,46,41 ITG3200 value for gyro short prop
int RollGyroScale = 42;         // 30,36,46,41 ITG3200 value gyro short prop
int YawGyroScale = 20;          // 20,22,24,20 ITG3200 value gyro short prop
//int PitchGyroScale = 27;        // 20,24,25,26,27 murata value for gyro long prop
//int RollGyroScale = 26;         // 20,24,25,26 murata value for gyro long prop
//int YawGyroScale = 20;          // 19,20 murata value for gyro long prop
int PitchStickScale = 4;        // 4,5 sensitivity multiplier value for stick
int RollStickScale = 4;         // 4,5 sensitivity multiplier value for stick
int YawStickScale = 5;          // sensitivity multiplier value for stick
int ThrottleStickScale = 12.5;  // sensitivity multiplier value for stick
int ThrottleHoverScale = 1;     // sensitivity multiplier value for altimeter

// No adjustment needed for these.
int PitchStickZero = 0;         // center position stick reading
int RollStickZero = 0;          // center position stick reading
int YawStickZero = 0;           // center position stick reading
float PitchGyroValue = 0;
float RollGyroValue = 0;
float YawGyroValue = 0;
float PitchGyroZero = 0;        // no motion zero offset value for gyro
float RollGyroZero = 0;         // no motion zero offset value for gyro
float YawGyroZero = 0;          // no motion zero offset value for gyro
float X_AccelValue = 0;         // value from from accelerometer sensor
float Y_AccelValue = 0;         // value from from accelerometer sensor
float Z_AccelValue = 0;         // value from from accelerometer sensor
int X_AccelValuelsb = 0;        // for wrangling twos compliment from BMA180
int X_AccelValuemsb = 0;        // for wrangling twos compliment from BMA180
int Y_AccelValuelsb = 0;        // for wrangling twos compliment from BMA180
int Y_AccelValuemsb = 0;        // for wrangling twos compliment from BMA180
int Z_AccelValuelsb = 0;        // for wrangling twos compliment from BMA180
int Z_AccelValuemsb = 0;        // for wrangling twos compliment from BMA180
float X_AccelSmoothValue = 1;            //
float Y_AccelSmoothValue = 1;            //
float Z_AccelSmoothValue = 1;            //
float HoverPitchHold = 0;       // Captured hover FPV when entering FPV mode
float HoverRollHold = 0;        // Captured hover attitude when entering FPV mode
float HoverHeadingHold =0;
int HoverPitchGyroScale = 0;    // Sensitivity multiplier value for gyro
int HoverRollGyroScale = 0;     // Sensitivity multiplier value for gyro

int X_MagValuelsb = 0;
int X_MagValuemsb = 0;
int Y_MagValuelsb = 0;
int Y_MagValuemsb = 0;
int Z_MagValuelsb = 0;
int Z_MagValuemsb = 0;
float X_MagValue = 0;
float Y_MagValue = 0;
float Z_MagValue = 0;
float X_MagValueAdj = 0;
float Y_MagValueAdj = 0;
float Z_MagValueAdj = 0;
int X_MagZero = 0;
int Y_MagZero = 0;
int Z_MagZero = 0;
int X_MagSensAdj = 0;           // Sensitivity adjustment value from factory
int Y_MagSensAdj = 0;           // Sensitivity adjustment value from factory
int Z_MagSensAdj = 0;           // Sensitivity adjustment value from factory
int Heading = 0;

unsigned int MagTimerStart = 0; // Nonblocking delay for Mag reads
unsigned int MagTimerEnd = 0;
unsigned int MagTimerNow = 0;
int MagTimerActiveFlag = false;

unsigned int MbarTimerStart = 0; // Nonblocking delay for mbar read
unsigned int MbarTimerEnd = 0;
unsigned int MbarTimerNow = 0;
int MbarTimerActiveFlag = false;
int MbarReadDoneFlag = true;

unsigned int TemperatureTimerStart = 0; // Nonblocking delay for temperature read
unsigned int TemperatureTimerEnd = 0;
unsigned int TemperatureTimerNow = 0;
int TemperatureTimerActiveFlag = false;
int TemperatureReadDoneFlag = false;

int LeftMotorOut = 0;           // value sent to left front ESC
int RightMotorOut = 0;          // value sent to right front ESC
int RearMotorOut = 0;           // value sent to rear ESC
int YawServoOut = 0;            // value sent to yaw servo
int RxInPitchHold = 0;
int RxInRollHold = 0;
int RxInYawHold = 0;
int RxInThrottleHold = 0;

float DD1 = 0;  // ADC value of the pressure conversion
float DD2 = 0;  // ADC value of the temperature conversion
int C[6];       // calibration coefficients
float Mbar;     // compensated pressure value
float T;        // compensated temperature value
float dT;       // difference between actual and measured temperature
float OFF;      // offset at actual temperature
float SENS;     // sensitivity at actual temperature
float HeightAdjust = 0;
float HoverHeightHold = 0;

bool FPV_ModeFlag = false;
bool FPV_ModeLatch = false;
bool NavLightsFlag = false;
bool LandingLightsFlag = false;
int loopcount = 0;
int navlightcount1 = 0;
int navlightcount2 = 0;

boolean firstSample = true;
int lastTime = 0;
int interval = 0;
float wGyro = 10.0;

float RwAcc[3];    //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer
float Gyro_ds[3];  //Gyro readings, in degrees per second scale
float RwGyro[3];   //Rw obtained from last estimated value and gyro movement
float Awz[2];      //angles between projection of R on XZ/YZ plane and Z axis (deg)
float RwEst[3];    //pitch roll and yaw attitude fused sensor values
float RwEstSmoothPitch = 0;
float RwEstSmoothRoll = 0;


 // Do housekeeping activities once at power-up********************************
void setup()
{
   #ifdef Use_SERIAL
    Serial1.begin(115200);
    Serial1.println("DiscoveryFlyer011");
   #endif
   //pinMode(BOARD_LED_PIN, OUTPUT);

    // Set up the RC receiver pins as inputs
   pinMode(Chan1Pin, INPUT);
   pinMode(Chan2Pin, INPUT);
   pinMode(Chan3Pin, INPUT);
   pinMode(Chan4Pin, INPUT);
   pinMode(Chan5Pin, INPUT);
   //pinMode(Chan6Pin, INPUT);

    // Set up the ESC pins as PWM outputs on Timer4
   pinMode(Esc1Pin, PWM);
   pinMode(Esc2Pin, PWM);
   pinMode(Esc3Pin, PWM);
   digitalWrite(Esc1Pin, LOW);
   digitalWrite(Esc2Pin, LOW);
   digitalWrite(Esc3Pin, LOW);

    // Set up built-in led pins as digital outputs
   pinMode(BlueLedPin, OUTPUT);
   pinMode(GreenLedPin, OUTPUT);
   digitalWrite(BlueLedPin, LOW);
   digitalWrite(GreenLedPin, LOW);

    // Set up navigation light led pins as digital outputs
   pinMode(RedGrnNavLedPin, OUTPUT_OPEN_DRAIN);
   pinMode(WhtNavLedPin, OUTPUT_OPEN_DRAIN);
   pinMode(LandingLedPin, OUTPUT_OPEN_DRAIN);
   digitalWrite(RedGrnNavLedPin, LOW);
   digitalWrite(WhtNavLedPin, LOW);
   digitalWrite(LandingLedPin, LOW);

    // Set up the yaw output pin as a servo. All 4 channels of (implied) Timer1
    // by virtue of the attached Pin, are now on the required 20mS
    // hobby RC Servo period for the PWM signal.
    // In other words, use separate timers for esc's and regular servos.
   yawservo.attach(YawServoPin);

    // Attach the RC receiver pins to their interrupt handlers
   attachInterrupt(Chan1Pin, handler_CH1, CHANGE);
   attachInterrupt(Chan2Pin, handler_CH2, CHANGE);
   attachInterrupt(Chan3Pin, handler_CH3, CHANGE);
   attachInterrupt(Chan4Pin, handler_CH4, CHANGE);
   attachInterrupt(Chan5Pin, handler_CH5, CHANGE);

    delay(600);                 // Time to accumulate a few interrupts.

   rx_check_loop:               // Infinite loop if no rx

   if((RxInPitch > 5) &&        // Check if reading rx...
      (RxInRoll > 5) &&
      (RxInYaw > 5)){
     digitalWrite(GreenLedPin, HIGH);
   }
   else{                        // No read?, try to 'reset'
     Chan1begin=0;
     Chan1end = 0;
     Chan2begin=0;
     Chan2end = 0;
     Chan3begin=0;
     Chan3end = 0;
     Chan4begin=0;
     Chan4end = 0;
     delay(1000);               // Time to accumulate a few interrupts.
     goto rx_check_loop;        // Don't continue if no rx
    }

   InitI2C2();                  // Initialize i2c hardware bus

   InitITG3200();               // Initialize the gyros
   InitBMA150();                // Initialize the accelerometer
   InitAK8975();                // Initialize the compass
   ReadMS5611_Prom();           // Read the pressure sensor offsets

    // Accurate reading if gyros are motionless here... :-)
   CalibrateGyro();

    // Accurate reading if sticks are not being moved here... :-)
   PitchStickZero    = RxInPitch;     // center position stick read
   RollStickZero     = RxInRoll;      // center position stick read
   YawStickZero      = RxInYaw;       // center position stick read

    // Setup pwm Timers for the ESC's after known good rx read
   timer_set_mode(TIMER4, TIMER_CH1, TIMER_PWM);
   timer_set_mode(TIMER4, TIMER_CH2, TIMER_PWM);
   timer_set_mode(TIMER4, TIMER_CH3, TIMER_PWM);
   timer_pause(TIMER4);
   timer_set_count(TIMER4, 0);
   timer_set_reload(TIMER4, 54000);// 60000, 55000, 50000 Period of the PWM signal
                                   // 48000 falls off the cliff w/ Plush 25Amp
                                   // 50000 spotty w/ 1 out of 3 esc's
   timer_resume(TIMER4);           // enable outputs to esc's
}

// Run continuously after running setup()*************************************
void loop() {
  uint8 buffer[6];
  int acc[3];
  int gyro[3];

    // Set flight mode to Sport or FPV with CH5 TX switch
   if(RxInChan5 > 1500){
     FPV_ModeFlag = true;
   }
   else{
     FPV_ModeFlag = false;
   }

   // Nav lights section, bump the roll stick full left or right to activate
   if((RxInThrottle < 1250) && (RxInRoll > 1850) && (NavLightsFlag == false)){
	  NavLightsFlag = true;
   }
   if((RxInThrottle < 1250) && (RxInRoll < 1100) && (NavLightsFlag == true)){
	  NavLightsFlag = false;
   }

   if(NavLightsFlag == true){
	 navlightcount1++;
	 navlightcount2++;
	 digitalWrite(RedGrnNavLedPin, HIGH);
	 if(navlightcount1 < 780) digitalWrite(WhtNavLedPin, HIGH);
	 if((navlightcount1 > 780) && (navlightcount2 < 790))digitalWrite(WhtNavLedPin, LOW);
	 if((navlightcount1 > 790) && (navlightcount2 < 820))digitalWrite(WhtNavLedPin, HIGH);
	 if((navlightcount1 > 820) && (navlightcount2 < 830))digitalWrite(WhtNavLedPin, LOW);
	 if(navlightcount2 > 830){navlightcount1 = 0; navlightcount2 = 0;}
   }
   else{
	 digitalWrite(RedGrnNavLedPin, LOW);
	 digitalWrite(WhtNavLedPin, LOW);
     digitalWrite(LandingLedPin, LOW);
   }

   // Landing lights section. Operate only when nav lights active, throttle ignored
   if((RxInRoll > 1850) && (NavLightsFlag == true)){
	   digitalWrite(LandingLedPin, HIGH);
   }
   if((RxInRoll < 1100) && (NavLightsFlag == true)){
	   digitalWrite(LandingLedPin, LOW);
   }

	// Stick values are "automatically" updated by interrupt handlers
	// Hold them to prevent unwanted value changes in the control loop
   RxInPitchHold    = RxInPitch;
   RxInRollHold     = RxInRoll;
   RxInThrottleHold = RxInThrottle;
   RxInYawHold      = RxInYaw;

   // Offset stick values to a +- value. Stick center = 0
   RxInPitchHold -= PitchStickZero;
   RxInRollHold  -= RollStickZero;
   RxInYawHold   -= YawStickZero;

   // Scale the stick values to increase/decrease stick sensitivity
   RxInPitchHold *= PitchStickScale;
   RxInRollHold  *= RollStickScale;
   RxInYawHold   *= YawStickScale;

   // Read, zero offset and scale the  gyro sensor values
   ReadITG3200(buffer);

   PitchGyroValue = ((buffer[0] << 8) | buffer[1]);
   if (PitchGyroValue > 32767) PitchGyroValue = PitchGyroValue - 65535;
   PitchGyroValue = PitchGyroValue - PitchGyroZero;
   PitchGyroValue = PitchGyroValue / 14.375; //11,13
                                          // 16 long prop

   RollGyroValue = ((buffer[2] << 8) | buffer[3]);
   if (RollGyroValue > 32767) RollGyroValue = RollGyroValue - 65535;
   RollGyroValue = RollGyroValue - RollGyroZero;
   RollGyroValue = RollGyroValue / 14.375;       // degrees per second

   YawGyroValue = ((buffer[4] << 8) | buffer[5]);
   if (YawGyroValue > 32767) YawGyroValue = YawGyroValue - 65535;
   YawGyroValue = YawGyroValue - YawGyroZero;
   YawGyroValue = YawGyroValue / 14.375;       // degrees per second

    // Gyro reverse area
   //PitchGyroValue  = -PitchGyroValue;  // Uncomment to reverse gyro output
   //RollGyroValue   = -RollGyroValue;   // Uncomment to reverse gyro output
   YawGyroValue = -YawGyroValue;         // Uncomment to reverse gyro output

    // Store offset gyro values for possible use in FPV mode before scaling
   gyro[0] = PitchGyroValue;
   gyro[1] = RollGyroValue;
   gyro[2] = YawGyroValue;

    // Read the  accelerometer sensor values
   ReadBMA150(buffer);

   X_AccelValuelsb = buffer[0] >> 6;
   X_AccelValuemsb = buffer[1] << 2;
   X_AccelValue = X_AccelValuelsb + X_AccelValuemsb;
   if (X_AccelValue > 511){ X_AccelValue = X_AccelValue - 1023;}

   Y_AccelValuelsb = buffer[2] >> 6;
   Y_AccelValuemsb = buffer[3] << 2;
   Y_AccelValue = Y_AccelValuelsb + Y_AccelValuemsb;
   if (Y_AccelValue > 511){ Y_AccelValue = Y_AccelValue - 1023;}

   Z_AccelValuelsb = buffer[4] >> 6;
   Z_AccelValuemsb = buffer[5] << 2;
   Z_AccelValue = Z_AccelValuelsb + Z_AccelValuemsb;
   if (Z_AccelValue > 511){ Z_AccelValue = Z_AccelValue - 1023;}

   X_AccelSmoothValue =(X_AccelSmoothValue * 24 + X_AccelValue) / 25;
   Y_AccelSmoothValue =(Y_AccelSmoothValue * 24 + Y_AccelValue) / 25;
   Z_AccelSmoothValue =(Z_AccelSmoothValue * 24 + Z_AccelValue) / 25;

    // Store accel values for possible use in FPV mode
   acc[1] = X_AccelSmoothValue;
   acc[0] = Y_AccelSmoothValue;
   acc[2] = Z_AccelSmoothValue;

    // 6DOF sensor fusion routines for FPV mode use
   rawAccToG(acc, RwAcc);
   normalize3DVec(RwAcc);
   rawGyroToDegsec(gyro, Gyro_ds);
   getInclination();

    // The magnetometer needs ~6mS for a conversion.
    // Nonblocking delay between conversion start and read.
    // Continue to read gyros and accel. Continue maths and esc updates.
   if(MagTimerActiveFlag == false){
    MagTimerActiveFlag	= true;
    StartConversionAK8975();
    MagTimerStart = millis();
    MagTimerEnd = MagTimerStart + 10;
   }

   MagTimerNow = millis();
   if ((MagTimerActiveFlag == true) && (MagTimerNow >= MagTimerEnd)){
    ReadAK8975(buffer);

    /*
    Adjusting the flux value with the sensitivity adjustment value should be
    done via the following formula:
    Hadj = H * ( ( ( (ASA-128)*0.5 ) / 128 ) + 1 )
    where H is the raw value, ASA is the sensitivity adjustment, and Hadj
    is the resultant adjusted value.
    */
    X_MagValuelsb = buffer[0];
    X_MagValuemsb = buffer[1] << 8;
    X_MagValue = X_MagValuelsb + X_MagValuemsb;
    if (X_MagValue > 32767){ X_MagValue = X_MagValue - 65535;}
    X_MagValueAdj = X_MagValue * ((((X_MagSensAdj-128)*0.5 ) / 128 ) + 1 );

    Y_MagValuelsb = buffer[2];
    Y_MagValuemsb = buffer[3] << 8;
    Y_MagValue = Y_MagValuelsb + Y_MagValuemsb;
    if (Y_MagValue > 32767){ Y_MagValue = Y_MagValue - 65535;}
    Y_MagValueAdj = Y_MagValue * ((((Y_MagSensAdj-128)*0.5 ) / 128 ) + 1 );

    Z_MagValuelsb = buffer[4];
    Z_MagValuemsb = buffer[5] << 8;
    Z_MagValue = Z_MagValuelsb + Z_MagValuemsb;
    if (Z_MagValue > 32767){ Z_MagValue = Z_MagValue - 65535;}
    Z_MagValueAdj = Z_MagValue * ((((Z_MagSensAdj-128)*0.5 ) / 128 ) + 1 );

    MagTimerActiveFlag = false;
   }

    // Simple 0to 360 degree compass heading
   if(X_MagValueAdj != 0){   //Holdoff for mag to wake up
	   Heading = ((atan2(Y_MagValueAdj,X_MagValueAdj)+M_PI)*180/M_PI);
	   //delay(30);
   }


    // This section is a mess with the nonblocking delay timers...
    // The altimeter needs ~8mS for a conversion. I use 12mS to be safe.
    // Nonblocking delay between conversion mbar and temperature start and read.
    // Continue to read gyros and accel. Continue esc updates.
   if((TemperatureTimerActiveFlag == false) &&
      (MbarReadDoneFlag == true)){
    TemperatureTimerActiveFlag	= true;
    StartMS5611_temp();  // ~8ms needed for temperature ADC conversion.
    TemperatureTimerStart = millis();
    TemperatureTimerEnd = TemperatureTimerStart + 12;
   }

   TemperatureTimerNow = millis();
   if ((TemperatureTimerActiveFlag == true) &&
	   (MbarReadDoneFlag == true) &&
	   (TemperatureTimerNow >= TemperatureTimerEnd)){
    ReadMS5611_temp(); // ~8ms needed for mbar ADC conversion.
    TemperatureTimerActiveFlag = false;
    TemperatureReadDoneFlag = true;
    MbarReadDoneFlag = false;
   }

   if((TemperatureReadDoneFlag == true) &&
      (MbarTimerActiveFlag == false) &&
      (MbarReadDoneFlag == false)){
     MbarTimerActiveFlag	= true;
     StartMS5611_mbar();  // 10ms each for temperature and mbar read.
     MbarTimerStart = millis();
     MbarTimerEnd = MbarTimerStart + 12;
   }

   MbarTimerNow = millis();
   if ((MbarTimerActiveFlag == true) &&
       (MbarTimerNow >= MbarTimerEnd) &&
       (TemperatureReadDoneFlag == true)){
	 ReadMS5611_mbar(); // 10ms each for temperature and mbar read.
	  //calcualte 1st order pressure and temperature (MS5607 1st order algorithm)
	 dT=DD2-C[4]*pow(2,8);
	 OFF=C[1]*pow(2,17)+dT*C[3]/pow(2,6);
	 SENS=C[1]*pow(2,16)+dT*C[2]/pow(2,7);
	 T=(2000+(dT*C[5])/pow(2,23))/100;
	 Mbar=(((DD1*SENS)/pow(2,21)-OFF)/pow(2,15))/100;

	 TemperatureReadDoneFlag = false;
	 MbarReadDoneFlag = true;
	 MbarTimerActiveFlag = false;

     #ifdef Use_SERIAL                  // Send gyro and accel data to Processing GUI
      if (loopcount > 30){              // Non-blocking delay for serial send
       loopcount = 0;
       Serial1.print("Mbar: ");
       Serial1.print(Mbar);
       Serial1.print(" T: ");
       Serial1.println(T);
      }
      loopcount++;                      // Overruns serial if no delay
     #endif
   }

   RwEstSmoothPitch =(RwEstSmoothPitch * 14 + RwEst[0]) / 15;
   RwEstSmoothRoll =(RwEstSmoothRoll * 14 + RwEst[1]) / 15;

    // Store X and Y accel values when entering FPV mode, this is 'hover' value
   if((FPV_ModeFlag == true) && (FPV_ModeLatch == false)){
    HoverPitchHold = (-RwEstSmoothPitch * 1600); // scaling factor hand tuned
    HoverRollHold  = (RwEstSmoothRoll * 1600); // scaling factor hand tuned
    //HoverHeadingHold  = Heading;
    HoverHeightHold  = Mbar * 205; // Dependant on props, weight, esc's, motors...
    digitalWrite(BlueLedPin, HIGH);
    FPV_ModeLatch = true;
   }

    // Apply auto-level if FPV mode
   if((FPV_ModeFlag == true) && (FPV_ModeLatch == true)){
    RxInPitchHold = RxInPitchHold - ((-RwEstSmoothPitch * 1600) - HoverPitchHold);
    RxInRollHold  = RxInRollHold  - ((RwEstSmoothRoll * 1600) - HoverRollHold);
    HeightAdjust = HoverHeightHold - (Mbar * 205);
    //HeightAdjust = 0;
   }
   else{
    HeightAdjust = 0;
    digitalWrite(BlueLedPin, LOW);
    FPV_ModeLatch = false;
   }
    // Always apply gyro stabilization
   RxInPitchHold += PitchGyroValue * PitchGyroScale;
   RxInRollHold += RollGyroValue * RollGyroScale;


   //#ifdef Use_SERIAL                  // Send gyro and accel data to Processing GUI
   // if (loopcount > 20){              // Non-blocking delay for serial send
   // loopcount = 0;
   // serialPrintFloatArr(RwAcc, 3);    // Output to Processing program
   // serialPrintFloatArr(Gyro_ds, 3);
   // serialPrintFloatArr(RwGyro, 3);
   // serialPrintFloatArr(Awz, 2);
   // serialPrintFloatArr(RwEst, 3);    // RwEst[0] and RwEst[1] used for cube rotation
   // Serial1.println();
   // }
   //loopcount++;                      // Overruns serial if no delay
   //#endif


    // Prime the motor speeds...
   LeftMotorOut  = (RxInThrottleHold - HeightAdjust) * ThrottleStickScale;
   RightMotorOut = (RxInThrottleHold - HeightAdjust) * ThrottleStickScale;
   RearMotorOut  = (RxInThrottleHold - HeightAdjust) * ThrottleStickScale;
   YawServoOut   = 90;               // Yaw center: 0 to 180 deg travel

    // Apply mixer calculations...
   RxInRollHold  = (RxInRollHold * 20)/23; //Sine 60= 0.866 ~ 20/23
   LeftMotorOut += RxInRollHold;
   RightMotorOut -= RxInRollHold;

   RearMotorOut -= (RxInPitchHold * 1.25);  // Fudge factor for COG
   RxInPitchHold = (RxInPitchHold >> 1); // cosine of 60
   LeftMotorOut += RxInPitchHold;
   RightMotorOut += RxInPitchHold;

   RxInYawHold += YawGyroValue * YawGyroScale;
   YawServoOut += RxInYawHold / 80;    // 76 Hand tuned scaling factor
                                       // Mechanical linkage dependent
    // Write speed/position values...
   pwmWrite(Esc1Pin,LeftMotorOut);     // Update front left motor speed
   pwmWrite(Esc2Pin,RightMotorOut);    // Update front right motor speed
   pwmWrite(Esc3Pin,RearMotorOut);     // Update rear motor speed
   yawservo.write(YawServoOut);        // Update yaw servo position

   // Do it again...
}
//*****************************************************************************
// Called routines
//*****************************************************************************
void InitI2C2() {
	 i2c_master_enable(I2C2, 1);       // i2c bus speed, 400khz=1, 100khz=0
	 delay(50);
}

//***************************************************************************
void InitITG3200() {
i2c_msg msgs[2];
uint8 buf[2] = {0x00, 0x00};

 buf[0] = 0x3E; buf[1] = 0x01;     // no reset, no sleep, no standby
 msgs[0].addr = 0x68;              // use X-gyro reference clock
 msgs[0].flags = 0;
 msgs[0].length = 2;
 msgs[0].data = buf;
 i2c_master_xfer(I2C2, msgs, 1);
 delay(50);

 buf[0] = 0x15; buf[1] = 0x01;     // Sample Rate Divider +1,  0-7 allowed
 msgs[0].addr = 0x68;              // 0x01 = 500Hz sample rate
 msgs[0].flags = 0;
 msgs[0].length = 2;
 msgs[0].data = buf;
 i2c_master_xfer(I2C2, msgs, 1);
 delay(50);
                                   // +/- 2000 dgrs/sec
 buf[0] = 0x16; buf[1] = 0x1B;     // 0x18 = 8KHz sample rate, 256Hz low pass way too twitchey
 msgs[0].addr = 0x68;              // 0x19 = 1KHz sample rate, 188Hz low pass still too twitchey
 msgs[0].flags = 0;                // 0x1A = 1KHz sample rate, 98Hz low pass jerkey, not predictable
 msgs[0].length = 2;               // 0x1B = 1KHz sample rate, 42Hz low pass not bad at all
 msgs[0].data = buf;               // 0x1C = 1KHz sample rate, 20Hz low pass OK, a little strange
 i2c_master_xfer(I2C2, msgs, 1);   // 0x1D = 1KHz sample rate, 10Hz low pass not tried
 delay(50);                        // 0X1E = 1KHZ sample rate, 5Hz low pass smooth but strange

 buf[0] = 0x17; buf[1] = 0x00;     // write configuration register
 msgs[0].addr = 0x68;
 msgs[0].flags = 0;
 msgs[0].length = 2;
 msgs[0].data = buf;
 i2c_master_xfer(I2C2, msgs, 1);
 delay(50);
}

//***************************************************************************
void ReadITG3200(uint8 *buf) {
i2c_msg msgs[2];

 buf[0] = 0x1D;
 msgs[0].addr = 0x68;
 msgs[0].flags = 0;
 msgs[0].length = 1;
 msgs[0].data = buf;

 msgs[1].addr = 0x68;
 msgs[1].flags = I2C_MSG_READ;
 msgs[1].length = 6;
 msgs[1].data = buf;
 i2c_master_xfer(I2C2, msgs, 2);
}

//***************************************************************************
void CalibrateGyro(){
unsigned int i;
uint8 buffer[6];

 for (i=0;i<32;i++){  // Average of 32 readings
  ReadITG3200(buffer);

  PitchGyroValue = ((buffer[0] << 8) | buffer[1]);
  if (PitchGyroValue > 32767) PitchGyroValue = PitchGyroValue - 65535;

  RollGyroValue = ((buffer[2] << 8) | buffer[3]);
  if (RollGyroValue > 32767) RollGyroValue = RollGyroValue - 65535;

  YawGyroValue = ((buffer[4] << 8) | buffer[5]);
  if (YawGyroValue > 32767) YawGyroValue = YawGyroValue - 65535;

  PitchGyroZero += PitchGyroValue;
  RollGyroZero += RollGyroValue;
  YawGyroZero += YawGyroValue;

  toggleLED();
  delay(50);
 }

 PitchGyroZero = (PitchGyroZero / 32);
 RollGyroZero  = (RollGyroZero / 32);
 YawGyroZero   = (YawGyroZero / 32);
}

//***************************************************************************
void InitBMA150() {              // Accelerometer initialization
i2c_msg msgs[2];
uint8 buf[2] = {0x00, 0x00};

 buf[0] = 0x14;                  // read register 0x14
 msgs[0].addr = 0x38;
 msgs[0].flags = 0;
 msgs[0].length = 1;
 msgs[0].data = buf;

 msgs[1].addr = 0x38;
 msgs[1].flags = I2C_MSG_READ;
 msgs[1].length = 1;
 msgs[1].data = buf;
 i2c_master_xfer(I2C2, msgs, 2);
 delay(50);

 buf[1] = buf[0] & 0b11100000;   // clear range and bandwidth, preserve 3 msb's
 buf[1] = buf[1] | 0b00000000;   // set range and bandwidth, write 5 lsb's
 buf[0] = 0x14;                  // control register address
 msgs[0].addr = 0x38;            // 50hz OK, now at 100hz
 msgs[0].flags = 0;              // 000 = 25hz, 001 = 50hz, 010 = 100hz
 msgs[0].length = 2;             // 011 = 190hz, 100 = 375hz
 msgs[0].data = buf;
 i2c_master_xfer(I2C2, msgs, 2); // +/-2g, 25hz lowpass filter
 delay(50);
}

// ***************************************************************************
void ReadBMA150(uint8 *buf) {
i2c_msg msgs[2];

 buf[0] = 0x02;
 msgs[0].addr = 0x38;
 msgs[0].flags = 0;
 msgs[0].length = 1;
 msgs[0].data = buf;

 msgs[1].addr = 0x38;
 msgs[1].flags = I2C_MSG_READ;
 msgs[1].length = 6;
 msgs[1].data = buf;
 i2c_master_xfer(I2C2, msgs, 2);
}

// ***************************************************************************
void ReadAK8975(uint8 *buf) {
i2c_msg msgs[2];

 buf[0] = 0x03;              // 1st byte of returned data address
 msgs[0].addr = 0x0C;
 msgs[0].flags = 0;
 msgs[0].length = 1;
 msgs[0].data = buf;

 msgs[1].addr = 0x0C;       // Start condition, then read 6 bytes back
 msgs[1].flags = I2C_MSG_READ;
 msgs[1].length = 6;
 msgs[1].data = buf;
 i2c_master_xfer(I2C2, msgs, 2);
}

// ***************************************************************************
void StartConversionAK8975() {
i2c_msg msgs[2];
uint8 buf[2] = {0x00, 0x00};

 buf[0] = 0x0A;               // CNTL register address
 buf[1] = 0x01;               // Single conversion mode command
 msgs[0].addr = 0x0C;         // device address
 msgs[0].flags = 0;
 msgs[0].length = 2;
 msgs[0].data = buf;
 i2c_master_xfer(I2C2, msgs, 1);
}

// ***************************************************************************
void InitAK8975() {
i2c_msg msgs[2];
uint8 buf[3] = {0x00, 0x00, 0x00};

 buf[0] = 0x0A;               // CNTL register address
 buf[1] = 0x0F;               // Fuse ROM access mode
 msgs[0].addr = 0x0C;         // device address
 msgs[0].flags = 0;
 msgs[0].length = 2;
 msgs[0].data = buf;
 i2c_master_xfer(I2C2, msgs, 1);
 delay(100);

 buf[0] = 0x10;              // 1st byte of returned data address
 msgs[0].addr = 0x0C;
 msgs[0].flags = 0;
 msgs[0].length = 1;
 msgs[0].data = buf;

 msgs[1].addr = 0x0C;        // Read sensitivity adjustment values from ROM
 msgs[1].flags = I2C_MSG_READ;
 msgs[1].length = 3;
 msgs[1].data = buf;
 i2c_master_xfer(I2C2, msgs, 2);

 X_MagSensAdj = buf[0];
 Y_MagSensAdj = buf[1];
 Z_MagSensAdj = buf[2];
}

// ***************************************************************************
void InitMS5611() {
i2c_msg msgs[2];
uint8 buf[2] = {0x00,0x00};

 buf[0] = 0x1E;                    // reset command
 msgs[0].addr = 0x77;              // write address
 msgs[0].flags = 0;
 msgs[0].length = 1;
 msgs[0].data = buf;
 i2c_master_xfer(I2C2, msgs, 1);
}

// ***************************************************************************
void ReadMS5611_Prom() {
i2c_msg msgs[2];
uint8 buf[2];

int i=0;

for (i=0;i<6;i++){             // read 8 coefficients

  buf[0] = 0xA2 + (i*2);       // Send read prom command
  msgs[0].addr = 0x77;         // Write address
  msgs[0].flags = 0;
  msgs[0].length = 1;
  msgs[0].data = buf;
  i2c_master_xfer(I2C2, msgs, 1);

  msgs[0].addr = 0x77;         // Read address
  msgs[0].flags = I2C_MSG_READ;
  msgs[0].length = 2;
  msgs[0].data = buf;
  i2c_master_xfer(I2C2, msgs, 1);

  C[i] = buf[0]*256;
  C[i]+= buf[1];

  //Serial1.print("coefficient ");
  //Serial1.print(i, DEC);
  //Serial1.print(": ");
  //Serial1.println(C[i], DEC);
 }
}

// ***************************************************************************
void StartMS5611_temp() {
i2c_msg msgs[2];
uint8 buf[3];

 buf[0] = 0x58;               // Send read DD2 command
 msgs[0].addr = 0x77;         // Write address
 msgs[0].flags = 0;
 msgs[0].length = 1;
 msgs[0].data = buf;
 i2c_master_xfer(I2C2, msgs, 1);
}

// ***************************************************************************
void ReadMS5611_temp() {
i2c_msg msgs[2];
uint8 buf[3];

 buf[0] = 0x00;               // Send read ADC command
 msgs[0].addr = 0x77;         // Write address
 msgs[0].flags = 0;
 msgs[0].length = 1;
 msgs[0].data = buf;
 i2c_master_xfer(I2C2, msgs, 1);

 msgs[0].addr = 0x77;         // Read address
 msgs[0].flags = I2C_MSG_READ;
 msgs[0].length = 3;
 msgs[0].data = buf;
 i2c_master_xfer(I2C2, msgs, 1);

 DD2  =  buf[0] * 65536;
 DD2 += (buf[1] * 256);
 DD2 +=  buf[2];
}


// ***************************************************************************
void StartMS5611_mbar() {
i2c_msg msgs[2];
uint8 buf[3];

 buf[0] = 0x48;               // Send read DD1 command
 msgs[0].addr = 0x77;         // Write address
 msgs[0].flags = 0;
 msgs[0].length = 1;
 msgs[0].data = buf;
 i2c_master_xfer(I2C2, msgs, 1);
}

// ***************************************************************************
void ReadMS5611_mbar() {
i2c_msg msgs[2];
uint8 buf[3];

 buf[0] = 0x00;               // Send read ADC command
 msgs[0].addr = 0x77;         // Write address
 msgs[0].flags = 0;
 msgs[0].length = 1;
 msgs[0].data = buf;
 i2c_master_xfer(I2C2, msgs, 1);

 msgs[0].addr = 0x77;         // Read address
 msgs[0].flags = I2C_MSG_READ;
 msgs[0].length = 3;
 msgs[0].data = buf;
 i2c_master_xfer(I2C2, msgs, 1);

 DD1  =  buf[0] * 65536;
 DD1 += (buf[1] * 256);
 DD1 +=  buf[2];
}
// ***************************************************************************
// Thanks to Fabio for the 6DOF sensor fusion code that follows.

// convert raw readings to degrees/sec
// adjust scale factor for your gryo's
void rawGyroToDegsec(int * raw, float * gyro_ds) {
   gyro_ds[0] = ((float) raw[0]); // / 11;    // / 14.375
   gyro_ds[1] = ((float) raw[1]); // / 11;    // / 14.375
   gyro_ds[2] = ((float) raw[2]); // / 11;    // / 14.375
}
// ***************************************************************************

void rawAccToG(int * raw, float * RwAcc) {
   RwAcc[0] = ((float) raw[0]) / 256.0;
   RwAcc[1] = ((float) raw[1]) / 256.0;
   RwAcc[2] = ((float) raw[2]) / 256.0;
}
// ***************************************************************************

void normalize3DVec(float * vector) {
 float R;
   R = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
   vector[0] /= R;
   vector[1] /= R;
   vector[2] /= R;
}
// ***************************************************************************

float squared(float x){
   return x*x;
}
// ***************************************************************************

void getInclination() {
 int w = 0;
 float tmpf = 0.0;
 int currentTime, signRzGyro;

   currentTime = millis();
   interval = currentTime - lastTime;
   lastTime = currentTime;

   if (firstSample) { // the NaN check is used to wait for good data from the Arduino
    for(w=0;w<=2;w++) {
      RwEst[w] = RwAcc[w];    //initialize with accelerometer readings
    }
   }
   else{
    //evaluate RwGyro vector
    if(abs(RwEst[2]) < 0.1) {
      //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
      //in this case skip the gyro data and just use previous estimate
      for(w=0;w<=2;w++) {
        RwGyro[w] = RwEst[w];
      }
    }
    else {
      //get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
      for(w=0;w<=1;w++){
        tmpf = Gyro_ds[w];                        //get current gyro rate in deg/s
        tmpf *= interval / 1000.0f;                     //get angle change in deg
        Awz[w] = atan2(RwEst[w],RwEst[2]) * 180 / PI;   //get angle and convert to degrees
        Awz[w] += tmpf;             //get updated angle according to gyro movement
      }

      //estimate sign of RzGyro by looking in what qudrant the angle Axz is,
      //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
      signRzGyro = ( cos(Awz[0] * PI / 180) >=0 ) ? 1 : -1;

      //reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
      for(w=0;w<=1;w++){
        RwGyro[0] = sin(Awz[0] * PI / 180);
        RwGyro[0] /= sqrt( 1 + squared(cos(Awz[0] * PI / 180)) * squared(tan(Awz[1] * PI / 180)) );
        RwGyro[1] = sin(Awz[1] * PI / 180);
        RwGyro[1] /= sqrt( 1 + squared(cos(Awz[1] * PI / 180)) * squared(tan(Awz[0] * PI / 180)) );
      }
      RwGyro[2] = signRzGyro * sqrt(1 - squared(RwGyro[0]) - squared(RwGyro[1]));
    }

    //combine Accelerometer and gyro readings
    for(w=0;w<=2;w++) RwEst[w] = (RwAcc[w] + wGyro * RwGyro[w]) / (1 + wGyro);

    normalize3DVec(RwEst);
  }

  firstSample = false;
}
// ***************************************************************************

void serialPrintFloatArr(float * arr, int length) {
   for(int i=0; i<length; i++) {
    serialFloatPrint(arr[i]);
    Serial1.print(",");
   }
}
// ***************************************************************************

void serialFloatPrint(float f) {
   byte * b = (byte *) &f;
   Serial1.print("f:");
   for(int i=0; i<4; i++) {

    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

    Serial1.print(c1);
    Serial1.print(c2);
   }
}
// ***************************************************************************

//*****************************************************************************
// Interrupt routines
//*****************************************************************************

void handler_CH1(void) {               // Measure RX Pitch duration
    if(Chan1begin == 0){
     Chan1begin = micros();
     Chan1prev = RxInPitch;
    }
    else{
     Chan1end = micros();
     RxInPitch = Chan1end - Chan1begin;
     if((RxInPitch < 1000) || (RxInPitch > 2000)) //Glitch suppression
     {RxInPitch = Chan1prev; Chan1end = 0;}
     Chan1begin = 0;
    }  
}    
    
void handler_CH2(void) {               // Measure RX Roll duration
    if(Chan2begin == 0){
     Chan2begin = micros();
     Chan2prev = RxInRoll;
    }
    else{
     Chan2end = micros();
     RxInRoll = Chan2end - Chan2begin;
     if((RxInRoll < 1000) || (RxInRoll > 2000)) //Glitch suppression
     {RxInRoll = Chan2prev; Chan2end = 0;}
     Chan2begin = 0;
    }  
}    

void handler_CH3(void) {               // Measure RX Throttle duration
    if(Chan3begin == 0){
     Chan3begin = micros();
     Chan3prev = RxInThrottle;
    }
    else{
     Chan3end = micros();
     RxInThrottle = Chan3end - Chan3begin;
     if((RxInThrottle < 1000) || (RxInThrottle > 2000)) //Glitch suppression
     {RxInThrottle = Chan3prev; Chan3end = 0;}
     Chan3begin = 0;
    }  
}    

void handler_CH4(void) {               // Measure RX Yaw duration
    if(Chan4begin == 0){
     Chan4begin = micros();
     Chan4prev = RxInYaw;
    }
    else{
     Chan4end = micros();
     RxInYaw = Chan4end - Chan4begin;
     if((RxInYaw < 1000) || (RxInYaw > 2000)) //Glitch suppression
     {RxInYaw = Chan4prev; Chan4end = 0;}
     Chan4begin = 0;
    }  
} 

void handler_CH5(void) {               // Measure Channel 5 duration
    if(Chan5begin == 0){
     Chan5begin = micros();
     Chan5prev = RxInChan5;
    }
    else{
     Chan5end = micros();
     RxInChan5 = Chan5end - Chan5begin;
     if((RxInChan5 < 1000) || (RxInChan5 > 2000)) //Glitch suppression
     {RxInChan5 = Chan5prev; Chan5end = 0;}
     Chan5begin = 0;
    }
}
// ***************************************************************************

__attribute__(( constructor )) void premain() {
    init();
}

int main(void) {
    setup();

    while (1) {
        loop();
    }
    return 0;
}

