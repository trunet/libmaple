#include "Wire.h"

/* the c++ frontend libraries, dubbed Wirish, are always included in sketches. 
However, low level libraries, dubbed libmaple, must be included manually */
#include "wirish.h"
#include "i2c.h"

#define TIMEOUT 30

#define XL345_ADDR    0x53 // use the 7 bit I2C address of the xl345
#define XL345_DEVID   0xE5
#define ITG3200_ADDR  0x69
#define ITG3200_DEVID 0x69

// ADXL Control Registers
#define ADXLREG_BW_RATE      0x2C
#define ADXLREG_POWER_CTL    0x2D
#define ADXLREG_DATA_FORMAT  0x31
#define ADXLREG_FIFO_CTL     0x38
#define ADXLREG_BW_RATE      0x2C
#define ADXLREG_TAP_AXES     0x2A
#define ADXLREG_DUR          0x21

//ADXL Data Registers
#define ADXLREG_DEVID        0x00
#define ADXLREG_DATAX0       0x32
#define ADXLREG_DATAX1       0x33
#define ADXLREG_DATAY0       0x34
#define ADXLREG_DATAY1       0x35
#define ADXLREG_DATAZ0       0x36
#define ADXLREG_DATAZ1       0x37

void writeADXL(uint8 reg, uint8 data) {
  /* all i2c transactions send and receive arrays of i2c_msg objects */
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
  uint8 msg_data[2];
  
  msg_data = {reg,data};
  msgs[0].addr = XL345_ADDR;
  msgs[0].flags = 0; // write
  msgs[0].length = 2; 
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C2, msgs, 1,0);
}

void xl345_init() {
  /* all i2c transactions send and receive arrays of i2c_msg objects */
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
  uint8 msg_data[2];

  /* first make sure everything is wired up properly, by reading the device ID
    The device ID is located in address 0x00 and should always read back 0xE5
    
    To read this device ID, we must first WRITE "0x00" to the device to set the 
    address to read on the next transaction. Then we must read 1 byte. 
   */
  
  /* i2c_msg objects have fields:
     addr - the 7 bit i2c address of the receiving device
     flags - could be I2C_MSG_READ and/or I2C_MSG_10BIT_ADDR, or 0 for regular write
     length - the number of bytes to transact
     data - pointer to data array being xmitted/received
   */
  msg_data = {0x00,0x00};
  msgs[0].addr = XL345_ADDR;
  msgs[0].flags = 0; 
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C2, msgs, 1,0);
  
  msgs[0].addr = XL345_ADDR;
  msgs[0].flags = I2C_MSG_READ; 
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C2, msgs, 1,0);
  
  /* now we check msg_data for our 0xE5 magic number */
  uint8 dev_id = msg_data[0];
  SerialUSB.print("Read device ID from xl345: ");
  SerialUSB.println(dev_id,HEX);
  
  if (dev_id != XL345_DEVID) {
    SerialUSB.println("Error, incorrect xl345 devid!");
    SerialUSB.println("Halting program, hit reset...");
    waitForButtonPress(0);
  }
  
  /* to enable the xl345 into read mode we need to send three i2c messages
    each messages represents a write into a particular configuration register 
    of the xl345. 
    
      register 0x2C is "BW_RATE" register, which we set to 0x0A (
      register 0x2D is the "POWER_CTRL" register, which we set to 0x08 (
      register 0x31 is the "DATa_FORMAT" register, which we set to 0x04 (
  */ 

  writeADXL(ADXLREG_POWER_CTL, 0x00);
  writeADXL(ADXLREG_POWER_CTL, 0x08);
  
}

void readADXL_burst(uint8 reg, uint8 len, uint8 *msg_data) {
  i2c_msg msgs[1]; 
  msg_data[0] = reg;
  
  msgs[0].addr = XL345_ADDR;
  msgs[0].flags = 0; 
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C2, msgs, 1,0);
  
  msgs[0].addr = XL345_ADDR;
  msgs[0].flags = I2C_MSG_READ; 
  msgs[0].length = len; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C2, msgs, 1,0);
}

int xl345_read_axis(uint8 AXIS_REG) {
  int axis = 0;
  uint8 temp[2]={0};
  
  readADXL_burst(AXIS_REG,2,temp);
  
  axis=((temp[1]<<8)|temp[0]);
  return axis;
}

void setup() {
  /* start the sketch when we hit the button, timeout=0 (wait forever) */
  waitForButtonPress(0);
  SerialUSB.println("Initializing sensors");
  
  /* configure I2C port 2 (pins 0, 1) with no special option flags (second argument)*/
  i2c_master_enable(I2C2, 0);
  delay(100); // give time for the peripheral to come up
  
  xl345_init();  
  waitForButtonPress();
}

void loop() {
  delay(250);
  int x,y,z;
  x = xl345_read_axis(ADXLREG_DATAX0);
  y = xl345_read_axis(ADXLREG_DATAY0);
  z = xl345_read_axis(ADXLREG_DATAZ0);
  SerialUSB.print(x);
  SerialUSB.print(", ");
  SerialUSB.print(y);
  SerialUSB.print(", ");
  SerialUSB.println(z);
}
