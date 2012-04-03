#ifndef BMA020_h
#define BMA020_h

#include <stdint.h>
#include "wirish.h"
#include "i2c.h"

void ACC_init();
void ACC_getADC(int &acc_x, int &acc_y, int &acc_z);

#endif
