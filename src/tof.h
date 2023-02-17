#include "Adafruit_VL53L0X.h"
#include <SPI.h>

class tof{
  public:
    void setup(void);
    float distance(void);

};