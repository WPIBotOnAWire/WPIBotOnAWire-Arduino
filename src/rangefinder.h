# include "rosHandler.h"
#include "Adafruit_VL53L0X.h"

class rangefinder{
    public:
    void init(void);
    void range(void);
    rosHandler rh;

    private:
    VL53L0X_RangingMeasurementData_t measure;
    unsigned long range_timer;
    Adafruit_VL53L0X sensor = Adafruit_VL53L0X();
};