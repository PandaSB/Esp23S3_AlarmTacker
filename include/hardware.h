#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include <Wire.h>

class MyMax17048 {

public:
    MyMax17048();
    float getBatteryLevel();
    ~MyMax17048(); 
private:   
};

class MyRGBLed {
public:
    MyRGBLed();
    void setBrightness(int brightness);
    void setColor( uint32_t Color);
    ~MyRGBLed();
private:
};

#endif // _HARDWARE_H_