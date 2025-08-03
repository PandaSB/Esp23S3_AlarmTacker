#include "hardware.h"
#include <Arduino.h>



#define MAX17048_I2C_ADDRESS 0x36


MyMax17048::MyMax17048() {
    Wire.begin(3, 2);
}   

float MyMax17048::getBatteryLevel() {
    Wire.beginTransmission(MAX17048_I2C_ADDRESS);
    Wire.write(0x02);
    Wire.endTransmission();

    Wire.requestFrom(MAX17048_I2C_ADDRESS, 2);
    uint16_t soc = (Wire.read() << 8) | Wire.read();

    if (soc > 65535) {
        soc = 65535;
    }

   return( (float)soc / 65535.0 * 5);
}


MyMax17048::~MyMax17048() {
  // Destructor
}   


MyRGBLed::MyRGBLed() {
}   

void MyRGBLed::setBrightness(int brightness) {
}

void MyRGBLed::setColor( uint32_t Color) {
}

MyRGBLed::~MyRGBLed() {
}   