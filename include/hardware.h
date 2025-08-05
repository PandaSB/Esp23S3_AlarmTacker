#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include <Wire.h>

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     34
#define SIOD_GPIO_NUM     15
#define SIOC_GPIO_NUM     16
#define Y9_GPIO_NUM       14
#define Y8_GPIO_NUM       13
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       11
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       9
#define Y3_GPIO_NUM       8
#define Y2_GPIO_NUM       7
#define VSYNC_GPIO_NUM    36
#define HREF_GPIO_NUM     35
#define PCLK_GPIO_NUM     37

#define CONFIG_MODEM_PWR  33

#define CONFIG_DATA_PIN   38
#define CONFIG_NUM_LEDS   1



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