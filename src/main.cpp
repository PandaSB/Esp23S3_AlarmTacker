#include <Arduino.h>
#include "hardware.h"
#include "config.h"
#include <esp_camera.h>
#include <FastLED.h>



#define SerialAT mySerial
#define SerialMon Serial

#define TINY_GSM_MODEM_SIM7670
#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 1024
#endif
#define DUMP_AT_COMMANDS
//#define TINY_GSM_DEBUG 

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>


static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sccb_sda = SIOD_GPIO_NUM,
    .pin_sccb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_SVGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.
    .jpeg_quality = 12,
    .fb_count = 2,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    
};




MyMax17048 max17048;
MyRGBLed rgbLed;

//MyModem modem(115200); // Initialize modem with baud rate 115200
String rev;

HardwareSerial mySerial(1);

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif

const char apn[]      = CONFIG_GPRS_APN;
const char gprsUser[] = CONFIG_GPRS_USER;
const char gprsPass[] = CONFIG_GPRS_PASSWD;

const char server[]   = CONFIG_SERVER_ADDR;
const int  port       = CONFIG_SERVER_PORT;
const char resource_record[] = CONFIG_SERVER_PATH_RECORD;
const char ressouce_config[] = CONFIG_SERVER_PATH_CONFIG;


TinyGsmClient client(modem);
HttpClient    http(client, server, port);
//TinyGsmClientSecure client(modem);
//HttpClient          http(client, server, port);

bool alarmon = false;
bool alarmstatus = false; 
bool rebootingissue = false ;
bool networkready = false ; 
bool datasend = false ; 
bool dataread = false ; 
bool batready = false ; 
bool posGPSready = false ; 
bool posGSMready = false ;
bool deviceGPSready = false ; 
float batvalue = -1; 
int loopstartGPS = 0; 
int loopstartGPRS = 0 ; 
float lat = 0;
float lon = 0;
float speed = 0;
float alt = 0;
float latgsm = 0;
float longsm = 0;
float accuracy=0;
float accuracygsm=0;
int loopgps = 0; 
int loopdelay = 5;
int64_t chipid = 0;
uint16_t chip = 0;
String Date = ""; 
String Time = ""; 

RTC_DATA_ATTR int bootCount = 0;

#define uS_TO_S_FACTOR 1000000ULL 
#define TIME_TO_SLEEP  60   
#define TIME_TO_SLEEP_REBOOT  10   

CRGB leds[CONFIG_NUM_LEDS];

static esp_err_t init_camera(void)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        SerialMon.println( "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup caused by ULP program"); break;
    default:                        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void SentSerial(const char *p_char) {
  for (int i = 0; i < strlen(p_char); i++) {
    SerialAT.write(p_char[i]);
    delay(10);
  }

  SerialAT.write('\r');
  delay(10);
  SerialAT.write('\n');
  delay(10);
}

bool SentMessage(const char *p_char, unsigned long timeout = 2000) {
  char r_buf[200];

  SentSerial(p_char);

  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (SerialAT.available()) {
      rev = SerialAT.readString();
      char *p_response = strstr(rev.c_str(), "\r\n");
      if (p_response != NULL) {
        p_response += 2;
        if (strstr(p_response, "OK") != NULL) {
          Serial.print(rev);
          Serial.println("Got OK!");
          return true;
        }
      }
    }
  }
  Serial.print(rev);
  Serial.println("Timeout!");
  return false;
}

uint32_t TinyGsmAutoBaud()
{
    static uint32_t rates[] = {115200, 57600,  38400, 19200, 9600,  74400, 74880,
                               230400, 460800, 2400,  4800,  14400, 28800
                              };

    for (uint8_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
        uint32_t rate = rates[i];
        if (rate < 9600 || rate > 115200) continue;

        Serial.print("Trying baud rate ");
        Serial.print(rate);
        Serial.println("...");
        SerialAT.updateBaudRate(rate);
        delay(10);
        for (int j = 0; j < 10; j++) {
            SerialAT.print("AT\r\n");
            String input = SerialAT.readString();
            if (input.indexOf("OK") >= 0) {
                Serial.print("Modem responded at rate ");
                Serial.println(rate);
                Serial.println(input);
                return rate;
            }
        }
    }
    return 0;
}

String SendGETMessage (String uri) {
    
    http.connectionKeepAlive();  // Currently, this is needed for HTTPS

    int err = http.get(uri);
    if (err != 0) {
      SerialMon.println(F("failed to connect"));
      delay(10000);
      return ("");
    }

    int status = http.responseStatusCode();
    SerialMon.print(F("Response status code: "));
    SerialMon.println(status);
    if (!status) {
      delay(10000);
      return ("");
    }

    SerialMon.println(F("Response Headers:"));
    while (http.headerAvailable()) {
      String headerName  = http.readHeaderName();
      String headerValue = http.readHeaderValue();
      SerialMon.println("    " + headerName + " : " + headerValue);
    }

    int length = http.contentLength();
    if (length >= 0) {
      SerialMon.print(F("Content length is: "));
      SerialMon.println(length);
    }
    if (http.isResponseChunked()) {
      SerialMon.println(F("The response is chunked"));
    }

    String body = http.responseBody();
    SerialMon.println(F("Response:"));
    SerialMon.println(body);

    SerialMon.print(F("Body length is: "));
    SerialMon.println(body.length());

    return (body);

}

String  SendPOSTMessage (String uri , String contentType , String data) 
{
    http.connectionKeepAlive();  // Currently, this is needed for HTTPS
    http.post(uri, contentType, data);

    // read the status code and body of the response
    int statusCode = http.responseStatusCode();
    String response = http.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode);
    Serial.print("Response: ");
    Serial.println(response);
    return response ; 

}


void setup() {

  
  FastLED.addLeds<WS2812B, CONFIG_DATA_PIN, RGB>(leds, CONFIG_NUM_LEDS); 
  leds[0] = CRGB::Red;
  FastLED.show();

  Serial.begin(115200);
  while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }  
  delay(5000); 

  bootCount ++;

  pinMode(CONFIG_MODEM_PWR, OUTPUT);
  digitalWrite(CONFIG_MODEM_PWR, LOW);
  delay (1000);
  digitalWrite(CONFIG_MODEM_PWR, HIGH);

  SerialAT.begin(115200,SERIAL_8N1, 17,  18 ); 
  SerialMon.println("Boot number: " + String(bootCount));
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);


  TinyGsmAutoBaud();
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  init_camera();
  camera_fb_t *pic = esp_camera_fb_get();
  leds[0] = CRGB::Yellow;
  FastLED.show();




}

void loop() {

  if (!networkready) {
    SerialMon.println("Connect to GPRS");
    modem.gprsConnect(apn, gprsUser, gprsPass);

    SerialMon.print("Waiting for network...");
    if (!modem.waitForNetwork()) {
      SerialMon.println(" fail");
      delay(2000);
      loopstartGPRS ++ ; 
      networkready = false ; 
      if (loopstartGPRS > 10 ) {
        rebootingissue = true ; 
      }
      return;
    }
    SerialMon.println(" success");
    networkready = true; 
    return; 

  }
  
  if (!batready) {
    SerialMon.println("Read Battery Value");

    batvalue = max17048.getBatteryLevel();
    batready = true; 
    chipid = ESP.getEfuseMac(); // The chip ID is essentially its MAC address(length: 6 bytes).
    chip = (uint16_t)(chipid >> 32);
  }

  if (!deviceGPSready) {
    SerialMon.println("Enable GPS device");
    if (!modem.enableGPS() ) loopstartGPS++; 
    else  deviceGPSready = true ;
    
    if (loopstartGPS > 3)
    {
      deviceGPSready = true ;
      rebootingissue = true ; 
    } 
    leds[0] = CRGB::Blue;
    FastLED.show();
  }

  if ((!posGPSready) && (deviceGPSready)){
    SerialMon.println("Enable GPS position");

    SerialMon.print(F("GNSS raw info: "));
    SerialMon.println(modem.getGPSraw());
    SerialMon.print(F("GNSS info: "));
    modem.getGPS(&lat, &lon , &speed ,&alt,NULL,NULL,&accuracy )  ; 
    SerialMon.print("lat : ");
    SerialMon.printf("%3.8f",lat);
    SerialMon.print(" - lon : ");
    SerialMon.printf("%3.8f",lon);
    SerialMon.print(" - loop : ");
    SerialMon.println(loopgps);

    if (((lat != 0) && (lon !=0)) || (loopgps >= (120/loopdelay))) {
      posGPSready = true ; 
    }
    else
    {
      delay(loopdelay * 1000);
      loopgps ++ ;  
    }
  }

  if (!posGSMready) {
    int year, month, day, hour, minute, second;
    modem.getGsmLocation(&latgsm,&longsm,&accuracygsm,&year,&month,&day,&hour,&minute,&second) ; 
    if ((latgsm != 0) && (longsm !=0) ) {
      Date = ((day < 10) ? "0" : "") + String (day) + "/" + ((month < 10) ? "0" : "") +  String (month) + "/" + String(year) ; 
      Time = ((hour < 10) ? "0" : "") + String (hour) + ":" + ((minute < 10) ? "0" : "") + String (minute) + ":" +((second < 10) ? "0" : "") + String (second) ; 
      posGSMready = true ; 
    } 
  }

  if (!dataread && networkready )
  {
    String data = ""; 
    String uri = ""; 
    String Answer = ""; 
    char szchipid[9];

    SerialMon.println ("Receive config from server ");
    leds[0] = CRGB::Orange;
    FastLED.show();
    itoa(chipid, szchipid, 16);
    data = "chipid=" + String(szchipid);

    Serial.println("making POST request");
    Answer = SendPOSTMessage (String (ressouce_config) , "application/x-www-form-urlencoded" , data) ; 

    SerialMon.print ("config answer : ") ; Serial.println (Answer) ; 
     dataread = true; 
  }

  if (networkready && batready && posGPSready && posGSMready && !datasend  ){
    String data = ""; 
    String uri = ""; 
    char szchipid[9];

    SerialMon.println("Send Data to GPRS");

    itoa(chipid, szchipid, 16);
    leds[0] = CRGB::Green;
    FastLED.show();
    data = "chipid=" + String(szchipid) + "&date=" + Date + "&time=" + Time + "&bat=" + String(batvalue,9) + "&lat=" + String(lat,9) + "&lon=" + String(lon,9) + "&speed=" + String (speed,9) +"&accuracy=" + String(accuracy,9) + "&longsm=" + String(longsm,9) + "&latgsm=" + String(latgsm,9) + "&accuracygsm=" + String(accuracygsm,9) + "&alarmon=" + String(alarmon) + "&alarmstatus=" + String(alarmstatus) ; 
#if 0
    Serial.println("making GET request");    
    uri = resource_record + String("?") + data ; 
    SendGETMessage ( uri) ; 

#endif 

    Serial.println("making POST request");
    SendPOSTMessage (String (resource_record) , "application/x-www-form-urlencoded" , data) ; 

    datasend = true ; 
  }

  if (datasend && dataread) {
    /*
    if (SerialAT.available()) {
      char buf ; 
      SerialMon.write(SerialAT.read());
    }
    if (SerialMon.available()) {
      char buf ; 
      SerialAT.write(SerialMon.read());
    }
  */
    leds[0] = CRGB::Black;
    FastLED.show();      
    SerialMon.println("Switch in sleep mode") ; 
    SerialMon.flush();
    esp_deep_sleep_start();
  }

  if (rebootingissue) {
    SerialMon.println("Issue need reboot") ; 
    SerialMon.flush();
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_REBOOT * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }

  delay(1000) ; 
  SerialMon.println("Status : ") ; 
  SerialMon.print (" - deviceGPSready") ; SerialMon.println (deviceGPSready) ; 
  SerialMon.print (" - batready") ; SerialMon.println (batready) ; 
  SerialMon.print (" - networkready") ; SerialMon.println (networkready) ; 
  SerialMon.print (" - posGPSready") ; SerialMon.println (posGPSready) ; 
  SerialMon.print (" - posGSMready") ; SerialMon.println (posGSMready) ; 
  SerialMon.print (" - datasend") ; SerialMon.println (datasend) ; 
  SerialMon.print (" - dataread") ; SerialMon.println (dataread) ; 

  if (SerialMon.available()) {
    char buf = SerialMon.read();
    if (buf == 'r') {
      esp_sleep_enable_timer_wakeup(1 * uS_TO_S_FACTOR);
      esp_deep_sleep_start();     
    }

  }

}
