#include <Arduino.h>
#include "hardware.h"
#include "config.h"

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


TinyGsmClient client(modem);
HttpClient    http(client, server, port);
//TinyGsmClientSecure client(modem);
//HttpClient          http(client, server, port);

  bool networkready = false ; 
  bool datasend = false ; 
  bool batready = false ; 
  bool posGPSready = false ; 
  bool posGSMready = false ;
  float batvalue = -1; 
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
#define TIME_TO_SLEEP_MODEM  30   

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



void setup() {
  int loopstartGPS = 0 ; 
  bool modemstatusfail = false ; 
  Serial.begin(115200);
  while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }  
  delay(5000); 

  bootCount ++;

  pinMode(33, OUTPUT);
  digitalWrite(33, LOW);
  delay (1000);
  digitalWrite(33, HIGH);

  SerialAT.begin(115200,SERIAL_8N1, 17,  18 ); 
  SerialMon.println("Boot number: " + String(bootCount));
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);


  TinyGsmAutoBaud();
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  while((modemstatusfail = !modem.enableGPS()) && (loopstartGPS < 3))
  {
    loopstartGPS++; 
  }
  if (modemstatusfail )
  {
      SerialMon.println ("Modem not start , Switch in sleep ");
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_MODEM * uS_TO_S_FACTOR);
      SerialMon.flush();
      esp_deep_sleep_start();

  }



}

void loop() {

  if (!networkready) {
    modem.gprsConnect(apn, gprsUser, gprsPass);

    SerialMon.print("Waiting for network...");
    if (!modem.waitForNetwork()) {
      SerialMon.println(" fail");
      delay(10000);
      return;
    }
    SerialMon.println(" success");
    if (modem.isNetworkConnected()) 
    { 
      SerialMon.println("Network connected"); 
      networkready = true;
    }
  }
  
  if (!batready) {
    batvalue = max17048.getBatteryLevel();
    batready = true; 
    chipid = ESP.getEfuseMac(); // The chip ID is essentially its MAC address(length: 6 bytes).
    chip = (uint16_t)(chipid >> 32);
  }

  if (!posGPSready) {
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



  if (networkready && batready && posGPSready && posGSMready && !datasend  ){
    String data = ""; 
    String uri = ""; 

    char szchipid[9];
    itoa(chipid, szchipid, 16);

    http.connectionKeepAlive();  // Currently, this is needed for HTTPS
    data = "chipid=" + String(szchipid) + "&date=" + Date + "&time=" + Time + "&bat=" + String(batvalue,9) + "&lat=" + String(lat,9) + "&lon=" + String(lon,9) + "&speed=" + String (speed,9) +"&accuracy=" + String(accuracy,9) + "&longsm=" + String(longsm,9) + "&latgsm=" + String(latgsm,9) + "&accuracygsm=" + String(accuracygsm,9) ; 
#if 0
    uri = resource_record + String("?") + data ; 
    
    SerialMon.print(F("Performing HTTPS GET request... "));
    int err = http.get(uri);
    if (err != 0) {
      SerialMon.println(F("failed to connect"));
      delay(10000);
      return;
    }

    int status = http.responseStatusCode();
    SerialMon.print(F("Response status code: "));
    SerialMon.println(status);
    if (!status) {
      delay(10000);
      return;
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
#endif 

    Serial.println("making POST request");
    String contentType = "application/x-www-form-urlencoded";

    uri = resource_record ; 
    http.post(uri, contentType, data);

    // read the status code and body of the response
    int statusCode = http.responseStatusCode();
    String response = http.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode);
    Serial.print("Response: ");
    Serial.println(response);




    datasend = true ; 
  }

  if (datasend) {
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
    SerialMon.println("Switch in sleep mode") ; 
    SerialMon.flush();
    esp_deep_sleep_start();
  }

}
