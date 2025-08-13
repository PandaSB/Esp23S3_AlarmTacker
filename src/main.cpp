#include <Arduino.h>
#include "hardware.h"
#include "config.h"
#include <esp_camera.h>
#include <FastLED.h>
#include <ArduinoJson.h>
#include <base64.h>
#include <FS.h>
#include <SD_MMC.h>
#include <ArduinoJson.h>



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
    .frame_size = FRAMESIZE_VGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.
    .jpeg_quality = 12,
    .fb_count = 2,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    
};

const int SDMMC_CLK = CONFIG_SDMMC_CLK; 
const int SDMMC_CMD = CONFIG_SDMMC_CMD;
const int SDMMC_DATA = CONFIG_SDMMC_DATA;
const int SD_CD_PIN = CONFIG_SDMMC_CD;


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

int config_alarmstate = CONFIG_DEFAULT_ALARMSTATE ;
String config_sms = CONFIG_DEFAULT_SMS;
String config_smsc = CONFIG_DEFAULT_SMSC; 
int config_delay = CONFIG_DEFAULT_DELAY ; 


RTC_DATA_ATTR int bootCount = 0;

#define uS_TO_S_FACTOR 1000000ULL 
#define TIME_TO_SLEEP  10   
#define TIME_TO_SLEEP_REBOOT  10   

CRGB leds[CONFIG_NUM_LEDS];

String base64data = ""; 

static esp_err_t init_camera(void)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        SerialMon.println( "Camera Init Failed");
        return err;
    }
    SerialMon.println( "Camera Init OK");
    return ESP_OK;
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     SerialMon.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     SerialMon.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    SerialMon.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: SerialMon.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:      SerialMon.println("Wakeup caused by ULP program"); break;
    default:                        SerialMon.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
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
          SerialMon.print(rev);
          SerialMon.println("Got OK!");
          return true;
        }
      }
    }
  }
  SerialMon.print(rev);
  SerialMon.println("Timeout!");
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

        SerialMon.print("Trying baud rate ");
        SerialMon.print(rate);
        SerialMon.println("...");
        SerialAT.updateBaudRate(rate);
        delay(10);
        for (int j = 0; j < 10; j++) {
            SerialAT.print("AT\r\n");
            String input = SerialAT.readString();
            if (input.indexOf("OK") >= 0) {
                SerialMon.print("Modem responded at rate ");
                SerialMon.println(rate);
                SerialMon.println(input);
                return rate;
            }
        }
    }
    return 0;
}

String SendGETMessage (String uri) {
    HttpClient http = HttpClient (client, server, port) ; 
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
    HttpClient http = HttpClient (client, server, port) ; 
    http.connectionKeepAlive();  // Currently, this is needed for HTTPS
    http.post(uri, contentType, data);

    // read the status code and body of the response
    int statusCode = http.responseStatusCode();
    String response = http.responseBody();

    SerialMon.print("Status code: ");
    SerialMon.println(statusCode);
    SerialMon.print("Response: ");
    SerialMon.println(response);
    return response ; 

}


String SendImage ()
{
    String getAll;
    String getBody;
    String uri = "/upload"; 
 
    String fileType = "image/jpeg";
    String content = "--boundary1\r\n";
    content += "Content-Disposition: form-data; name=\"fileToUpload\"; filename="+String("img.jpg")+"\r\n";  // the fileToUpload is the form parameter
    content += "Content-Type: "+fileType+"\r\n";

    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();
    int32_t imageLen = fb->len;

    String closingContent = "\r\n--boundary1--\r\n";


    if (client.connect(server,port)) {

        client.println(String("POST ") + uri +" HTTP/1.1"); // or wherever you need to call/print to.
        client.println(String("Host: ") + server);
        client.println("Content-Length: "+String(content.length()+imageLen+closingContent.length()));
        client.println("Content-Type: multipart/form-data; boundary=boundary1");
        client.println();
        client.println(content);
  
        uint8_t *fbBuf = fb->buf;
        size_t fbLen = fb->len;
        for (size_t n=0; n<fbLen; n=n+1024) {
            if (n+1024 < fbLen) {
                client.write(fbBuf, 1024);
                fbBuf += 1024;
            }
            else if (fbLen%1024>0) {
                size_t remainder = fbLen%1024;
                client.write(fbBuf, remainder);
            }
        }   
        client.print(closingContent);



        
        int timoutTimer = 10000;
        long startTimer = millis();
        boolean state = false;
        
        while ((startTimer + timoutTimer) > millis()) {
        SerialMon.print(".");
        delay(100);      
        while (client.available()) {
            char c = client.read();
            if (c == '\n') {
            if (getAll.length()==0) { state=true; }
            getAll = "";
            }
            else if (c != '\r') { getAll += String(c); }
            if (state==true) { getBody += String(c); }
            startTimer = millis();
        }
        if (getBody.length()>0) { break; }
        }
        SerialMon.println();
        client.stop();
        SerialMon.println(getBody);
    }
    esp_camera_fb_return(fb);


    return (getBody); 


 }

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    SerialMon.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root) {
        SerialMon.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        SerialMon.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            SerialMon.print("  DIR : ");
            SerialMon.println(file.name());
            if (levels) {
                listDir(fs, file.path(), levels - 1);
            }
        } else {
            SerialMon.print("  FILE: ");
            SerialMon.print(file.name());
            SerialMon.print("  SIZE: ");
            SerialMon.println(file.size());
        }
        file = root.openNextFile();
    }
}


void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path ,  char  *  buffer  ){
    Serial.printf("Reading file: %s\n", path);
    strcpy(buffer, "") ; 
    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        char sztmp[2] ; 
        sztmp[0] = file.read(); 
        sztmp[1] = 0 ; 
       strcat (buffer, sztmp);
    }

}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}


bool testFile(fs::FS &fs, const char * path){
    File file = fs.open(path);
    bool present = false ; 
    if(file){
        present = true ; 
        size_t len = file.size();
        SerialMon.printf("%u bytes  \n", len);
        file.close();
    } else {
        SerialMon.println("Failed to open file");
    }
    return (present);
}

void setup() {
    char configbuffer[300] = {0} ; 
    FastLED.addLeds<WS2812B, CONFIG_DATA_PIN, RGB>(leds, CONFIG_NUM_LEDS); 
    leds[0] = CRGB::Red;
    FastLED.show();

    SerialMon.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }  
        
    pinMode(SD_CD_PIN, OUTPUT);
    digitalWrite(SD_CD_PIN,LOW) ; 
    delay (1000); 
    digitalWrite(SD_CD_PIN,HIGH) ; 
    delay(2000);


    SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA);

    if (!SD_MMC.begin("/sdcard", true)) {
        SerialMon.println("Card Mount Failed");
        while (1) {
            SerialMon.println("Card Mounting");
            delay(1000);
        }
    }
    uint8_t cardType = SD_MMC.cardType();
    SerialMon.print("SD_MMC Card Type: ");
    if (cardType == CARD_MMC) {
        SerialMon.println("MMC");
    } else if (cardType == CARD_SD) {
        SerialMon.println("SDSC");
    } else if (cardType == CARD_SDHC) {
        SerialMon.println("SDHC");
    } else {
        SerialMon.println("UNKNOWN");
    }

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    SerialMon.printf("SD_MMC Card Size: %lluMB\n", cardSize);
    
    listDir(SD_MMC, "/", 0);

    bootCount ++;

    pinMode(CONFIG_MODEM_PWR, OUTPUT);
    digitalWrite(CONFIG_MODEM_PWR, LOW);
    delay (1000);
    digitalWrite(CONFIG_MODEM_PWR, HIGH);

    SerialAT.begin(115200,SERIAL_8N1, 17,  18 ); 
    SerialMon.println("Boot number: " + String(bootCount));
    print_wakeup_reason();


    TinyGsmAutoBaud();
    String modemInfo = modem.getModemInfo();
    SerialMon.print("Modem Info: ");
    SerialMon.println(modemInfo);

    init_camera();

    leds[0] = CRGB::Yellow;
    FastLED.show();
    
    if (!testFile (SD_MMC,"/config.txt")) {

        JsonDocument doc;
        String  JSONmessageBuffer = "" ; 


        doc["alarmon"] = config_alarmstate;
        doc["SMS"] = config_sms;
        doc["SMSC"] = config_smsc;
        doc["delay"] = config_delay;
        serializeJson(doc, JSONmessageBuffer);
        SerialMon.println(JSONmessageBuffer);
        writeFile(SD_MMC, "/config.txt", JSONmessageBuffer.c_str());
    }
    readFile(SD_MMC, "/config.txt",configbuffer) ; 
    SerialMon.println (configbuffer) ; 


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

    if (((lat != 0) && (lon !=0)) || (loopgps >= (CONFIG_TIMEOUT_GPSREADY/loopdelay))) {
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
    JsonDocument doc;

    char szchipid[9];

    SerialMon.println ("Receive config from server ");
    leds[0] = CRGB::Orange;
    FastLED.show();
    itoa(chipid, szchipid, 16);
    data = "chipid=" + String(szchipid);

    SerialMon.println("making POST request");
    Answer = SendPOSTMessage (String (ressouce_config) , "application/x-www-form-urlencoded" , data) ; 

    SerialMon.print ("config answer : ") ; SerialMon.println (Answer) ; 
    DeserializationError error =  deserializeJson(doc, Answer);
    if (error) {
      SerialMon.print(F("deserializeJson() failed: "));
      SerialMon.println(error.c_str());
    } else
    {
        int id = doc["id"];
        const char* chipid = doc["chipid"];
        int alarmon = doc["alarmon"];
        const char* SMS = doc["SMS"];
        const char* SMSC = doc["SMSC"];
        int delay = doc["delay"];

        SerialMon.printf ("id : %d - chipid : %s  - alarmon : %d  - SMS : %s - SMSC : %s\r\n ", id , chipid , alarmon , SMS, SMSC, delay) ; 

        /*Save configuration to file */

        JsonDocument doc;
        String  JSONmessageBuffer = "" ; 


        doc["alarmon"] = config_alarmstate;
        doc["SMS"] = config_sms;
        doc["SMSC"] = config_smsc;
        doc["delay"] = config_delay;
        serializeJson(doc, JSONmessageBuffer);
        SerialMon.println(JSONmessageBuffer);
        writeFile(SD_MMC, "/config.txt", JSONmessageBuffer.c_str());
    }

     dataread = true; 
  }

  if (networkready && batready && posGPSready && posGSMready && !datasend  ){
    String data = ""; 
    String uri = ""; 

    String getAll;
    String getBody;

    char szchipid[9];

    SerialMon.println("Send Data to GPRS");

    itoa(chipid, szchipid, 16);
    leds[0] = CRGB::Green;
    FastLED.show();
    data = "chipid=" + String(szchipid) + "&date=" + Date + "&time=" + Time + "&bat=" + String(batvalue,9) + "&lat=" + String(lat,9) + "&lon=" + String(lon,9) + "&speed=" + String (speed,9) +"&accuracy=" + String(accuracy,9) + "&longsm=" + String(longsm,9) + "&latgsm=" + String(latgsm,9) + "&accuracygsm=" + String(accuracygsm,9) + "&alarmon=" + String(alarmon) + "&alarmstatus=" + String(alarmstatus)  ; 

    SerialMon.println("making POST request");
    SendPOSTMessage (String (resource_record) , "application/x-www-form-urlencoded" , data) ; 
    SendImage(); 



    datasend = true ; 
  }

  if (datasend && dataread) {
 
    leds[0] = CRGB::Black;
    FastLED.show();      
    SerialMon.println("Switch in sleep mode") ; 
    SerialMon.print("delay : ") ; SerialMon.println (config_delay); 
    SerialMon.flush();
    esp_sleep_enable_ulp_wakeup(); 
    esp_sleep_enable_uart_wakeup(0);
    esp_sleep_enable_timer_wakeup(config_delay * uS_TO_S_FACTOR);
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
  SerialMon.print (" - deviceGPSready = ") ; SerialMon.println (deviceGPSready) ; 
  SerialMon.print (" - batready       = ") ; SerialMon.println (batready) ; 
  SerialMon.print (" - networkready   = ") ; SerialMon.println (networkready) ; 
  SerialMon.print (" - posGPSready    = ") ; SerialMon.println (posGPSready) ; 
  SerialMon.print (" - posGSMready    = ") ; SerialMon.println (posGSMready) ; 
  SerialMon.print (" - datasend       = ") ; SerialMon.println (datasend) ; 
  SerialMon.print (" - dataread.      = ") ; SerialMon.println (dataread) ; 

  if (SerialMon.available()) {
    char buf = SerialMon.read();
    if (buf == 'r') {
      esp_sleep_enable_timer_wakeup(1 * uS_TO_S_FACTOR);
      esp_deep_sleep_start();     
    }

  }

}
