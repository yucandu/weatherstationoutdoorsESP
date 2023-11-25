#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <BlynkSimpleEsp32.h>
#include "time.h"
#include <FastLED.h>
#include "Adafruit_SHT31.h"
#include <Average.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NonBlockingDallas.h>     
#include "Plantower_PMS7003.h"
char output[256];
Plantower_PMS7003 pms7003 = Plantower_PMS7003();

#define LED_PIN 15  //d2
#define POOLTEMP_PIN 32


Average<float> pm1Avg(6);
Average<float> pm25Avg(6);
Average<float> pm10Avg(6);
Average<float> wifiAvg(30);

OneWire oneWire(POOLTEMP_PIN);
DallasTemperature dallasTemp(&oneWire);
NonBlockingDallas sensorDs18b20(&dallasTemp); //start up the DS18 temp probes
#define TIME_INTERVAL 15000 //for DS18 probe

Adafruit_SHT31 sht31 = Adafruit_SHT31();





#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -18000;  //Replace with your GMT offset (secs)
const int daylightOffset_sec = 0;   //Replace with your daylight offset (secs)
int hours, mins, secs;

char auth[] = "X_pnRUFOab29d3aNrScsKq1dryQYdTw7"; //auth token for Blynk - this is a LOCAL token, can't be used without LAN access
char remoteAuth[] = "eT_7FL7IUpqonthsAr-58uTK_-su_GYy";
char remoteAuth2[] = "8_-CN2rm4ki9P3i_NkPhxIbCiKd5RXhK";

const char* ssid = "mikesnet";
const char* password = "springchicken";

float old1p0, old2p5, old10, new1p0, new2p5, new10;
float old1p0a, old2p5a, old10a, new1p0a, new2p5a, new10a;
unsigned int up3, up5, up10, up25, up50, up100;
float abshumBME, tempBME, presBME, humBME, ds18temp, gasBME, tempPool;
int firstvalue = 1;
float bridgedata, windbridgedata, windmps, winddir;
double windchill;
float dewpoint = 0;
float humidex = 0;
unsigned long lastmillis = 0;
unsigned long millisBlynk = 0;
unsigned long millisAvg = 0;
unsigned long deltamillis = 0;
unsigned long rgbmillis = 0;
int zebraR, zebraG, zebraB;
int sliderValue = 128;
int menuValue = 2;
float  pmR, pmG, pmB;
bool rgbON = true;

AsyncWebServer server(80);

WidgetTerminal terminal(V19); //terminal widget
WidgetBridge bridge1(V60);
WidgetBridge bridge2(V70);


BLYNK_WRITE(V21)
{
   menuValue = param.asInt(); // assigning incoming value from pin V1 to a variable
}

BLYNK_WRITE(V34)
{
   sliderValue = param.asInt(); // assigning incoming value from pin V1 to a variable
}

BLYNK_WRITE(V18)
{
     zebraR = param[0].asInt();
     zebraG = param[1].asInt();
     zebraB = param[2].asInt();
}

BLYNK_WRITE(V19)
{
    if (String("help") == param.asStr()) 
    {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("temps");
    terminal.println("wets");
    terminal.println("particles");
     terminal.println("==End of list.==");
    }
        if (String("wifi") == param.asStr()) 
    {
        terminal.print("> Connected to: ");
        terminal.println(WiFi.SSID());
        terminal.print("IP address:");
        terminal.println(WiFi.localIP());
        terminal.print("Signal strength: ");
        terminal.println(WiFi.RSSI());
    }

    if (String("temps") == param.asStr()) {
        terminal.print("> tempBME[v0],tempPool[v5],humidex[v17],dewpoint[v2]: ");
        terminal.print(tempBME);
        terminal.print(",");
        terminal.print(tempPool);
        terminal.print(",");
        terminal.print(humidex);
        terminal.print(",");
        terminal.println(dewpoint);
    }
    if (String("wets") == param.asStr()) {
        terminal.print("> humBME[v3],abshumBME[v4]: ");
        terminal.print(humBME);
        terminal.print(",");
        terminal.print(abshumBME);

    }
    if (String("particles") == param.asStr()) {
     

        sprintf(output, "\nSensor Version: %d    Error Code: %d\n",
                      pms7003.getHWVersion(),
                      pms7003.getErrorCode());
        terminal.print(output);

        sprintf(output, "    PM1.0 (ug/m3): %2d     [atmos: %d]\n",
                      pms7003.getPM_1_0(),
                      pms7003.getPM_1_0_atmos());              
        terminal.print(output);
        sprintf(output, "    PM2.5 (ug/m3): %2d     [atmos: %d]\n",
                      pms7003.getPM_2_5(),
                      pms7003.getPM_2_5_atmos());
        terminal.print(output);
        sprintf(output, "    PM10  (ug/m3): %2d     [atmos: %d]\n",
                      pms7003.getPM_10_0(),
                      pms7003.getPM_10_0_atmos());              
        terminal.print(output);

        sprintf(output, "\n    RAW: %2d[>0.3] %2d[>0.5] %2d[>1.0] %2d[>2.5] %2d[>5.0] %2d[>10]\n",
                      pms7003.getRawGreaterThan_0_3(),
                      pms7003.getRawGreaterThan_0_5(),
                      pms7003.getRawGreaterThan_1_0(),
                      pms7003.getRawGreaterThan_2_5(),
                      pms7003.getRawGreaterThan_5_0(),
                      pms7003.getRawGreaterThan_10_0());
        terminal.print(output);
      

    
    }

    terminal.flush();

}

BLYNK_WRITE(V51){
    float pinData = param.asFloat();
    bridgedata = pinData;
}

BLYNK_WRITE(V56){
    float pinData = param.asFloat();
    windbridgedata = pinData;
    windmps = windbridgedata / 3.6;
}

BLYNK_WRITE(V58){
    int pinData = param.asInt();
winddir = pinData;
}

String windDirection(int temp_wind_deg)   //Source http://snowfence.umn.edu/Components/winddirectionanddegreeswithouttable3.htm
{
  switch(temp_wind_deg){
    case 0 ... 11:
      return "N";
      break;
    case 12 ... 33:
      return "NNE";
      break;
    case 34 ... 56:
      return "NE";
      break;
    case 57 ... 78:
      return "ENE";
      break;
    case 79 ... 101:
      return "E";
      break;
    case 102 ... 123:
      return "ESE";
      break;
    case 124 ... 146:
      return "SE";
      break;
    case 147 ... 168:
      return "SSE";
      break;
    case 169 ... 191:
      return "S";
      break;
    case 192 ... 213:
      return "SSW";
      break;
    case 214 ... 236:
      return "SW";
      break;
    case 237 ... 258:
      return "WSW";
      break;
    case 259 ... 281:
      return "W";
      break;
    case 282 ... 303:
      return "WNW";
      break;
    case 304 ... 326:
      return "NW";
      break;
    case 327 ... 348:
      return "NNW";
      break;
    case 349 ... 360:
      return "N";
      break;
    default:
      return "error";
      break;
  }
}

void handleTemperatureChange(float temperature, bool valid, int deviceIndex){
  tempPool = temperature;
}

void printLocalTime()
{
  time_t rawtime;
  struct tm * timeinfo;
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  terminal.print("-");
  terminal.print(asctime(timeinfo));
  terminal.println(" - ");
  terminal.flush();
}

void readPMS(void){
     if (pms7003.hasNewData()) {

        pm1Avg.push(pms7003.getPM_1_0());
        pm25Avg.push(pms7003.getPM_2_5());
        pm10Avg.push(pms7003.getPM_10_0());
  
       up3 = pms7003.getRawGreaterThan_0_3();
       up5 = pms7003.getRawGreaterThan_0_5();
       up10 = pms7003.getRawGreaterThan_1_0();
       up25 = pms7003.getRawGreaterThan_2_5();
       up50 = pms7003.getRawGreaterThan_5_0();
       up100 = pms7003.getRawGreaterThan_10_0();

  }
}

void setup() {
  pinMode(POOLTEMP_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, 3, 1);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  sht31.begin(0x44);
  pms7003.init(&Serial1);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    leds[0] = CRGB(100, 100, 0);
    FastLED.show();
    delay(250);
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
    delay(250);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
    leds[0] = CRGB(0, 100, 0);
    FastLED.show();
    delay(1000);
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
    Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
    Blynk.connect();
  sensorDs18b20.begin(NonBlockingDallas::resolution_12, NonBlockingDallas::unit_C, TIME_INTERVAL);
  sensorDs18b20.onTemperatureChange(handleTemperatureChange);
  tempBME = sht31.readTemperature();
   humBME = sht31.readHumidity();
       configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    delay(250);
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    hours = timeinfo.tm_hour;
    mins = timeinfo.tm_min;
    secs = timeinfo.tm_sec;
    terminal.println("********************************");
    terminal.println("BEGIN OUTDOOR WEATHER STATION v3.3");
    terminal.print("Connected to: ");
    terminal.println(WiFi.SSID());
    terminal.print("IP address:");
    terminal.println(WiFi.localIP());
    terminal.print("Signal strength: ");
    terminal.println(WiFi.RSSI());
    printLocalTime();
}

BLYNK_CONNECTED() {
  bridge1.setAuthToken (remoteAuth);
  bridge2.setAuthToken (remoteAuth2);
}


void loop() {
  sensorDs18b20.update();
      pms7003.updateFrame();
    readPMS();
    if (WiFi.status() == WL_CONNECTED) {Blynk.run();}  // put your main code here, to run repeatedly:
    if  (millis() - millisAvg >= 1000)  //if it's been 1 second
    {
        wifiAvg.push(WiFi.RSSI());
        millisAvg = millis();

        pmG = 55 - pm25Avg.mean();
        if (pmG < 0) {pmG = 0;}
        pmG *= (255.0/55.0);
        if (pmG > 255) {pmG = 255;}
        
        pmR = pm25Avg.mean();
        if (pmR < 0) {pmR = 0;}
        pmR *= (255.0/55.0);
        if (pmR > 255) {pmR = 255;}
        
        pmB = pm25Avg.mean() - 100;
        if (pmB < 0) {pmB = 0;}
        pmB *= (255.0/55.0);
        if (pmB > 255) {pmB = 255;}

        if (menuValue == 1) 
        {
        leds[0] = CRGB(0, 0, 0);
        }
        
        if (menuValue == 2) 
        {
        leds[0] = CRGB(pmR, pmG, pmB);
        }

        if (menuValue == 3) 
        {
        leds[0] = CRGB(zebraR, zebraG, zebraB);
        }
        FastLED.setBrightness(sliderValue);
        FastLED.show();
    }

    if  (millis() - millisBlynk >= 30000)  //if it's been 30 seconds 
    {
        millisBlynk = millis();

        

        tempBME = sht31.readTemperature();
        humBME = sht31.readHumidity();
        abshumBME = (6.112 * pow(2.71828, ((17.67 * tempBME)/(tempBME + 243.5))) * humBME * 2.1674)/(273.15 + tempBME); //calculate absolute humidity
        dewpoint = tempBME - ((100 - humBME)/5); //calculate dewpoint
        humidex = tempBME + 0.5555 * (6.11 * pow(2.71828, 5417.7530*( (1/273.16) - (1/(273.15 + dewpoint)) ) ) - 10); //calculate humidex using Environment Canada formula
        windchill  = 13.12 + (0.6215*tempBME) - (11.37*pow(windbridgedata,0.16)) + (0.3965*tempBME*pow(windbridgedata,0.16));
        if ((tempBME <= 0) && (humBME <= 0)) {}
        else {
          Blynk.virtualWrite(V0, tempBME);
          Blynk.virtualWrite(V1, presBME);
          Blynk.virtualWrite(V2, dewpoint);
          Blynk.virtualWrite(V3, humBME);
          Blynk.virtualWrite(V4, abshumBME);
          Blynk.virtualWrite(V17, humidex);
          bridge1.virtualWrite(V62, tempBME);
          bridge1.virtualWrite(V63, abshumBME);
          bridge2.virtualWrite(V62, tempBME);
          bridge2.virtualWrite(V63, abshumBME);
          bridge2.virtualWrite(V64, windchill);
          bridge2.virtualWrite(V65, humidex);
        }
        if (tempPool > 0) {Blynk.virtualWrite(V5, tempPool);}
        Blynk.virtualWrite(V6, tempBME);
        Blynk.virtualWrite(V8, pm1Avg.mean());
        Blynk.virtualWrite(V9, pm25Avg.mean());
        Blynk.virtualWrite(V10, pm10Avg.mean());
        Blynk.virtualWrite(V11, up3);
        Blynk.virtualWrite(V12, up5);
        Blynk.virtualWrite(V13, up10);
        Blynk.virtualWrite(V14, up25);
        Blynk.virtualWrite(V15, up50);
        Blynk.virtualWrite(V16, up100);
        //Blynk.virtualWrite(V22, pm1aAvg.mean());
        //Blynk.virtualWrite(V23, pm25aAvg.mean());
        //Blynk.virtualWrite(V24, pm10aAvg.mean());
        Blynk.virtualWrite(V31, wifiAvg.mean());
        if ((windbridgedata == 0) && (winddir == 0)) {}
         else {
          Blynk.virtualWrite(V32, windchill);
          Blynk.virtualWrite(V33, windDirection(winddir));
        }
        if (bridgedata > 0) {Blynk.virtualWrite(V53, bridgedata);
        bridge1.virtualWrite(V61, bridgedata);}
    }
}
