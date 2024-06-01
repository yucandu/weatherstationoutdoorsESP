#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <BlynkSimpleEsp32.h>
#include "time.h"
#include <FastLED.h>
#include "Adafruit_SHT4x.h"
#include <Average.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NonBlockingDallas.h>     

#include <AP3216_WE.h>
#include <Adafruit_ADS1X15.h>

#define LED_PIN 15  //d2
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

Adafruit_ADS1115 ads;

  int16_t adc0;
  double volts0;

AP3216_WE myAP3216 = AP3216_WE();

char output[256];



#define POOLTEMP_PIN 32

  float als; 
  unsigned int prox;
  unsigned int ir;


Average<float> wifiAvg(30);

OneWire oneWire(POOLTEMP_PIN);
DallasTemperature dallasTemp(&oneWire);
NonBlockingDallas sensorDs18b20(&dallasTemp); //start up the DS18 temp probes
#define TIME_INTERVAL 4000 //for DS18 probe

Adafruit_SHT4x sht4 = Adafruit_SHT4x();
  sensors_event_t humidity, temp;


bool heater = false;



const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -18000;  //Replace with your GMT offset (secs)
const int daylightOffset_sec = 0;   //Replace with your daylight offset (secs)
int hours, mins, secs;

char auth[] = "X_pnRUFOab29d3aNrScsKq1dryQYdTw7"; //auth token for Blynk - this is a LOCAL token, can't be used without LAN access
char remoteAuth[] = "pO--Yj8ksH2fjJLMW6yW9trkHBhd9-wc";  //costello auth
char remoteAuth2[] = "8_-CN2rm4ki9P3i_NkPhxIbCiKd5RXhK"; //hubert clock auth
char remoteAuth3[] = "qS5PQ8pvrbYzXdiA4I6uLEWYfeQrOcM4"; //indiana clock auth
char remoteAuth4[] = "19oL8t8mImCdoUqYhfhk6DADL7540f8s"; //TDSMeter  auth


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
int luxrange = 1;
float  pmR, pmG, pmB;
bool rgbON = true;

AsyncWebServer server(80);

WidgetTerminal terminal(V19); //terminal widget
WidgetBridge bridge1(V60);
WidgetBridge bridge2(V70);
WidgetBridge bridge3(V80);
WidgetBridge bridge4(V90);


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
    terminal.println("heater");
    terminal.println("reset");
    terminal.println("tds");
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
                sht4.getEvent(&humidity, &temp);
          tempBME = temp.temperature;
          humBME = humidity.relative_humidity;
        terminal.print("> tempBME[v0],tempPool[v5],humidex[v17],dewpoint[v2]: ");
        terminal.print(tempBME);
        terminal.print(",");
        terminal.println(tempPool);
        als = myAP3216.getAmbientLight();
        prox = myAP3216.getProximity();
        ir = myAP3216.getIRData(); // Ambient IR light
        setluxrange(); //RANGE_20661 (default), RANGE_  RANGE_5162, RANGE_1291, RANGE_323
        myAP3216.setMode(AP3216_ALS_PS_ONCE);
        terminal.print(">als, prox, ir: ");
        terminal.print(als);
        terminal.print(",");
        terminal.print(prox);
        terminal.print(",");
        terminal.println(ir);       
    }
    if (String("wets") == param.asStr()) {
        terminal.print("> humBME[v3],abshumBME[v4]: ");
        terminal.print(humBME);
        terminal.print(",");
        terminal.print(abshumBME);

    }

    if (String("heater") == param.asStr()) {
        heater = !heater;
      terminal.print("> Heater is now: ");
      terminal.print(heater);
    }
  if (String("reset") == param.asStr()) {
    terminal.println("Restarting...");
    terminal.flush();
    ESP.restart();
  }
  if (String("tds") == param.asStr()) {
    adc0 = ads.readADC_SingleEnded(0);
    terminal.print("ADC0: ");
    terminal.println(adc0);    
  }
    terminal.flush();

}

BLYNK_WRITE(V51){
    float pinData = param.asFloat();
    bridgedata = pinData; //bridgedata is pm25data
}

BLYNK_WRITE(V56){
    float pinData = param.asFloat();
    windbridgedata = pinData;
    windmps = windbridgedata / 3.6;
}

float windgust;

BLYNK_WRITE(V57){
    float pinData = param.asFloat();
windgust = pinData;
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

void handleTemperatureChange(int deviceIndex, int32_t temperatureRAW){
  tempPool = sensorDs18b20.rawToCelsius(temperatureRAW);
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



void setluxrange(){
  if (luxrange == 4){
    if (als < 5160) {luxrange--;}
  }

  if (luxrange == 3){
    if (als > 5160) {luxrange++;}
    if (als < 1289) {luxrange--;}
  }

  if (luxrange == 2){
    if (als > 1289) {luxrange++;}
    if (als < 321) {luxrange--;}
  }

  if (luxrange == 1){
    if (als > 321) {luxrange++;}
  }

  if (als < 2) {luxrange = 1;}

  if (luxrange == 4) {myAP3216.setLuxRange(RANGE_20661);} //RANGE_20661 (default), RANGE_  RANGE_5162, RANGE_1291, RANGE_323
  if (luxrange == 3) {myAP3216.setLuxRange(RANGE_5162);}
  if (luxrange == 2) {myAP3216.setLuxRange(RANGE_1291);}
  if (luxrange == 1) {myAP3216.setLuxRange(RANGE_323);}
}

void setup() {
  ads.begin();
  ads.setGain(GAIN_TWO);
  pinMode(POOLTEMP_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  myAP3216.init();
  myAP3216.setLuxRange(RANGE_323); //myAP3216.setLuxRange(RANGE_20661);
  myAP3216.setPSIntegrationTime(1);
  myAP3216.setMode(AP3216_ALS_PS_ONCE);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
sht4.begin();
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);
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
  sensorDs18b20.begin(NonBlockingDallas::resolution_12, TIME_INTERVAL);
  sensorDs18b20.onTemperatureChange(handleTemperatureChange);
          sht4.getEvent(&humidity, &temp);
          tempBME = temp.temperature;
          humBME = humidity.relative_humidity;
       configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    delay(250);
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    hours = timeinfo.tm_hour;
    mins = timeinfo.tm_min;
    secs = timeinfo.tm_sec;
    terminal.println("********************************");
    terminal.println("BEGIN OUTDOOR WEATHER STATION v3.6");
    terminal.print("Connected to: ");
    terminal.println(WiFi.SSID());
    terminal.print("IP address:");
    terminal.println(WiFi.localIP());
    terminal.print("Signal strength: ");
    terminal.println(WiFi.RSSI());
    printLocalTime();
    terminal.flush();
        leds[0] = CRGB(0, 100, 0);
        FastLED.show();
}

BLYNK_CONNECTED() {
  bridge1.setAuthToken (remoteAuth);
  bridge2.setAuthToken (remoteAuth2);
  bridge3.setAuthToken (remoteAuth3);
  bridge4.setAuthToken (remoteAuth4);
}


void loop() {
  sensorDs18b20.update();
    if (WiFi.status() == WL_CONNECTED) {Blynk.run();}  // put your main code here, to run repeatedly:
    if  (millis() - millisAvg >= 2000)  //if it's been 1 second
    {
        wifiAvg.push(WiFi.RSSI());
        millisAvg = millis();

    }

    if  (millis() - millisBlynk >= 30000)  //if it's been 30 seconds 
    {
        millisBlynk = millis();
        als = myAP3216.getAmbientLight();
        prox = myAP3216.getProximity();
        ir = myAP3216.getIRData(); // Ambient IR light
        setluxrange();
        myAP3216.setMode(AP3216_ALS_PS_ONCE);
        

          sht4.getEvent(&humidity, &temp);
          tempBME = temp.temperature;
          humBME = humidity.relative_humidity;
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
          bridge1.virtualWrite(V73, tempBME);
          //bridge1.virtualWrite(V63, abshumBME);
          bridge2.virtualWrite(V62, tempBME);
          bridge2.virtualWrite(V63, abshumBME);
          bridge2.virtualWrite(V64, windchill);
          bridge2.virtualWrite(V65, humidex);
          
          bridge3.virtualWrite(V61, tempPool);
          bridge4.virtualWrite(V61, tempPool);
          bridge4.virtualWrite(V62, tempBME);
          bridge4.virtualWrite(V63, humBME);
          bridge3.virtualWrite(V62, tempBME);
          bridge3.virtualWrite(V63, dewpoint);
          bridge3.virtualWrite(V64, windchill);
          bridge3.virtualWrite(V65, humidex);
          bridge3.virtualWrite(V66, windgust);
          bridge3.virtualWrite(V67, bridgedata);
          bridge3.virtualWrite(V68, windbridgedata);
          bridge3.virtualWrite(V69, winddir); 
        }
        if (tempPool > 0) {Blynk.virtualWrite(V5, tempPool);}
        Blynk.virtualWrite(V6, tempBME);

        Blynk.virtualWrite(V31, wifiAvg.mean());
        if ((windbridgedata == 0) && (winddir == 0)) {}
         else {
          Blynk.virtualWrite(V32, windchill);
          Blynk.virtualWrite(V33, windDirection(winddir));
        }
        if (bridgedata > 0) {Blynk.virtualWrite(V53, bridgedata);
        bridge1.virtualWrite(V61, bridgedata);}
        Blynk.virtualWrite(V35, als);
        Blynk.virtualWrite(V36, prox);
        Blynk.virtualWrite(V37, ir);  
        Blynk.virtualWrite(V38, luxrange);
        adc0 = ads.readADC_SingleEnded(0);
        volts0 = ads.computeVolts(adc0);

        float compensationCoefficient = 1.0+0.02*(tempPool-25.0);
        float compensationVoltage = volts0/compensationCoefficient;
              
        //convert voltage value to tds value
        float tdsValue = (133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
        float tdsuncompValue = (133.42*volts0*volts0*volts0 - 255.86*volts0*volts0 + 857.39*volts0)*0.5;
        Blynk.virtualWrite(V41, adc0);
        Blynk.virtualWrite(V42, volts0);
        Blynk.virtualWrite(V43, tdsValue);
        Blynk.virtualWrite(V44, tdsuncompValue);
    }
}
