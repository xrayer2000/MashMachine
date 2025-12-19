#include <Arduino.h>
#include <Wire.h> //temperatur
#include <Adafruit_GFX.h> //oled
#include <Adafruit_SSD1306.h> //oled
#include <U8g2lib.h> //oled
#include <Adafruit_I2CDevice.h>
#include <PID_v1.h> //PID
#include <PressButton.h> //Interface
#include <RotaryEncoderAccel.h> //Interface
#include <EEPROM.h> //Save Settings
#include <WiFi.h> //Home assistant
#include <PubSubClient.h> //Home assistant
#include "Privates.h" //Homeassistant 
#include <ArduinoJson.h> //Homeassistant
#include <math.h>
#include <MAX6675.h>
#include <OneWire.h>
#include <DallasTemperature.h> //temperatur
#include <Adafruit_ADS1X15.h> //ADC
#include <HTTPClient.h>      //InfluxDB
//-----------------------------------------------------------------------
// OLED display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
//-----------------------------------------------------------------------
//IO-pins
#define dutyCycleOutPin 26 //    green LED
#define ledOnPin 14 //           orange LED
#define ledWaterDetectedPin 4 // red LED
#define ledWifiPin 17 //         blue LED
#define ledMqttPin 23 //         yellow LED
#define VoutPin 35
//rotary encoder
#define outputA 13 //pin 13 ESP32
#define outputB 16 //pin 16 ESP32
#define confirmBtnPin 25 // pin 25 ESP32 - ok
// // MAX6675 Thermocouple
// #define MAX6675_CS 5 //pin 4 ESP32 - MAX6675 chip select
// #define MAX6675_SO 19 //pin 2 ESP32 - MAX6675 serial out
// #define MAX6675_SCK 18 //pin 15 ESP32 - MAX6675 serial clock
//NTC Thermistor
//#define NTC_PIN 34 //adc1 ads1115
//Pump
#define pumpPin 27 //pin 27 ESP32 - Pump
//Larm
#define alarmPin 19 //pin 33 ESP32 - alarm
//
#define ONE_WIRE_BUS 18 //pin 32 ESP32 - OneWire bus
//Water indicator
//#define waterDetectedPin adc2 ads1115
#define waterDetectedPin 34
//-----------------------------------------------------------------------
//Settings
#define DISP_ITEM_ROWS 4
#define DISP_CHAR_WIDTH 16
#define PACING_MC 30 //25
#define FLASH_RST_CNT 3 //30
#define SETTINGS_CHKVAL 3647 
#define CHAR_X SCREEN_WIDTH/DISP_CHAR_WIDTH // 240/16
#define CHAR_Y SCREEN_HEIGHT/DISP_ITEM_ROWS // 240/8
#define SHIFT_UP 0 //240/8
//PID
//--------------------------------------------------------------
#define P_ON_M 0
#define P_ON_E 1
//-----------------------------------------------------------------------
//Varibles

//Homeassistant
//-----------------------------------------------------------------------
Privates privates; 
WiFiClient espClient; 
PubSubClient client(espClient); 
char messages[50]; 
volatile  bool TopicArrived = false;
const     int mqttpayloadSize = 10;
char mqttpayload [mqttpayloadSize] = {'\0'};
String mqtttopic;
unsigned long lastMqttReconnectAttempt = 0;
const unsigned long mqttReconnectInterval = 5000; // 5 seconds
//-----------------------------------------------------------------------
//rotary encoder
RotaryEncoderAccel encoder(outputA, outputB);  // GPIO16 och GPIO17 på ESP32
// ISR för båda pins, som anropas vid ändring (rising/falling)
void IRAM_ATTR handleInterrupt() {
  encoder.tick();
}
//-----------------------------------------------------------------------
//Buttons
PressButton btnOk(confirmBtnPin);
//-----------------------------------------------------------------------
//Menu structure
enum pageType{
  MENU_ROOT,
  MENU_TARGET_TEMP,
  MENU_MISC,
  MENU_PID,
  MENU_MASH_PROGRAM,
  MENU_HOPS_PROGRAM

};
enum pageType currPage = MENU_ROOT;
void page_MenuRoot();
void page_MENU_TARGET_TEMP();
void page_MENU_MISC();
void page_MENU_PID();
void page_MENU_MASH_PROGRAM();
void page_MENU_HOPS_PROGRAM();
//-----------------------------------------------------------------------
//Menu internals
boolean updateAllItems = true;
boolean updateItemValue;
uint8_t itemCnt;
int8_t cursorPos;
uint8_t saveCursorPos;
uint8_t dispOffset;
uint8_t saveDispOffset;
bool edditing = false;
bool detectedRotation = false;
uint8_t root_pntrPos = 1;
uint8_t root_dispOffset = 0;
uint8_t flashCntr;
boolean flashIsOn;
void initMenuPage(String title, uint8_t itemCount);
void captureButtonDownStates();
void incrementDecrementFloat(float *v, float amount, float min, float max);
void incrementDecrementDouble(double *v, double amount, double min, double max);
void doPointerNavigation();
bool menuItemPrintable(uint8_t xPos, uint8_t yPos);
//-----------------------------------------------------------------------
//Print tools
void printPointer();
void printOnOff(bool val);
void printInt32_tAtWidth(int32_t value, uint8_t width, char c);
void printDoubleAtWidth(double value, uint8_t width, char c, uint8_t decimals = 1);
//-----------------------------------------------------------------------
//Settings
#pragma pack(1) //memory alignment
struct Mysettings{
  double Kp_mash = 250.0;
  double Ki_mash = 0.0;
  double targetTemp = 25;
  double k_heatLoss = 1.0;
  bool POnE_mash = true;

  boolean autoModeMash = 1;
  boolean manualModeHops = 1;

  int16_t mashTemps[3] = {66,76,78};
  int16_t mashTimes[3] = {60,15,15};

  int16_t hopTimes[3] = {60, 15, 1};

  int16_t timeBeforeDisable = 2;

  boolean power = true;
  boolean pump = false;
  int16_t callibrationCold = -2;
  int16_t callibrationHot = 115;
  int16_t maxElementTemp = 100; //220

  double filter_adc1 = 0.95f;
  double filterDC = 0.90f;

  int16_t alarmVolume = 20;
  int16_t alarmTime = 5;

  int16_t waterSensorThreshold = 20000;
  int16_t waterSensorOffset = 75;

  uint16_t settingsCheckValue = SETTINGS_CHKVAL;
};

Mysettings settings;
Mysettings oldSettings;
void sets_SetDeafault();
void sets_Load();
void sets_Save();
//-----------------------------------------------------------------------
//Time
unsigned long previousTime = 0; 
unsigned long currentTime;
double tS;
long passedTimeS;
long previousPassedTimeS;
unsigned long previousSensorRead = 0;
const unsigned long saveInterval = 60000; // 60 000 ms = 60 sekunder
unsigned long lastEditTime = 0;
//-----------------------------------------------------------------------
//PID
double Output_mashTemp;
double current_mashTemp;
double previous_mashTemp;
double current_airTemp;
double previous_airTemp;
double DutyCycle = 0;
double previousDutyCycle = 0;
//                 y(t), u(t), r(t) 
PID PID_mashTemp(&current_mashTemp, &Output_mashTemp, &settings.targetTemp, &current_airTemp, settings.Kp_mash, settings.Ki_mash, 0, 0, DIRECT); //P_ON_M
//-----------------------------------------------------------------------
// setting PWM properties
int freq = 1000; //5
const int ledChannel = 0;
const int resolution = 12;
//-----------------------------------------------------------------------
// Oled - Adafruit_SSD1306
U8G2_SH1106_128X64_NONAME_F_HW_I2C display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C display2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
u16_t timeLastTouched = 0;
//-----------------------------------------------------------------------
//NTC Thermistor
const double Vref = 3.27; //power supply voltage (3.3 V rail) -STM32 ADC pin is NOT 5 V tolerant
double Vout; //Voltage divider output
double R_NTC = 104300.0; //NTC thermistor, Resistance at 25 °C
const double R_ref = 101900.0; //100k resistor measured resistance in Ohms (other element in the voltage divider)
const double BETA = 3950.0; //B-coefficient of the thermistor
const double T_0 = 298.15; //25°C in Kelvin
double Temp_C; //Temperature measured by the thermistor (Celsius)
float adc1_raw_volt = 1.5;
float filteredAdc1_volt = adc1_raw_volt;
unsigned long lastReadTime = 0;
const unsigned long readInterval = 250;  // ms
//-----------------------------------------------------------------------
double passedTime,previousPassedTime1,previousPassedTime2 = 0;
bool initPage = true;
bool changeValue = false;
bool changeValues [10];
//-----------------------------------------------------------------------
//MAX6675 Thermocouple
// MAX6675 thermoCouple(MAX6675_CS, MAX6675_SO, MAX6675_SCK);
// float temp = 0;
//-----------------------------------------------------------------------
//Dallas Temperature
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dallasTemp(&oneWire);
//-----------------------------------------------------------------------
//Alarm
bool hopsAlarm = false;
//-----------------------------------------------------------------------
//ADC
Adafruit_ADS1115 adc;
//
//ARX
float a1 = -0.99, a2 = 0.00, b0 = 0.01, b1 = 0.00, b2 = 0.00;
int16_t y[3] = {adc1_raw_volt, adc1_raw_volt, adc1_raw_volt};
int16_t u[3] = {adc1_raw_volt, adc1_raw_volt, adc1_raw_volt};
//-----------------------------------------------------------------------
//Water indicator
bool waterDetected = true;
int16_t adc2_raw = 10000;
int16_t filteredAdc2 = adc2_raw;
int16_t filteredAdc2_offset = adc2_raw;

void setup() {//=================================================SETUP=======================================================
  Serial.begin(115200);

  // WiFi.disconnect(true);
  // WiFi.mode(WIFI_OFF);

  //setupADC
  adc.begin();
  adc.setDataRate(RATE_ADS1115_128SPS);

  adc1_raw_volt = readADCwithAutoGain(adc, 0);
  filteredAdc1_volt = adc1_raw_volt;

  int stableCount = 0;
  const int requiredStable = 8;
  const int maxMeasurements = 40;

  delay(100);
  for(int i = 0; i < maxMeasurements && stableCount < requiredStable; i++) {
      adc1_raw_volt = readADCwithAutoGain(adc, 0);
      filteredAdc1_volt = Filter(adc1_raw_volt, filteredAdc1_volt, settings.filter_adc1);

      if(abs(filteredAdc1_volt - adc1_raw_volt) <= 0.01) stableCount++;
      else stableCount = 0;

      delay(50);
  }

  currentTime = millis();
  
  dallasTemp.begin();

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(dutyCycleOutPin, ledChannel);

  pinMode(ledOnPin, OUTPUT);
  pinMode(ledWaterDetectedPin, OUTPUT);
  pinMode(ledWifiPin, OUTPUT);
  pinMode(ledMqttPin, OUTPUT);
  pinMode(confirmBtnPin, INPUT_PULLDOWN);
  pinMode(outputA,INPUT_PULLUP);
  pinMode(outputB,INPUT_PULLUP);
  //pinMode(NTC_PIN, INPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(alarmPin, OUTPUT);
  pinMode(waterDetectedPin, INPUT);

  // Koppla interrupt på båda encoder-pins till samma ISR
  attachInterrupt(digitalPinToInterrupt(outputA), handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), handleInterrupt, CHANGE);

  Wire.begin(21, 22);           // pins only
  
  display1.setI2CAddress(0x3C << 1); 
  display1.begin();  
  display1.setBusClock(1000000);    // ask U8g2 to set speed
  display1.setFont(u8g2_font_6x10_mf);	// choose a suitable font
  display1.clearBuffer();					// clear the internal memory
  display1.setCursor(0, 0);

  display2.setI2CAddress(0x3D << 1);
  display2.begin();  
  display2.setBusClock(1000000);    // ask U8g2 to set speed
  display2.setFont(u8g2_font_6x10_mf);	// choose a suitable font  
  display2.clearBuffer();					// clear the internal memory
  display2.setCursor(0, 0);

  EEPROM.begin(sizeof(settings));
  sets_Load();

  PID_mashTemp.SetTunings(settings.Kp_mash, settings.Ki_mash, 0, settings.POnE_mash ? P_ON_E : P_ON_M); //P_ON_M
  PID_mashTemp.SetMode(AUTOMATIC);
  PID_mashTemp.Compute();
  //Thermocouple
  // SPI.begin();
  // thermoCouple.begin();
  // thermoCouple.setSPIspeed(4000000);
  // thermoCouple.setOffset(0); 
  // if (thermoCouple.read() == STATUS_OK) {
  //   temp = thermoCouple.getCelsius();
  // } 

  

  adc1_raw_volt = readADCwithAutoGain(adc, 0);
  filteredAdc1_volt = adc1_raw_volt;

  Temp_C = convertRawToCelsius(filteredAdc1_volt);
  Temp_C = roundTo(Temp_C, 2); // Round to 2 decimal place

  current_mashTemp = Temp_C;
  previous_mashTemp = Temp_C;

  setupWiFi(); //Home assistant
  setupMQTT(); //Home assistant

  WiFi.onEvent([](WiFiEvent_t event) {
    if (event == SYSTEM_EVENT_STA_DISCONNECTED) {
      if (client.connected()) {
        client.publish("esp32/wifi", "offline", true);
      }
    }
  });

}

void loop() { //=================================================LOOP=======================================================

  passedTime = millis() / 1000.0;

  if(millis()/1000.0/60.0 - timeLastTouched > settings.timeBeforeDisable)
  {
    display1.setPowerSave(1);
    display2.setPowerSave(1);
  }
  else
  {
    display1.setPowerSave(0);
    display2.setPowerSave(0);
  }
    
  const float UPDATE_INTERVAL1 = 0.10;
  const float UPDATE_INTERVAL2 = 0.01;
  bool shouldUpdate1 = (passedTime - previousPassedTime1 >= UPDATE_INTERVAL1);
  bool shouldUpdate2 = (passedTime - previousPassedTime2 >= UPDATE_INTERVAL2);
  updateSettings();
  
  if(shouldUpdate1)
  {
    updateSensorValues(); 
    updateDisp2();
    previousPassedTime1 = passedTime;
  }
    
  switch (currPage)
  {
    case MENU_ROOT: page_MenuRoot(); break;
    case MENU_TARGET_TEMP: page_MENU_TARGET_TEMP(); break;
    case MENU_MISC: page_MENU_MISC(); break;
    case MENU_PID: page_MENU_PID(); break;
    case MENU_MASH_PROGRAM: page_MENU_MASH_PROGRAM(); break;
    case MENU_HOPS_PROGRAM: page_MENU_HOPS_PROGRAM(); break;
  }

  printPointer();              
  if ((updateAllItems || updateItemValue) && shouldUpdate2)
  {
    unsigned long t0 = micros();
    display1.sendBuffer();                        //välldigt långsam
    unsigned long t1 = micros();
    //Serial.println(t1 - t0);  // microseconds

    previousPassedTime2 = passedTime;
    updateAllItems = false;
    updateItemValue = false;
  } 

  captureButtonDownStates();
  
  //Homeassistant
  if(passedTime - previousTime >= 1.0) {publishMessage(); previousTime = passedTime;}

  detectedRotation = encoder.getDirection() != RotaryEncoderAccel::Direction::NOROTATION;
  if(detectedRotation)
  {
    lastEditTime = millis();
  }

  // === WiFi reconnect ===
  if (WiFi.status() != WL_CONNECTED) {
    reconnectWiFi();
  }

  // === MQTT reconnect ===
   if (!client.connected()) {
    reconnectMQTT();
  }

  // === MQTT keep-alive ===
  client.loop();
}

void page_MenuRoot(){//=================================================ROOT_MENU============================================
  if(initPage)
  {
    cursorPos = root_pntrPos;
    dispOffset = root_dispOffset;
    
    initMenuPage(F("MAIN MENU"), 5);
    initPage = false;
  }
  doPointerNavigation();
  
  if(menuItemPrintable(1,1)){display1.print(F("TARGET TEMP        "));}
  if(menuItemPrintable(1,2)){display1.print(F("MISC               "));} 
  if(menuItemPrintable(1,3)){display1.print(F("PID                "));} 
  if(menuItemPrintable(1,4)){display1.print(F("MASH PROGRAM       "));} 
  if(menuItemPrintable(1,5)){display1.print(F("HOPS PROGRAM       "));} 

  if(btnOk.PressReleased())
  {
    FlashPointer();
    root_pntrPos = cursorPos;
    root_dispOffset = dispOffset;
    initPage = true;
    switch (cursorPos)
    {
    case 0: currPage = MENU_TARGET_TEMP; return;
    case 1: currPage = MENU_MISC; return;
    case 2: currPage = MENU_PID; return;
    case 3: currPage = MENU_MASH_PROGRAM; return;
    case 4: currPage = MENU_HOPS_PROGRAM; return;
    }
  }
}
void page_MENU_TARGET_TEMP(){//=================================================TARGET_TEMP============================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("TARGET_TEMP"), 2);
    changeValue = false;
    initPage = false;
  }

  if(menuItemPrintable(1,1)){display1.print(F("Target Temp =      "));}
  if(menuItemPrintable(1,2)){display1.print(F("Back               "));}

  if(menuItemPrintable(12,1)){printInt32_tAtWidth((uint32_t)settings.targetTemp, 3, 'C');}

  if(btnOk.PressReleased())
  {
    FlashPointer();
    
    switch (cursorPos)
    {
      case 0: changeValue = !changeValue; edditing = !edditing; break;
      case 1: currPage = MENU_ROOT; sets_Save(); initPage = true; return;
    }
  }

  if(changeValue)
  {
    incrementDecrementDouble(&settings.targetTemp, 1.0, 15.0, settings.maxElementTemp);
    settings.targetTemp = round(settings.targetTemp);
  }
  else 
    doPointerNavigation(); 
}

void page_MENU_MISC(){//=================================================MISC==========================================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("MISC"), 11);
    changeValues [10];
    initPage = false;
  }
   
  if(menuItemPrintable(1,1)){display1.print(F("POWER        =     "));}
  if(menuItemPrintable(1,2)){display1.print(F("PUMP         =     "));}
  if(menuItemPrintable(1,3)){display1.print(F("ColdCalli    =     "));}
  if(menuItemPrintable(1,4)){display1.print(F("HotCalli     =     "));}
  if(menuItemPrintable(1,5)){display1.print(F("FiltTemp  =     "));}
  if(menuItemPrintable(1,6)){display1.print(F("FiltDC       =     "));}
  if(menuItemPrintable(1,7)){display1.print(F("max_Ele_temp =     "));}
  if(menuItemPrintable(1,8)){display1.print(F("TimeToIdle   =     "));}
  if(menuItemPrintable(1,9)){display1.print(F("AlarmVolume  =     "));}
  if(menuItemPrintable(1,10)){display1.print(F("AlarmTime   =     "));}
  if(menuItemPrintable(1,11)){display1.print(F("Back              "));}

  if(menuItemPrintable(12,1)){printOnOff(settings.power);}
  if(menuItemPrintable(12,2)){printOnOff(settings.pump);}
  if(menuItemPrintable(12,3)){printInt32_tAtWidth(settings.callibrationCold, 3, ' ');}
  if(menuItemPrintable(12,4)){printInt32_tAtWidth(settings.callibrationHot, 3, ' ');}
  if(menuItemPrintable(12,5)){printDoubleAtWidth(settings.filter_adc1, 3, ' ', 2);}
  if(menuItemPrintable(12,6)){printDoubleAtWidth(settings.filterDC, 3, ' ', 2);}
  if(menuItemPrintable(12,7)){printInt32_tAtWidth(settings.maxElementTemp, 3, 'C');}
  if(menuItemPrintable(12,8)){printInt32_tAtWidth(settings.timeBeforeDisable, 3, 'm');}
  if(menuItemPrintable(12,9)){printInt32_tAtWidth(settings.alarmVolume, 3, '%');}
  if(menuItemPrintable(12,10)){printInt32_tAtWidth(settings.alarmTime, 3, 's');}
      
  if(btnOk.PressReleased())
  {
    FlashPointer();
    
    switch (cursorPos)
    {
      case 0: changeValues [0] = !changeValues [0]; edditing = !edditing; break; 
      case 1: changeValues [1] = !changeValues [1]; edditing = !edditing; break; 
      case 2: changeValues [2] = !changeValues [2]; edditing = !edditing; break; 
      case 3: changeValues [3] = !changeValues [3]; edditing = !edditing; break;
      case 4: changeValues [4] = !changeValues [4]; edditing = !edditing; break; 
      case 5: changeValues [5] = !changeValues [5]; edditing = !edditing; break;
      case 6: changeValues [6] = !changeValues [6]; edditing = !edditing; break;
      case 7: changeValues [7] = !changeValues [7]; edditing = !edditing; break;
      case 8: changeValues [8] = !changeValues [8]; edditing = !edditing; break;
      case 9: changeValues [9] = !changeValues [9]; edditing = !edditing; break;
      case 10: currPage = MENU_ROOT; sets_Save(); initPage = true; return;
    }
  }

       if(changeValues[0]){*&settings.power = !*&settings.power; changeValues [0] = false; updateItemValue = true; }
       if(changeValues[1] && waterDetected){*&settings.pump = !*&settings.pump; changeValues [1] = false; updateItemValue = true; }
  else if(changeValues[2])incrementDecrementInt(&settings.callibrationCold, 1, -50, 50);
  else if(changeValues[3])incrementDecrementInt(&settings.callibrationHot, 1, 60, 140);
  else if(changeValues[4])incrementDecrementDouble(&settings.filter_adc1, 0.01f, 0.0f, 0.99f);
  else if(changeValues[5])incrementDecrementDouble(&settings.filterDC, 0.01f, 0.0f, 0.99f);
  else if(changeValues[6])incrementDecrementInt(&settings.maxElementTemp, 1, 15, 100);
  else if(changeValues[7])incrementDecrementInt(&settings.timeBeforeDisable, 1, 1, 90);
  else if(changeValues[8])incrementDecrementInt(&settings.alarmVolume, 1, 0, 100);
  else if(changeValues[9])incrementDecrementInt(&settings.alarmTime, 1, 1, 30);
  else 
    doPointerNavigation(); 
}
void page_MENU_PID(){//=================================================PID===============================================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("PID"), 5);
    changeValues [10]; 
    initPage = false;
  }

  if(menuItemPrintable(1,1)){display1.print(F("Kp_ele     =           "));}
  if(menuItemPrintable(1,2)){display1.print(F("Ki_ele     =           "));}
  if(menuItemPrintable(1,3)){display1.print(F("k_heatLoss =           "));}
  if(menuItemPrintable(1,4)){display1.print(F("POnE       =           "));}
  if(menuItemPrintable(1,5)){display1.print(F("Back                   "));}

  if(menuItemPrintable(12,1)){printInt32_tAtWidth(settings.Kp_mash, 3, ' ');}
  if(menuItemPrintable(12,2)){printDoubleAtWidth(settings.Ki_mash, 3, ' ');}
  if(menuItemPrintable(12,3)){printDoubleAtWidth(settings.k_heatLoss, 3, ' ');}
  if(menuItemPrintable(12,4)){printOnOff(settings.POnE_mash);}
     
  if(btnOk.PressReleased())
  {
    FlashPointer();
    
    switch (cursorPos)
    {
      case 0: changeValues [0] = !changeValues [0]; edditing = !edditing; break;
      case 1: changeValues [1] = !changeValues [1]; edditing = !edditing; break;
      case 2: changeValues [2] = !changeValues [2]; edditing = !edditing; break;
      case 3: changeValues [3] = !changeValues [3]; edditing = !edditing; break;
      case 4: currPage = MENU_ROOT; sets_Save(); initPage = true; return;
    }
  }
       if(changeValues[0])incrementDecrementDouble(&settings.Kp_mash, 1.0, 0.0, 1000.0);
  else if(changeValues[1])incrementDecrementDouble(&settings.Ki_mash, 0.1, 0.0, 200.0);
  else if(changeValues[2])incrementDecrementDouble(&settings.k_heatLoss, 1.0, 0.0, 200.0);
  else if(changeValues[3]){*&settings.POnE_mash = !*&settings.POnE_mash; changeValues [3] = false; updateItemValue = true;}
  else 
    doPointerNavigation(); 
}
void page_MENU_MASH_PROGRAM(){//=================================================MASH_PROGRAM====================================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("Mash Program"), 9);
    changeValues [10];
    initPage = false;
  }

  if(menuItemPrintable(1,1)){display1.print(F("Auto Mode   =            "));}
  if(menuItemPrintable(1,3)){display1.print(F("Mash Temp1  =            "));}
  if(menuItemPrintable(1,4)){display1.print(F("Mash Time1  =            "));}
  if(menuItemPrintable(1,5)){display1.print(F("Mash Temp2  =            "));}
  if(menuItemPrintable(1,6)){display1.print(F("Mash Time2  =            "));}
  if(menuItemPrintable(1,7)){display1.print(F("Mash Temp3  =            "));}
  if(menuItemPrintable(1,8)){display1.print(F("Mash Time3  =            "));}
  if(menuItemPrintable(1,9)){display1.print(F("Back                     "));}

  if(menuItemPrintable(12,1)){printOnOff(settings.autoModeMash);}
  if(menuItemPrintable(1,2)){printDoubleAtWidth(passedTimeS/60.0, 3, 'm');}
  if(menuItemPrintable(12,2)){printInt32_tAtWidth(settings.targetTemp, 3, 'C');}
  if(menuItemPrintable(12,3)){printInt32_tAtWidth(settings.mashTemps[0], 3, 'C');}
  if(menuItemPrintable(12,4)){printInt32_tAtWidth(settings.mashTimes[0], 3, 'm');}
  if(menuItemPrintable(12,5)){printInt32_tAtWidth(settings.mashTemps[1], 3, 'C');}  
  if(menuItemPrintable(12,6)){printInt32_tAtWidth(settings.mashTimes[1], 3, 'm');}
  if(menuItemPrintable(12,7)){printInt32_tAtWidth(settings.mashTemps[2], 3, 'C');}
  if(menuItemPrintable(12,8)){printInt32_tAtWidth(settings.mashTimes[2], 3, 'm');}
   
  if(btnOk.PressReleased())
  {
    FlashPointer();
    
    switch (cursorPos)
    {
      case 0: changeValues [0] = !changeValues [0]; edditing = !edditing; break;
      case 2: changeValues [1] = !changeValues [1]; edditing = !edditing; break;
      case 3: changeValues [2] = !changeValues [2]; edditing = !edditing; break;
      case 4: changeValues [3] = !changeValues [3]; edditing = !edditing; break;
      case 5: changeValues [4] = !changeValues [4]; edditing = !edditing; break;
      case 6: changeValues [5] = !changeValues [5]; edditing = !edditing; break;
      case 7: changeValues [6] = !changeValues [6]; edditing = !edditing; break;
      case 8: currPage = MENU_ROOT; sets_Save(); initPage = true; return; 
    }
  }

  if(changeValues[0])
  {
    *&settings.autoModeMash = !*&settings.autoModeMash;
    if(settings.autoModeMash){settings.pump = true;}
    else{settings.pump = false;}
    changeValues [0] = false;
    updateItemValue = true; 
  } 
  else if(changeValues[1])incrementDecrementInt(&settings.mashTemps[0], 1, 15, settings.maxElementTemp);
  else if(changeValues[2])incrementDecrementInt(&settings.mashTimes[0], 1, 0, 90);
  else if(changeValues[3])incrementDecrementInt(&settings.mashTemps[1], 1, 15, settings.maxElementTemp);
  else if(changeValues[4])incrementDecrementInt(&settings.mashTimes[1], 1, 0, 90);
  else if(changeValues[5])incrementDecrementInt(&settings.mashTemps[2], 1, 15, settings.maxElementTemp);
  else if(changeValues[6])incrementDecrementInt(&settings.mashTimes[2], 1, 0, 90);
  else if(changeValues[7])
  {
    currPage = MENU_ROOT; 
    sets_Save(); 
    initPage = true; 
    changeValues [8] = false;
    updateItemValue = true; 
  }
  else 
    doPointerNavigation();
}

void page_MENU_HOPS_PROGRAM(){//=================================================HOPS_PROGRAM====================================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("Hops Program"), 6);
    changeValues [10];
    initPage = false;
  }

  if(menuItemPrintable(1,1)){display1.print(F("Manual =            "));}
  if(menuItemPrintable(1,3)){display1.print(F("Time1  =            "));}
  if(menuItemPrintable(1,4)){display1.print(F("Time2  =            "));}
  if(menuItemPrintable(1,5)){display1.print(F("Time3  =            "));}
  if(menuItemPrintable(1,6)){display1.print(F("Back                "));}

  if(menuItemPrintable(12,1)){printOnOff(settings.autoModeMash);}
  if(menuItemPrintable(1,2)){printDoubleAtWidth(passedTimeS/60.0, 3, 'm');}
  if(menuItemPrintable(12,2)){printInt32_tAtWidth(settings.targetTemp, 3, 'C');}
  if(menuItemPrintable(12,3)){printInt32_tAtWidth(settings.hopTimes[0], 3, 'm');}
  if(menuItemPrintable(12,4)){printInt32_tAtWidth(settings.hopTimes[1], 3, 'm');}
  if(menuItemPrintable(12,5)){printInt32_tAtWidth(settings.hopTimes[2], 3, 'm');}
   
  if(btnOk.PressReleased())
  {
    FlashPointer();
    
    switch (cursorPos)
    {
      case 0: changeValues [0] = !changeValues [0]; edditing = !edditing; break;
      case 2: changeValues [1] = !changeValues [1]; edditing = !edditing; break;
      case 3: changeValues [2] = !changeValues [2]; edditing = !edditing; break;
      case 4: changeValues [3] = !changeValues [3]; edditing = !edditing; break;
      case 5: currPage = MENU_ROOT; sets_Save(); initPage = true; return; 
    }
  }

  if(changeValues[0])
  {
    *&settings.autoModeMash = !*&settings.autoModeMash;
    changeValues [0] = false;
    updateItemValue = true; 
  } 

  else if(changeValues[1])incrementDecrementInt(&settings.hopTimes[0], 1, 0, 90);
  else if(changeValues[2])incrementDecrementInt(&settings.hopTimes[1], 1, 0, 90);
  else if(changeValues[3])incrementDecrementInt(&settings.hopTimes[2], 1, 0, 90);
  else if(changeValues[4])
  {
    currPage = MENU_ROOT; 
    sets_Save(); 
    initPage = true; 
    changeValues [3] = false;
    updateItemValue = true; 
  }
  else 
    doPointerNavigation();
}

//======================================================TOOLS - menu Internals==================================================
void initMenuPage(String title, uint8_t itemCount){
  display1.clearBuffer();
  printPointer();
  uint8_t fillCnt = (DISP_CHAR_WIDTH - title.length()) / 2;

  btnOk.ClearWasDown();
 
  itemCnt = itemCount;
  flashCntr = 0;
  flashIsOn = false;
  updateAllItems = true;
}
void captureButtonDownStates(){
  btnOk.CaptureDownState();
}

void doPointerNavigation() 
{
  int direction = round(encoder.getRPM());

  if (direction != 0) {
    int newCursorPos = cursorPos + direction;

    // Clamp
    if (newCursorPos < 0) newCursorPos = 0;
    if (newCursorPos > itemCnt - 1) newCursorPos = itemCnt - 1;

    if (newCursorPos != cursorPos) {
      cursorPos = newCursorPos;

      // Scroll logic
      if (cursorPos < dispOffset) {
        dispOffset = cursorPos;
        updateAllItems = true; // Force update of all items
      } else if (cursorPos >= dispOffset + DISP_ITEM_ROWS) {
        dispOffset = cursorPos - DISP_ITEM_ROWS + 1;
        updateAllItems = true; // Force update of all items
      }

      printPointer();  // Only redraw when view actually changes
    }
    timeLastTouched = millis()/1000.0/60.0; // Update last touched time
    
    // Serial.print("Direction: ");
    // Serial.print(direction);
    // Serial.print(",\t\tNew Cursor Position: ");
    // Serial.print(newCursorPos);
    // Serial.print(",\t\tDisplay Offset: ");
    // Serial.println(dispOffset);
  }
}
void incrementDecrementInt(int16_t *v, int16_t amount, int16_t min, int16_t max)
{
    int16_t direction = encoder.getRPM();

    if (direction != 0) {
        int16_t target = direction * amount;
        int16_t newValue = *v + target;

        if (newValue >= min && newValue <= max) {
            *v = newValue;
        }
        else if (newValue < min) {
            *v = min;
        }
        else {
            *v = max;
        }

        updateItemValue = true;
        timeLastTouched = millis()/1000.0/60.0;
    }

    delayMicroseconds(5);
}

void incrementDecrementFloat(float *v, float amount, float min, float max)
{
  float direction = encoder.getRPM();

    if (direction != 0) {
        float target = direction * amount;
        float newValue = *v + target;

        if (newValue >= min && newValue <= max) 
          *v = newValue;
        else if (newValue < min) 
          *v = min;
        else 
          *v = max;

        updateItemValue = true;
        timeLastTouched = millis()/1000.0/60.0;
    }

    delayMicroseconds(5);
}
void incrementDecrementDouble(double *v, double amount, double min, double max)
{
  double direction = encoder.getRPM();

    if (direction != 0) {
        double target = direction * amount;
        double newValue = *v + target;

        if (newValue >= min && newValue <= max) 
          *v = newValue;
        else if (newValue < min) 
          *v = min;
        else 
          *v = max;

        updateItemValue = true;
        timeLastTouched = millis()/1000.0/60.0;
    }

    delayMicroseconds(5);
}

bool isFlashChanged(){
  if(flashCntr == 0){
    flashIsOn = !flashIsOn;
    flashCntr = FLASH_RST_CNT;
    return true;
  }
  else{flashCntr--; return false;}
}

bool menuItemPrintable(uint8_t xPos, uint8_t yPos){
  if(!(updateAllItems || (updateItemValue && cursorPos == yPos))){return false;}
  uint8_t yMaxOffset = 0;
  if(yPos > DISP_ITEM_ROWS) {yMaxOffset = yPos - DISP_ITEM_ROWS;}
  if(dispOffset <= (yPos) && dispOffset >= yMaxOffset){display1.setCursor(CHAR_X*xPos, CHAR_Y*(yPos - dispOffset)); return true;}
  return false;
}

bool menuItemPrintableDisp2(uint8_t xPos, uint8_t yPos){ 
  if(!(updateAllItems || (updateItemValue && cursorPos == yPos))){return false;}
  uint8_t yMaxOffset = 0;
  if(yPos > DISP_ITEM_ROWS) {yMaxOffset = yPos - DISP_ITEM_ROWS;}
  if(0 <= (yPos) && 0 >= yMaxOffset){display2.setCursor(CHAR_X*xPos, CHAR_Y*(yPos)); return true;}
  return false;
}

//======================================================TOOLS_display========================================================
void printPointer(){
  //Serial.println("printPointer");
  display1.drawStr(0, 1*CHAR_Y, " ");
  display1.drawStr(0, 2*CHAR_Y, " ");
  display1.drawStr(0, 3*CHAR_Y, " ");
  display1.drawStr(0, 4*CHAR_Y, " ");
  display1.drawStr(0, (cursorPos - dispOffset + 1)*CHAR_Y, "*");
  display1.sendBuffer();
}
void FlashPointer(){
  timeLastTouched = millis()/1000.0/60.0;
  display1.drawStr(0, 1*CHAR_Y, " ");
  display1.drawStr(0, 2*CHAR_Y, " ");
  display1.drawStr(0, 3*CHAR_Y, " ");
  display1.drawStr(0, 4*CHAR_Y, " ");
  display1.sendBuffer();                  //nytt

  delay(100);
  //Serial.println("FlashPointer");
  display1.drawStr(0, (cursorPos - dispOffset + 1)*CHAR_Y, "*");
  display1.sendBuffer();                  //nytt
}

void printOnOff(bool val){
  if(val){display1.print(F("ON    "));}
  else   {display1.print(F("OFF   "));}
}
void printChars(uint8_t cnt, char c){
  if(cnt > 0){
    char cc[] = " "; cc[0] = c;
    for(u_int8_t i = 1; i < cnt; i++){display1.print(cc);}
  }
}
uint8_t getInt32_tCharCnt(int32_t value)
{
  if(value == 0){return 1;}
  int32_t tensCalc = 10; int8_t cnt = 1;
  while (tensCalc <= value && cnt < 20){tensCalc *= 10; cnt += 1;}
  return cnt;
}
uint8_t getDoubleCharCnt(double value)
{
  if(value == 0){return 1;}
  uint32_t tensCalc = 10; uint8_t cnt = 1;
  while (tensCalc <= value && cnt < 20){tensCalc *= 10; cnt += 1;}
  return cnt;
}
void printInt32_tAtWidth(int32_t value, uint8_t width, char c){
  display1.print(value);
  display1.print(c);
  printChars(width-getInt32_tCharCnt(value), ' ');
}
void printDoubleAtWidth(double value, uint8_t width, char c, uint8_t decimals){
  char buf[10];
  dtostrf(value, width-getDoubleCharCnt(value), decimals, buf); // 1 decimal
  display1.print(buf);
  display1.print(c);
}
//======================================================DISPLAY_2======================================================
void printCharsDisplay2(uint8_t cnt, char c){
  if(cnt > 0){
    char cc[] = " "; cc[0] = c;
    for(u_int8_t i = 1; i < cnt; i++){display2.print(cc);}
  }
}
void printInt32_tAtWidthDisplay2(int32_t value, uint8_t width, char c){
  display2.print(value);
  display2.print(c);
  printCharsDisplay2(width-getInt32_tCharCnt(value), ' ');
}
void printDoubleAtWidthDisplay2(double value, uint8_t width, char c){
  char buf[10];
  dtostrf(value, width-getDoubleCharCnt(value), 1, buf); // 1 decimal
  display2.print(buf);
  display2.print(c);
}

//======================================================TOOLS_settings======================================================
void sets_SetDeafault()
{
  Mysettings tempSets;
  memcpy(&settings, &tempSets, sizeof settings);
}

void sets_Load()
{
  EEPROM.get(0,settings);
  if(settings.settingsCheckValue != SETTINGS_CHKVAL){sets_SetDeafault();}
}
void sets_Save()
{
  if (memcmp(&settings, &oldSettings, sizeof(settings)) != 0) {
    EEPROM.put(0, settings);
    EEPROM.commit();
    oldSettings = settings; // uppdatera efter commit
  }
}

void updateSettings()
{
  if(!waterDetected || !settings.power){*&settings.pump = false;}

  if(settings.power)
  {
    digitalWrite(ledOnPin, HIGH);
    digitalWrite(ledWaterDetectedPin, !waterDetected);

    previousPassedTimeS = passedTimeS;

    bool isAuto = settings.autoModeMash;
    passedTimeS = isAuto ? (millis() - tS) / 1000.0 : 0;

    if (!isAuto) {
      tS = millis();
    }

    settings.targetTemp = !isAuto ? settings.targetTemp :
    (passedTimeS/60.0 < settings.mashTimes[0]) ? settings.mashTemps[0] :
    (passedTimeS/60.0 < settings.mashTimes[0] + settings.mashTimes[1]) ? settings.mashTemps[1] :
    (passedTimeS/60.0 < settings.mashTimes[0] + settings.mashTimes[1] + settings.mashTimes[2]) ? settings.mashTemps[2] :
    15.0;
    
    // Serial.print("PassedTimeS: ");
    // Serial.println(passedTimeS/60.0);

         if (isAuto) {hopsAlarm = false; analogWrite(alarmPin, 0);} 
    else if (passedTimeS/60.0 >= settings.hopTimes[0] && passedTimeS/60.0 < (settings.hopTimes[0] + settings.alarmTime/60.0)) {hopsAlarm = true; analogWrite(alarmPin, settings.alarmVolume /100.0 * 4095.0);} 
    else if (passedTimeS/60.0 >= settings.hopTimes[1] && passedTimeS/60.0 < (settings.hopTimes[1] + settings.alarmTime/60.0)) {hopsAlarm = true; analogWrite(alarmPin, settings.alarmVolume /100.0 * 4095.0);} 
    else if (passedTimeS/60.0 >= settings.hopTimes[2] && passedTimeS/60.0 < (settings.hopTimes[2] + settings.alarmTime/60.0)) {hopsAlarm = true; analogWrite(alarmPin, settings.alarmVolume /100.0 * 4095.0);}
    else {hopsAlarm = false; analogWrite(alarmPin, 0);}

    PID_mashTemp.SetTunings(settings.Kp_mash, settings.Ki_mash, 0, settings.POnE_mash ? P_ON_E : P_ON_M);
    PID_mashTemp.SetMode(AUTOMATIC);
    updateAllItems = PID_mashTemp.Compute();

    PID_mashTemp.SetHeatLossCoefficient(settings.k_heatLoss);

    previousDutyCycle = DutyCycle;
    DutyCycle = Output_mashTemp;  
    
    // DutyCycle = (1.0 - settings.filterDC) * DutyCycle + settings.filterDC * previousDutyCycle; //smoothing

    if(waterDetected) DutyCycle = Filter(DutyCycle, previousDutyCycle, settings.filterDC); 
    else DutyCycle = 0;
    ledcWrite(ledChannel, DutyCycle);
    // Serial.print(", DutyCycle: ");
    // Serial.print(DutyCycle);

    client.loop();
    if(TopicArrived)
    {
      // Serial.print("Message arrived: ");
      // Serial.println(mqttpayload);
      //settings.targetTemp = atof(mqttpayload);
      TopicArrived = false;
    }

    if(settings.pump)
    {
      digitalWrite(pumpPin, HIGH);
    }
    else
    {
      digitalWrite(pumpPin, LOW);
    }
  }
  else
  {
    digitalWrite(ledOnPin, LOW);
    ledcWrite(ledChannel, 0);
    digitalWrite(ledWaterDetectedPin, 0);
  }
}

void updateSensorValues() {

  unsigned long currentTime = millis();
  if (currentTime - lastReadTime >= readInterval)
  {
    lastReadTime = currentTime;

    adc1_raw_volt = readADCwithAutoGain(adc, 0);

    // adc.setGain(GAIN_TWO); // 1x gain ±2.046V
    // adc.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/false);
    // while(!adc.conversionComplete());
    // adc1_raw_volt = adc.getLastConversionResults();

    filteredAdc1_volt = Filter(adc1_raw_volt, filteredAdc1_volt, settings.filter_adc1); 
    //18200  wet (3.3v)
    //20000  dry (3.3v)
    //filtered = Filter(raw, filtered);

    Temp_C = convertRawToCelsius(filteredAdc1_volt);
    Temp_C = roundTo(Temp_C, 3); // Round to 3 decimal places

    dallasTemp.requestTemperatures(); // Request temperature readings from the sensor
    previous_airTemp = current_airTemp;
    current_airTemp = dallasTemp.getTempCByIndex(0);

    waterDetected = !digitalRead(waterDetectedPin);

    Serial.print("Voltage adc1: ");
    Serial.print(adc1_raw_volt, 4);
    Serial.print(" V");

    Serial.print(", Gain range: ±");
    Serial.print(adc.getFsRange(), 2);
    Serial.print(" V");

    Serial.print(", Water Detected: ");
    Serial.print(waterDetected ? "YES" : "NO");
    
    Serial.print(", Filtered ADC1: ");
    Serial.print(filteredAdc1_volt, 4); 

    Serial.print(", current_mashTemp: ");
    Serial.print(current_mashTemp, 1);
    Serial.println(" °C, ");

    // Serial.print(", current_airTemp:\t");
    // Serial.print(current_airTemp);
    // Serial.print(" °C, ");

    // Serial.print(", TargetTemp:\t");
    // Serial.print(settings.targetTemp);
    // Serial.print(" °C, ");
    // Serial.print(", DutyCycle:\t");
    // Serial.print(DutyCycle/4095.0 * 100.0);

    // Serial.println(", Q_ff: " + String(PID_mashTemp.Q_ff));


    previous_mashTemp = current_mashTemp;
    current_mashTemp = Temp_C;
  }

}


void updateDisp2()
{
  display2.clearBuffer();
  if(menuItemPrintableDisp2(1,1)){display2.print(F("TargetTemp =  "));}
  if(menuItemPrintableDisp2(1,2)){display2.print(F("MashTemp   =  "));}
  if(menuItemPrintableDisp2(1,3)){display2.print(F("DC         =  "));}
  if(menuItemPrintableDisp2(1,4)){display2.print(F("AirTemp    =  "));}

  if(menuItemPrintableDisp2(11,1)){printInt32_tAtWidthDisplay2(settings.targetTemp, 3, 'C');}
  if(menuItemPrintableDisp2(11,2)){printDoubleAtWidthDisplay2(current_mashTemp, 3, 'C');}
  if(menuItemPrintableDisp2(11,3)){printDoubleAtWidthDisplay2(DutyCycle/4095.0 * 100.0, 3, '%');}
  if(menuItemPrintableDisp2(11,4)){printInt32_tAtWidthDisplay2(current_airTemp, 3, 'C');}
  display2.sendBuffer();
  display2.clearBuffer();
}

void setupWiFi() // Home Assistant
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);              // IMPORTANT for stability
  WiFi.begin(privates.ssid, privates.pass);

  Serial.print("Connecting to WiFi");

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000)
  {
    delay(100);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("\nWiFi connected, IP: ");
    Serial.println(WiFi.localIP());
    digitalWrite(ledWifiPin, LOW);
  }
  else
  {
    Serial.println("\nWiFi failed, will retry in loop()");
    digitalWrite(ledWifiPin, HIGH);
  }
}


void setupMQTT() // Home Assistant
{
  client.setServer(privates.broker, 1883);
  client.setCallback(callback);
  client.setKeepAlive(30);   // IMPORTANT

  unsigned long t0 = millis();
  while (!client.connected() && millis() - t0 < 10000)
  {
    Serial.print("Connecting to MQTT... ");

    if (client.connect(
          "MashMachine",           // client ID
          privates.brokerUser,
          privates.brokerPass,
          "esp32/status",           // LWT topic
          0,                        // QoS
          true,                     // retained
          "offline"                 // LWT payload
        ))
    {
      Serial.println("connected");
      publishWifiMqttStatus();
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }

    digitalWrite(ledMqttPin, client.connected() ? LOW : HIGH);
  }

  Serial.println("MQTT setup complete");
}


void reconnectWiFi() //Homeassistant
{
  static unsigned long lastAttempt = 0;

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(ledWifiPin, LOW);
    return;
  }

  if (millis() - lastAttempt > 5000) {
    lastAttempt = millis();

    Serial.println("Trying to reconnect WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(privates.ssid, privates.pass);
  }

  digitalWrite(ledWifiPin, HIGH);
}

void reconnectMQTT() //Homeassistant
{
  if (WiFi.status() != WL_CONNECTED) return;

  static unsigned long lastAttempt = 0;

  if (client.connected()) {
    digitalWrite(ledMqttPin, LOW);
    return;
  }

  if (millis() - lastAttempt > 5000) { 
    //blocking code when trying to reconnect MQTT
    //istället för "PubSubClient" använd "AsyncMqttClient" för icke-blockerande
    lastAttempt = millis();
    Serial.println("Trying to reconnect MQTT...");

    client.setKeepAlive(30);

    if (client.connect(
          "MashMachine",
          privates.brokerUser,
          privates.brokerPass,
          "esp32/status",  // LWT topic
          0,
          true,
          "offline"
        ))
    {
      Serial.println("MQTT connected!");
      publishWifiMqttStatus();
    }
  }

  digitalWrite(ledMqttPin, HIGH);
}

// void publishFloat(const char* topic, float value, int decimals = 1) {
//   //int "%d", float "%f", bool "%s", obs vikitgt!
//   char format[10];
//   snprintf(format, sizeof(format), "%%.%df", decimals);
//   snprintf(messages, sizeof(messages), format, value);
//   client.publish(topic, messages);
// }

// void publishString(const char* topic, String value) {
//   snprintf(messages, sizeof(messages), "%s", value);
//   client.publish(topic, messages);
// }

void publishWifiMqttStatus()
{
  client.publish("esp32/status", "online", true);

  if (client.connected())
  {
    client.publish(
      "esp32/wifi",
      WiFi.status() == WL_CONNECTED ? "online" : "offline",
      true
    );
  }
}

void publishMessage() 
{
  StaticJsonDocument<512> doc;

  // --- Core readings ---
  doc["mashTemp"]      = roundTo(current_mashTemp, 6);  // 3 decimal
  doc["airTemp"]       = roundTo(current_airTemp, 2);   // 2 decimal
  doc["timePassed"]    = roundTo(passedTimeS / 60.0f, 1);

  // --- Setpoint ---
  doc["targetTemp"]    = settings.targetTemp;

  // --- Power / control ---
  doc["dutyCycle"]     = roundTo(DutyCycle / 4095.0f * 100.0f, 2);
  doc["powerIn"]       = roundTo(DutyCycle / 4095.0f * 2500.0f, 6); // integer watts

  // --- Boolean states (REAL booleans) ---
  doc["pumpOn"]        = settings.pump;
  doc["POnE"]          = settings.POnE_mash;
  doc["hopsAlarm"]     = hopsAlarm;
  doc["waterDetected"] = waterDetected;

  // --- PID parameters ---
  doc["kp"]            = settings.Kp_mash;
  doc["ki"]            = settings.Ki_mash;
  doc["heatLoss"]      = settings.k_heatLoss;

  // --- Filters (rounded for readability) ---
  doc["filterTemp"]    = roundTo(settings.filter_adc1, 2);
  doc["filterDC"]      = roundTo(settings.filterDC, 2);

  // --- Serialize and publish ---
  char buffer[512];
  size_t n = serializeJson(doc, buffer);
  client.publish("mashTun/state", buffer, n);

  if (!writeInflux(current_mashTemp, DutyCycle / 4095.0f * 2500.0f)) {
  Serial.println("Influx write failed");
}

}

bool writeInflux(float mashTemp, float powerW)
{
  if (WiFi.status() != WL_CONNECTED)
    return false;

  HTTPClient http;
  http.begin(privates.influxURL);
  http.addHeader("Authorization", String("Token ") + privates.influxToken);
  http.addHeader("Content-Type", "text/plain");

  String line =
    "mashtun,source=esp32 "
    "mashtemp=" + String(mashTemp, 3) +
    ",power="   + String(powerW, 1);

  int code = http.POST(line);
  Serial.print("Influx HTTP code: ");
  Serial.println(code);
  http.end();

  return (code == 204);
}




// void publishMessage() //Homeassistant
// {
//   // Serial.print("current_mashTemp: ");
//   // Serial.print(current_mashTemp, 1);
//   // Serial.print(", filtered: ");
//   // Serial.println(filtered, 1);

//   publishFloat(privates.topicMashTemp, current_mashTemp);
//   publishFloat(privates.topicAirTemp, current_airTemp);
//   publishFloat(privates.topicTimePassed, passedTimeS/60.0);
//   publishFloat(privates.topicTargetTemp, settings.targetTemp);
//   publishFloat(privates.topicDutyCycle, DutyCycle/4095.0 * 100.0);
//   publishFloat(privates.topicPowerIn,   DutyCycle/4095.0 * 2500.0);
//   publishString(privates.topicPumpOn, settings.pump ? "ON" : "OFF");
//   publishFloat(privates.topicKp, settings.Kp_mash);
//   publishFloat(privates.topicKi, settings.Ki_mash);
//   publishFloat(privates.topicHeatLoss, settings.k_heatLoss);
//   publishString(privates.topicPOnE, settings.POnE_mash ? "ON" : "OFF");
//   publishFloat(privates.topicfilterTemp, settings.filter_adc1);
//   publishFloat(privates.topicfilterDC, settings.filterDC);
//   publishString(privates.topicHopsAlarm, hopsAlarm ? "ON" : "OFF");
//   publishString(privates.topicWaterDetected, waterDetected ? "YES" : "NO");
// }


void callback(char* topic, byte* payload, unsigned int length) //Homeassistant
{
  if ( !TopicArrived )
  {
    memset( mqttpayload, '\0', mqttpayloadSize ); // clear payload char buffer
    mqtttopic = ""; //clear topic string buffer
    mqtttopic = topic; //store new topic
    memcpy( mqttpayload, payload, length );
    TopicArrived = true;
  }
}

double convertRawToCelsius(double vOut) {
  double rNTC = (R_ref * vOut) / (3.3 - vOut); 
  // Beta-formeln
  double invT = (1.0 / T_0) + (1.0 / BETA) * log(rNTC / R_NTC);
  double T = 1.0 / invT;    // Kelvin
  T -= 273.15;             // Celsius
  T = map(T, settings.callibrationCold, settings.callibrationHot, 0.0, 100.0);
  T = constrain(T, 0.0, 100.0);
  // Serial.print(", Temp_C: ");
  // Serial.println(T, 1);
  return T;       // Celsius
}

double Filter(double New, double Current, double alpha) //Moisture sensor
{
  return (1.0 - alpha) * New + alpha * Current;
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float roundTo(float value, int decimals) {
  float multiplier = powf(10.0f, decimals);
  return roundf(value * multiplier) / multiplier;
}
float computeARX(float u0){

  u[0] = u0;
  // Compute ARX output
  y[0] = - a1 * y[1] - a2 * y[2] + b0 * u[0] + b1 * u[1] + b2 * u[2];

  // Shift past values
  y[2] = y[1];
  y[1] = y[0];
  u[2] = u[1];
  u[1] = u[0];

  return y[0];
}

float readADCwithAutoGain(Adafruit_ADS1115 &adc, int channel) {
    // Step 1: rough read with lowest gain
    adc.setGain(GAIN_TWOTHIRDS);
    adc.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0 + channel, false);
    while(!adc.conversionComplete());
    int16_t raw = adc.getLastConversionResults();

    // convert voltage using TWOTHIRDS gain
    float voltage = raw * 0.0001875;  // ±6.144V -> 0.1875 mV/bit

    // Step 2: choose gain based on voltage
    adsGain_t chosenGain;
    float lsb = 0.0001875; // default for TWOTHIRDS

    if (voltage < 0.20) {
        chosenGain = GAIN_SIXTEEN; // ±0.256V
        lsb = 0.0000078125;        // 7.8125 uV/bit
    }
    else if (voltage < 0.40) {
        chosenGain = GAIN_EIGHT;   // ±0.512V
        lsb = 0.000015625;
    }
    else if (voltage < 0.85) {
        chosenGain = GAIN_FOUR;    // ±1.024V
        lsb = 0.00003125;
    }
    else if (voltage < 1.80) {
        chosenGain = GAIN_TWO;     // ±2.048V
        lsb = 0.0000625;
    }
    else if (voltage < 3.5) {
        chosenGain = GAIN_ONE;     // ±4.096V
        lsb = 0.000125;
    }
    else {
        chosenGain = GAIN_TWOTHIRDS; // ±6.144V
        lsb = 0.0001875;
    }

    // Step 3: read again with correct gain
    adc.setGain(chosenGain);
    adc.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0 + channel, false);
    while(!adc.conversionComplete());
    raw = adc.getLastConversionResults();

    // Now raw * lsb gives correct voltage
    voltage = raw * lsb;

    return voltage; // or return voltage if you prefer
}
