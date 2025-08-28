#include <Arduino.h>
#include <Wire.h> //temperatur
#include <Adafruit_GFX.h> //oled
#include <Adafruit_SSD1306.h> //oled
#include <U8g2lib.h> //oled
#include <Adafruit_I2CDevice.h>
#include <OneWire.h> //temperatur
#include <DallasTemperature.h> //temperatur
#include <PID_v1.h> //PID
#include <PressButton.h> //Interface
#include <RotaryEncoderAccel.h>; //Interface
#include <EEPROM.h> //Save Settings
#include <WiFi.h> //Home assistant
#include <PubSubClient.h> //Home assistant
#include "Privates.h" //Homeassistant 
#include <math.h>
//-----------------------------------------------------------------------
// OLED display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
//-----------------------------------------------------------------------
//IO-pins
#define oneWireBus1pin 4 //pin 4 ESP32 - temperatur
#define oneWireBus2pin 32 //pin 32 ESP32 - temperatur thermostat
#define oneWireBus3pin 17 //pin 17 ESP32 - temperatur luft
#define dutyCycleOutPin 26 //pin 26 ESP32 - Grön LED
#define confirmBtnPin 25 // pin 25 ESP32 - ok
#define ledOnPin 14 //pin 14 ESP32 - röd LED
#define alarmPin 33 //pin 33 ESP32 - alarm
//rotary encoder
#define outputA 13 //pin 13 ESP32 - 
#define outputB 16 //pin 16 ESP32 - 
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
//-----------------------------------------------------------------------
//Varibles

//Homeassistant
//-----------------------------------------------------------------------
Privates privates; //Homeassistant
WiFiClient espClient; //Home assistant
PubSubClient client(espClient); //Home assistant
char messages[50]; //Home assistant
volatile  bool TopicArrived = false;
const     int mqttpayloadSize = 10;
char mqttpayload [mqttpayloadSize] = {'\0'};
String mqtttopic;
//-----------------------------------------------------------------------
//rotary encoder
RotaryEncoderAccel encoder(outputA,outputB);
// ISR för båda pins, som anropas vid ändring (rising/falling)
void IRAM_ATTR handleEncoderInterrupt() {
  encoder.tick();
}
//-----------------------------------------------------------------------
//Buttons
PressButton btnOk(confirmBtnPin);
void IRAM_ATTR handleButtonInterrupt() {
  btnOk.CaptureDownState();
}
//-----------------------------------------------------------------------
//Menu structure
enum pageType{
  MENU_ROOT,
  MENU_TARGET_TEMP,
  MENU_TEMPERATURES,
  MENU_MISC,
  MENU_PID,
  MENU_TIME
};

enum pageType currPage = MENU_ROOT;
void page_MenuRoot();
void page_MENU_TARGET_TEMP();
void page_MENU_MISC();
void page_MENU_PID();
void page_MENU_TIME();
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
bool initPage = true;
bool changeValue = false;
bool changeValues [10];

void initMenuPage(String title, uint8_t itemCount);
void captureButtonDownStates();
void incrementDecrementDouble(double *v, double amount, double min, double max);
void doPointerNavigation();
bool isFlashChanged();
void pacingWait();
bool menuItemPrintable(uint8_t xPos, uint8_t yPos);
//-----------------------------------------------------------------------
//Print tools
void printPointer();
void printOnOff(bool val);
void printUint32_tAtWidth(uint32_t value, uint8_t width, char c);
void printDoubleAtWidth(double value, uint8_t width, char c);
//-----------------------------------------------------------------------
//Settings
#pragma pack(1) //memory alignment
struct Mysettings{
  double Kp_mash = 125.0;
  double Ki_mash = 0.0;
  double Kp_element = 125.0/32;
  double Ki_element = 0.0;
  double Kp_load = 1.0;

  double targetTemp = 25;

  boolean manualMode = 1;
  double temp1 = 65;
  double time1 = 45;
  double temp2 = 70;
  double time2 = 30;
  double temp3 = 77;
  double time3 = 15;

  boolean power = true;
  double RawLow = 0.31;
  double RawHigh = 99.56;
  double maxElementTemp = 126; //126
  double marginalTemp = 10;
  double dutycycleThreshold = 50;

  uint16_t settingsCheckValue = SETTINGS_CHKVAL;
};

Mysettings settings;
Mysettings oldSettings;
void sets_SetDefaults();
void sets_Load();
void sets_Save();
//-----------------------------------------------------------------------
//Time
unsigned long previousTime = 0; 
unsigned long currentTime;
const unsigned long saveInterval = 60000; // 60 000 ms = 60 sekunder
unsigned long lastEditTime = 0;
double tS;
long passedTimeS;
long previousPassedTimeS;
double passedTime,previousPassedTime1,previousPassedTime2 = 0;
//-----------------------------------------------------------------------
//Temperatur
OneWire oneWire1(oneWireBus1pin);
DallasTemperature sensor1(&oneWire1);

OneWire oneWire2(oneWireBus2pin);
DallasTemperature sensor2(&oneWire2);

OneWire oneWire3(oneWireBus3pin);
DallasTemperature sensor3(&oneWire3);

double current_mashTemp;
double previous_mashTemp;
double current_thermostatTemp;
double previous_thermostatTemp;
double current_airTemp;
double previous_airTemp;
//-----------------------------------------------------------------------
double loadCompensation;

double RawRange;
double ReferenceHigh = 100.0;
double ReferenceLow = 0.0;
double ReferenceRange = ReferenceHigh - ReferenceLow;
//-----------------------------------------------------------------------
//PID
double error;
double previousError;
double dt, last_time;
double integral, previous, pid = 0;
double Output_mashTemp;
double Output_elementTemp;
double tempC;
double DutyCycle = 0;
double previousDutyCycle = 0;
double targetElementTemp = settings.maxElementTemp - settings.marginalTemp;
PID PID_mashTemp(&current_mashTemp, &Output_mashTemp, &settings.targetTemp, settings.Kp_mash, settings.Ki_mash, 0, DIRECT);
PID PID_elementTemp(&current_thermostatTemp, &Output_elementTemp, &targetElementTemp, settings.Kp_element, settings.Ki_element, 0, DIRECT);
//-----------------------------------------------------------------------
// setting PWM properties
int freq = 5; //5
const int ledChannel = 0;
const int resolution = 12;
 
// Declaration for an SSD1306 display1 connected to I2C (SDA, SCL pins)
//Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
//-----------------------------------------------------------------------
// Oled - Adafruit_SSD1306
U8G2_SH1106_128X64_NONAME_F_HW_I2C display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C display2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
long timeLastTouched = 0;
const long timeBeforeDisable = 120000;
//-----------------------------------------------------------------------


void setup() {
//=================================================SETUP=======================================================
  Serial.begin(115200);

  sensor1.begin();
  sensor2.begin();
  sensor3.begin();
  sensor1.setResolution(9);
  sensor2.setResolution(9);
  sensor3.setResolution(9);

  currentTime = millis();

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(dutyCycleOutPin, ledChannel);

  pinMode(oneWireBus1pin, INPUT_PULLUP);
  pinMode(oneWireBus2pin, INPUT_PULLUP);
  pinMode(oneWireBus3pin, INPUT_PULLUP);
  pinMode(ledOnPin, OUTPUT);
  pinMode(alarmPin, OUTPUT);
  pinMode(outputA,INPUT_PULLUP);
  pinMode(outputB,INPUT_PULLUP);

    // Koppla interrupt på båda encoder-pins till samma ISR
  attachInterrupt(digitalPinToInterrupt(outputA), handleEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), handleEncoderInterrupt, CHANGE);

  attachInterrupt(digitalPinToInterrupt(confirmBtnPin), handleButtonInterrupt, CHANGE);

  Wire.begin(21, 22);           // pins only
  
  display1.setI2CAddress(0x3C << 1); 
  display1.begin();  
  display1.setBusClock(3400000);    // ask U8g2 to set speed
  display1.setFont(u8g2_font_6x10_mf);	// choose a suitable font
  display1.clearBuffer();					// clear the internal memory
  display1.setCursor(0, 0);

  display2.setI2CAddress(0x3D << 1);
  display2.begin();  
  display2.setBusClock(3400000);    // ask U8g2 to set speed
  display2.setFont(u8g2_font_6x10_mf);	// choose a suitable font  
  display2.clearBuffer();					// clear the internal memory
  display2.setCursor(0, 0);

  EEPROM.begin(sizeof(settings));
  sets_Load();
  
  PID_mashTemp.SetTunings(settings.Kp_mash, settings.Ki_mash, 0);
  PID_mashTemp.SetOutputLimits(0,4095.0 * settings.dutycycleThreshold * 0.01);
  PID_mashTemp.SetMode(AUTOMATIC);  
  PID_mashTemp.Compute();

  PID_elementTemp.SetTunings(settings.Kp_element, settings.Ki_element, 0);
  PID_elementTemp.SetOutputLimits(0,settings.maxElementTemp - settings.marginalTemp);
  PID_elementTemp.SetMode(AUTOMATIC);
  PID_elementTemp.Compute();

  setupWiFi(); //Home assistant
  setupMQTT(); //Home assistant

}

void loop() { //=================================================LOOP=======================================================

  passedTime = millis()/1000.0;
  
  if(millis() - timeLastTouched > timeBeforeDisable)
  {
    display1.setPowerSave(1);
    display2.setPowerSave(1);
  }
  else
  {
    display1.setPowerSave(0);
    display2.setPowerSave(0);
  }

  RawRange = settings.RawHigh - settings.RawLow;

  const float UPDATE_INTERVAL1 = 0.25;
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
    case MENU_TIME: page_MENU_TIME(); break;
  }

  printPointer();              
  if ((updateAllItems || updateItemValue) && shouldUpdate2)
  {
    display1.sendBuffer();                        //välldigt långsam
    previousPassedTime2 = passedTime;
    updateAllItems = false;
    updateItemValue = false;
  } 

  //captureButtonDownStates();

  //Homeassistant
  if(passedTime - previousTime > 1.0) {publishMessage(); previousTime = passedTime;}

  if(edditing) // If edditing is true, save the current cursor position and display offset
  {   
    // Save current cursor position and display offset
    // and set the encoder position to the current cursor position
    saveCursorPos = encoder.getPosition();
    saveDispOffset = dispOffset;

    encoder.setAccel(150, 5); //encoder.setAccel(150, 5);
    // Serial.println(",\tEditing: ");
  }
  else // If edditing is false, restore the cursor position and display offset
  {
    // encoder.setPosition(saveCursorPos);
    // dispOffset = saveDispOffset; 

    //encoder.setAccel(0, 1);
  }

      // === WiFi reconnect ===
  if (WiFi.status() != WL_CONNECTED) {
    reconnectWiFi();
  }

  // === MQTT reconnect ===
  if (WiFi.status() == WL_CONNECTED && !client.connected()) {
    reconnectMQTT();
  }

  // === MQTT keep-alive ===
  if (client.connected()) {
    client.loop();   // måste köras så ofta som möjligt
  }

  //digitalWrite(ledWifiPin, WiFi.status() == WL_CONNECTED ? LOW : HIGH);
  //digitalWrite(ledMqttPin, client.connected() ? LOW : HIGH);
}

void page_MenuRoot()
{
  //=================================================ROOT_MENU============================================
  if(initPage)
  {
    cursorPos = root_pntrPos;
    dispOffset = root_dispOffset;
    initMenuPage(F("MAIN MENU"), 4);
    initPage = false;
  }

  doPointerNavigation();

  if(menuItemPrintable(1,1)){display1.print(F("TARGET TEMP  "));} //kan inte ändra
  if(menuItemPrintable(1,2)){display1.print(F("MISC         "));} //kan ändra
  if(menuItemPrintable(1,3)){display1.print(F("PID          "));} //settings, kan ändra
  if(menuItemPrintable(1,4)){display1.print(F("MashProgram  "));} //settings, kan ändra  

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
      case 3: currPage = MENU_TIME; return;
    }
  }
}

void page_MENU_TARGET_TEMP(){//=================================================TARGET_TEMP============================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("TARGET_TEMP"), 2);
    initPage = false;
    changeValue = false;
  }

  if(menuItemPrintable(1,1)){display1.print(F("Target Temp =     "));}
  if(menuItemPrintable(1,2)){display1.print(F("Back              "));}
  
  if(menuItemPrintable(13,1)){printUint32_tAtWidth(settings.targetTemp, 3, 'C');}

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
    incrementDecrementDouble(&settings.targetTemp, 1.0, 15.0, 100.0);
    settings.targetTemp = round(settings.targetTemp); // ensure integer steps
  }
    
  else 
    doPointerNavigation(); 
}

void page_MENU_MISC(){//=================================================MISC==========================================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("MISC"), 7);
    double passedTime,previousPassedTime = 0;
    initPage = false;
    changeValues [10];
  }

  if(menuItemPrintable(1,1)){display1.print(F("POWER        =     "));}
  if(menuItemPrintable(1,2)){display1.print(F("RawLow       =     "));}
  if(menuItemPrintable(1,3)){display1.print(F("RawHigh      =     "));}
  if(menuItemPrintable(1,4)){display1.print(F("MaxEleTemp   =     "));}
  if(menuItemPrintable(1,5)){display1.print(F("MarginalTemp =     "));}
  if(menuItemPrintable(1,6)){display1.print(F("DC TH        =     "));}
  if(menuItemPrintable(1,7)){display1.print(F("Back               "));}
  
  if(menuItemPrintable(12,1)){printOnOff(settings.power);}
  if(menuItemPrintable(12,2)){printDoubleAtWidth(settings.RawLow, 4, ' ');}
  if(menuItemPrintable(12,3)){printDoubleAtWidth(settings.RawHigh, 4, ' ');}
  if(menuItemPrintable(12,4)){printUint32_tAtWidth(settings.maxElementTemp, 4, 'C');}
  if(menuItemPrintable(12,5)){printUint32_tAtWidth(settings.marginalTemp, 4, 'C');}
  if(menuItemPrintable(12,6)){printUint32_tAtWidth(settings.dutycycleThreshold, 4, '%');}

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
      case 6: currPage = MENU_ROOT; sets_Save(); initPage = true; return;
    }
  }

  if(changeValues[0])
  {
    *&settings.power = !*&settings.power;
    changeValues [0] = false;
    updateItemValue = true; 
  }
  else if(changeValues[1])incrementDecrementDouble(&settings.RawLow, 0.05, 0.0, 5.0);
  else if(changeValues[2])incrementDecrementDouble(&settings.RawHigh, 0.05, 95.0, 105.0);
  else if(changeValues[3]){incrementDecrementDouble(&settings.maxElementTemp, 1, 15, 130); settings.maxElementTemp = round(settings.maxElementTemp);}
  else if(changeValues[4]){incrementDecrementDouble(&settings.marginalTemp, 1, 0, 20); settings.marginalTemp = round(settings.marginalTemp);}
  else 
    doPointerNavigation(); 

  PID_elementTemp.SetTunings(settings.Kp_element, settings.Ki_element, 0);
  PID_elementTemp.SetOutputLimits(0,settings.maxElementTemp - settings.marginalTemp);
  PID_elementTemp.SetMode(AUTOMATIC);
  PID_elementTemp.Compute();
}

void page_MENU_PID(){//=================================================PID===============================================================
  if(initPage)
  { 
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("PID"), 6);
    initPage = false;
    changeValues [10];
  }

  if(menuItemPrintable(1,1)){display1.print(F("Kp Mash  =           "));}
  if(menuItemPrintable(1,2)){display1.print(F("Ki Mash  =           "));}
  if(menuItemPrintable(1,3)){display1.print(F("Kp Ele   =           "));}
  if(menuItemPrintable(1,4)){display1.print(F("Ki Ele   =           "));}
  if(menuItemPrintable(1,5)){display1.print(F("Kp Load  =           "));}
  if(menuItemPrintable(1,6)){display1.print(F("Back                 "));}

  if(menuItemPrintable(12,1)){printUint32_tAtWidth(settings.Kp_mash, 4, ' ');}
  if(menuItemPrintable(12,2)){printDoubleAtWidth(settings.Ki_mash, 4, ' ');}
  if(menuItemPrintable(12,3)){printDoubleAtWidth(settings.Kp_element, 4, ' ');}
  if(menuItemPrintable(12,4)){printDoubleAtWidth(settings.Ki_element, 4, ' ');}
  if(menuItemPrintable(12,5)){printDoubleAtWidth(settings.Kp_load, 4, ' ');}

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
      case 5: currPage = MENU_ROOT; sets_Save(); initPage = true; return;
    }
  }
        if(changeValues[0]){incrementDecrementDouble(&settings.Kp_mash, 1, 0.0, 200.0); settings.Kp_mash = round(settings.Kp_mash);}
  else if(changeValues[1])incrementDecrementDouble(&settings.Ki_mash, 0.1, 0.0, 100.0);
  else if(changeValues[2])incrementDecrementDouble(&settings.Kp_element, 0.1, 0.0, 50.0);
  else if(changeValues[3])incrementDecrementDouble(&settings.Ki_element, 0.05, 0.0, 25.0);
  else if(changeValues[4])incrementDecrementDouble(&settings.Kp_load, 0.1, 1.0, 5.0);
  else 
    doPointerNavigation(); 

  PID_mashTemp.SetTunings(settings.Kp_mash, settings.Ki_mash, 0);
  PID_mashTemp.SetOutputLimits(0,4095.0 * settings.dutycycleThreshold * 0.01);
  PID_mashTemp.SetMode(AUTOMATIC);
  PID_mashTemp.Compute();

  PID_elementTemp.SetTunings(settings.Kp_element, settings.Ki_element, 0);
  PID_elementTemp.SetOutputLimits(0,settings.maxElementTemp - settings.marginalTemp);
  PID_elementTemp.SetMode(AUTOMATIC);
  PID_elementTemp.Compute();
  
}

void page_MENU_TIME(){//=================================================MASH_PROGRAM====================================================
  if(initPage)
  { 
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("MashProgram"), 9);
    initPage = false;
    changeValues [10];
  }
 
  if(menuItemPrintable(1,1)){display1.print(F("Manual =            "));}
  if(menuItemPrintable(1,2)){display1.print(F("Mash:               "));}
  if(menuItemPrintable(1,3)){display1.print(F("Temp 1 =            "));}
  if(menuItemPrintable(1,4)){display1.print(F("Time 1 =            "));}
  if(menuItemPrintable(1,5)){display1.print(F("Temp 2 =            "));}
  if(menuItemPrintable(1,6)){display1.print(F("Time 2 =            "));}
  if(menuItemPrintable(1,7)){display1.print(F("Temp 3 =            "));}
  if(menuItemPrintable(1,8)){display1.print(F("Time 3 =            "));}
  if(menuItemPrintable(1,9)){display1.print(F("Back                "));}
  
  if(menuItemPrintable(11,1)){printOnOff(settings.manualMode);}
  if(menuItemPrintable(7,2)){printDoubleAtWidth(passedTimeS/60.0, 3, 'm');}
  if(menuItemPrintable(13,2)){printUint32_tAtWidth(settings.targetTemp, 10, 'C');}
  if(menuItemPrintable(11,3)){printUint32_tAtWidth(settings.temp1, 10, 'C');}
  if(menuItemPrintable(11,4)){printUint32_tAtWidth(settings.time1, 10, 'm');}
  if(menuItemPrintable(11,5)){printUint32_tAtWidth(settings.temp2, 10, 'C');}
  if(menuItemPrintable(11,6)){printUint32_tAtWidth(settings.time2, 10, 'm');}
  if(menuItemPrintable(11,7)){printUint32_tAtWidth(settings.temp3, 10, 'C');}
  if(menuItemPrintable(11,8)){printUint32_tAtWidth(settings.time3, 10, 'm');}

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
    *&settings.manualMode = !*&settings.manualMode;
    changeValues [0] = false;
    updateItemValue = true; 
  } 
  else if(changeValues[1]){incrementDecrementDouble(&settings.temp1, 1.0, 15.0, 100.0); settings.temp1 = round(settings.temp1);}
  else if(changeValues[2]){incrementDecrementDouble(&settings.time1, 1.0, 0.0, 90.0); settings.time1 = round(settings.time1);}
  else if(changeValues[3]){incrementDecrementDouble(&settings.temp2, 1.0, 15.0, 100.0); settings.temp2 = round(settings.temp2);}
  else if(changeValues[4]){incrementDecrementDouble(&settings.time2, 1.0, 0.0, 90.0); settings.time2 = round(settings.time2);}
  else if(changeValues[5]){incrementDecrementDouble(&settings.temp3, 1.0, 15.0, 100.0); settings.temp3 = round(settings.temp3);}
  else if(changeValues[6]){incrementDecrementDouble(&settings.time3, 1.0, 0.0, 90.0); settings.time3 = round(settings.time3);}
  else 
    doPointerNavigation();
  
}

//======================================================TOOLS - menu Internals==================================================
void initMenuPage(String title, uint8_t itemCount){
  //display1.clearDisplay();
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

  if(!edditing)
  {
    direction = constrain(direction, -(itemCnt - 1), itemCnt - 1);
  }

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
      timeLastTouched = millis(); // Update last touched time
    }
    
    // Serial.print("Direction: ");
    // Serial.print(direction);
    // Serial.print(",\t\tNew Cursor Position: ");
    // Serial.print(newCursorPos);
    // Serial.print(",\t\tDisplay Offset: ");
    // Serial.println(dispOffset);
  }
  delayMicroseconds(5); //behövs för stabiliteten av doPointerNavigation
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
        // Serial.print(", newValue: ");
        // Serial.print(*v);
        updateItemValue = true;
        timeLastTouched = millis();
    }
    delayMicroseconds(5); //behövs för stabiliteten av incrementDecrementDouble
}

void incrementDecrementInt(uint16_t *v, uint16_t amount, uint16_t min, uint16_t max)
{
    int direction = encoder.getRPM();

    if (direction != 0) {
        int target = direction * amount;
        int newValue = *v + target;

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
        timeLastTouched = millis();
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
  display1.drawStr(0, 1*CHAR_Y, " ");
  display1.drawStr(0, 2*CHAR_Y, " ");
  display1.drawStr(0, 3*CHAR_Y, " ");
  display1.drawStr(0, 4*CHAR_Y, " ");
  display1.sendBuffer();

  delay(50);
  //Serial.println("FlashPointer");
  display1.drawStr(0, (cursorPos - dispOffset + 1)*CHAR_Y, "*");
  display1.sendBuffer();
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
uint8_t getUint32_tCharCnt(uint32_t value)
{
  if(value == 0){return 1;}
  uint32_t tensCalc = 10; int8_t cnt = 1;
  while (tensCalc <= value && cnt < 20){tensCalc *= 10; cnt += 1;}
  return cnt;
}
uint8_t getDoubleCharCnt(double value)
{
  if(value == 0){return 1;}
  uint32_t tensCalc = 10; int8_t cnt = 1;
  while (tensCalc <= value && cnt < 20){tensCalc *= 10; cnt += 1;}
  return cnt;
}
void printUint32_tAtWidth(uint32_t value, uint8_t width, char c){
  display1.print(value);
  display1.print(c);
  printChars(width-getUint32_tCharCnt(value), ' ');
}
void printDoubleAtWidth(double value, uint8_t width, char c){
  char buf[10];
  dtostrf(value, width-getDoubleCharCnt(value), 1, buf); // 1 decimal
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
void printUint32_tAtWidthDisplay2(uint32_t value, uint8_t width, char c){
  display2.print(value);
  display2.print(c);
  printCharsDisplay2(width-getUint32_tCharCnt(value), ' ');
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
  if (current_thermostatTemp > settings.maxElementTemp)
  {
    updateAllItems = true;
    updateItemValue = true;
    settings.power = false;
    analogWrite(alarmPin, 100);
  }
  else analogWrite(alarmPin, LOW);

  if(settings.power)
  {
    digitalWrite(ledOnPin, HIGH);
    if (settings.manualMode == false)
    {
      previousPassedTimeS = passedTimeS;
      passedTimeS = (millis() - tS) / (1000);

      if (0 <= passedTimeS && passedTimeS < settings.time1 * 60) //60
        settings.targetTemp = settings.temp1;
      else if (settings.time1 * 60 <= passedTimeS && passedTimeS < (settings.time1 + settings.time2) * 60) //60
        settings.targetTemp = settings.temp2;     
      else if ((settings.time1 + settings.time2) * 60 <= passedTimeS && passedTimeS < (settings.time1 + settings.time2 + settings.time3) * 60) //60     
        settings.targetTemp = settings.temp3;     
      else
        settings.targetTemp = 15;
      
    }
    else
    {
      tS = millis();
      passedTimeS = 0;
    }
  
    // Serial.print("PID_mashTemp.Compute(): ");
    // Serial.println(PID_mashTemp.Compute());
    // Serial.println(settings.Kp_mash);
    // Serial.println(PID_mashTemp.GetKp());

    updateAllItems = PID_mashTemp.Compute() || PID_elementTemp.Compute();
    updateAllItems = true;                                                        //nytt
    previousDutyCycle = DutyCycle;
    DutyCycle = Output_mashTemp;  
    DutyCycle *= (1 + (loadCompensation / 60.0 ) * (settings.Kp_load - 1)); //(settings.Kp_load + loadCompensation)/settings.Kp_load;
    DutyCycle = constrain(DutyCycle, 0, 4096.0 * settings.dutycycleThreshold * 0.01); 
    ledcWrite(ledChannel, DutyCycle); 
    
    settings.dutycycleThreshold = Output_elementTemp; 
    //Serial.println(settings.dutycycleThreshold);
    settings.dutycycleThreshold = constrain(settings.dutycycleThreshold, 0, 100); //100

    targetElementTemp = settings.maxElementTemp - settings.marginalTemp;

    client.loop();
    // if(TopicArrived)
    // {
    //   Serial.print("Message arrived: ");
    //   Serial.println(mqttpayload);
    //   //settings.targetTemp = atof(mqttpayload);
    //   TopicArrived = false;
    // }
  }
  else
  {
    digitalWrite(ledOnPin, LOW);
    ledcWrite(ledChannel, 0);
  }
  
}
void updateSensorValues()                        // långsamast i hela programet
{
    sensor1.requestTemperatures();
    sensor2.requestTemperatures();
    sensor3.requestTemperatures();

    previous_mashTemp = current_mashTemp;
    previous_thermostatTemp = current_thermostatTemp;

    float RawValue1 = sensor1.getTempCByIndex(0);  // långsamast i hela programet
    float RawValue2 = sensor2.getTempCByIndex(0);  // långsamast i hela programet
    float RawValue3 = 22; //sensor3.getTempCByIndex(0);

    current_mashTemp = constrain((((RawValue1 - settings.RawLow) * ReferenceRange) / RawRange) + ReferenceLow, 0, 110);
    current_thermostatTemp = constrain((((RawValue2 - settings.RawLow) * ReferenceRange) / RawRange) + ReferenceLow, 0, 150);
    current_airTemp = constrain((((RawValue3 - settings.RawLow) * ReferenceRange) / RawRange) + ReferenceLow, 0, 40);
    loadCompensation = constrain(current_mashTemp - current_airTemp, 0, 60.0);

/*  
    Serial.print("current_airTemp: ");
    Serial.print(current_airTemp);
    Serial.print(", current_mashTemp: ");
    Serial.print(current_mashTemp);
    Serial.print(", current_mashTemp - current_airTemp: ");
    Serial.println(loadCompensation);
    */
 
}
void updateDisp2()
{
  display2.clearBuffer();
  if(menuItemPrintableDisp2(1,1)){display2.print(F("TargetTemp =      "));}
  if(menuItemPrintableDisp2(1,2)){display2.print(F("MashTemp   =      "));}
  if(menuItemPrintableDisp2(1,3)){display2.print(F("DC         =      "));}
  if(menuItemPrintableDisp2(1,4)){display2.print(F("EleTemp    =      "));}

  if(menuItemPrintableDisp2(11,1)){printDoubleAtWidthDisplay2(settings.targetTemp, 3, 'C');}  
  if(menuItemPrintableDisp2(11,2)){printDoubleAtWidthDisplay2(current_mashTemp, 3, 'C');}
  if(menuItemPrintableDisp2(11,3)){printDoubleAtWidthDisplay2(DutyCycle/4096.0 * 100.0, 3, '%');}
  if(menuItemPrintableDisp2(11,4)){printDoubleAtWidthDisplay2(current_thermostatTemp, 3, 'C');}
  display2.sendBuffer();
  display2.clearBuffer(); 
}

double Pid(double error, float kp, float ki, float kd)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}

void setupWiFi() //Homeassistant
{
  WiFi.begin(privates.ssid, privates.pass);
  Serial.print("\nConnecting to ");
  Serial.print(privates.ssid);

  while(WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }
  Serial.print("\nConnected to ");
  Serial.println(privates.ssid);
}
void setupMQTT() //Homeassistant
{
  client.setServer(privates.broker, 1883);
  client.setCallback(callback);
  client.connect("MashMachine", privates.brokerUser, privates.brokerPass);
  while (!client.connected())
  {
    delay(100);
    Serial.print(".");
  }
  Serial.println("MQTT setup complete");
}
void reconnectWiFi()  //Homeassistant
{
  static unsigned long lastAttempt = 0;
  if (millis() - lastAttempt > 5000) { // försök var 5:e sekund
    lastAttempt = millis();
    WiFi.begin(privates.ssid, privates.pass);
    Serial.println("Trying to reconnect WiFi...");
  }
}
void reconnectMQTT()  //Homeassistant
{
  static unsigned long lastAttempt = 0;
  if (millis() - lastAttempt > 3000) { // försök var 3:e sekund
    lastAttempt = millis();
    Serial.println("Trying to reconnect MQTT...");
    if (client.connect("MashMachine", privates.brokerUser, privates.brokerPass)) {
      Serial.println("MQTT connected!");
      // subscribe till topics här
    }
  }
}
void publishMessage() //Homeassistant
{
  publishInt(privates.topicMashTemp, current_mashTemp);
  publishInt(privates.topicElementTemp, current_thermostatTemp);
  publishFloat(privates.topicTimePassed, passedTimeS/60.0);
  publishInt(privates.topicTargetTemp, settings.targetTemp);
  publishFloat(privates.topicDutyCycle, DutyCycle/4096.0 * 100.0);
}
void publishFloat(const char* topic, float value) {
  //int "%d", float "%f", bool "%s", obs vikitgt!
  snprintf(messages, sizeof(messages), "%.1f", value); // 2 decimal places
  client.publish(topic, messages);
}
void publishInt(const char* topic, int value) {
  //int "%d", float "%f", bool "%s", obs vikitgt!
  snprintf(messages, sizeof(messages), "%d", value);
  client.publish(topic, messages);
}
void publishString(const char* topic, String value) {
  snprintf(messages, sizeof(messages), "%s", value);
  client.publish(topic, messages);
}
void subscribeMessage() //Homeassistant
{
    // if(!client.connected()){reconnectMQTT();}
    // client.loop();
    // client.subscribe(privates.topicTargetTemp); 
}

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