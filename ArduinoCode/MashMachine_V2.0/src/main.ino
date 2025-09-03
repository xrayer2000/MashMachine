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
#include <math.h>
#include <MAX6675.h>
//-----------------------------------------------------------------------
// OLED display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
//-----------------------------------------------------------------------
//IO-pins
#define dutyCycleOutPin 26 //pin 26 ESP32 - Grön LED
#define confirmBtnPin 25 // pin 25 ESP32 - ok
#define ledOnPin 14 //pin 14 ESP32 - röd LED
#define VoutPin 35
//rotary encoder
#define outputA 13 //pin 13 ESP32
#define outputB 16 //pin 16 ESP32
// MAX6675 Thermocouple
#define MAX6675_CS 5 //pin 4 ESP32 - MAX6675 chip select
#define MAX6675_SO 19 //pin 2 ESP32 - MAX6675 serial out
#define MAX6675_SCK 18 //pin 15 ESP32 - MAX6675 serial clock
//NTC Thermistor
#define NTC_PIN 34 //pin 34 ESP32 - NTC thermistor
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
void printUint32_tAtWidth(uint32_t value, uint8_t width, char c);
void printDoubleAtWidth(double value, uint8_t width, char c);
//-----------------------------------------------------------------------
//Settings
#pragma pack(1) //memory alignment
struct Mysettings{
  double Kp_element = 125.0;
  double Ki_element = 0.0;
  double targetTemp = 25;

  boolean manualMode = 1;
  u16_t temp1 = 66;
  u16_t temp2 = 76;
  u16_t temp3 = 78;

  u16_t time1 = 60;
  u16_t time2 = 15;
  u16_t time3 = 15;

  u16_t temps[3] = {66,76,78};
  u16_t times[3] = {60,15,15};

  boolean power = true;
  u16_t RawLow = 0;
  u16_t RawHigh = 4000;
  u16_t maxElementTemp = 100; //220

  float filterWeight = 0.2;

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
double DutyCycle = 0;
double previousDutyCycle = 0;
PID PID_elementTemp(&current_mashTemp, &Output_mashTemp, &settings.targetTemp, settings.Kp_element, settings.Ki_element, 0, DIRECT);
//-----------------------------------------------------------------------
// setting PWM properties
int freq = 1000; //5
const int ledChannel = 0;
const int resolution = 12;
//-----------------------------------------------------------------------
// Oled - Adafruit_SSD1306
U8G2_SH1106_128X64_NONAME_F_HW_I2C display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C display2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
long timeLastTouched = 0;
const long timeBeforeDisable = 120000;
//-----------------------------------------------------------------------
//NTC Thermistor
const double Vref = 3.27; //power supply voltage (3.3 V rail) -STM32 ADC pin is NOT 5 V tolerant
double Vout; //Voltage divider output
double R_NTC = 101800.0; //NTC thermistor, Resistance at 25 °C
const double R_ref = 101700.0; //10k resistor measured resistance in Ohms (other element in the voltage divider)
const double BETA = 3950.0; //B-coefficient of the thermistor
const double T_0 = 298.15; //25°C in Kelvin
double Temp_C; //Temperature measured by the thermistor (Celsius)
double filteredValue = 0.0f;
int raw = 0;
float filtered = 0.0f;
//-----------------------------------------------------------------------
double passedTime,previousPassedTime1,previousPassedTime2 = 0;
bool initPage = true;
bool changeValue = false;
bool changeValues [10];
//-----------------------------------------------------------------------
//MAX6675 Thermocouple
MAX6675 thermoCouple(MAX6675_CS, MAX6675_SO, MAX6675_SCK);
float temp = 0;
unsigned long lastReadTime = 0;
const unsigned long readInterval = 250;  // ms
//-----------------------------------------------------------------------

void setup() {//=================================================SETUP=======================================================
  Serial.begin(115200);

  // WiFi.disconnect(true);
  // WiFi.mode(WIFI_OFF);

  analogSetAttenuation(ADC_11db); // Match input range to expected voltage
  analogSetWidth(12);             // Ensure full resolution

  currentTime = millis();

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(dutyCycleOutPin, ledChannel);

  pinMode(ledOnPin, OUTPUT);
  pinMode(confirmBtnPin, INPUT_PULLDOWN);
  pinMode(outputA,INPUT_PULLUP);
  pinMode(outputB,INPUT_PULLUP);
  pinMode(NTC_PIN, INPUT);

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

  PID_elementTemp.SetTunings(settings.Kp_element, settings.Ki_element, 0);
  PID_elementTemp.SetMode(AUTOMATIC);
  PID_elementTemp.Compute();
  //Thermocouple
  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);
  thermoCouple.setOffset(0); 
  if (thermoCouple.read() == STATUS_OK) {
    temp = thermoCouple.getCelsius();
  } 

  setupWiFi(); //Home assistant
  setupMQTT(); //Home assistant
}

void loop() { //=================================================LOOP=======================================================

  passedTime = millis() / 1000.0;

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
  if(passedTime - previousTime > 1.0) {publishMessage(); previousTime = passedTime;}

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

void page_MenuRoot(){//=================================================ROOT_MENU============================================
  if(initPage)
  {
    cursorPos = root_pntrPos;
    dispOffset = root_dispOffset;
    
    initMenuPage(F("MAIN MENU"), 4);
    initPage = false;
  }
  doPointerNavigation();
  
  if(menuItemPrintable(1,1)){display1.print(F("TARGET TEMP        "));}
  if(menuItemPrintable(1,2)){display1.print(F("MISC               "));} 
  if(menuItemPrintable(1,3)){display1.print(F("PID                "));} 
  if(menuItemPrintable(1,4)){display1.print(F("MASH PROGRAM       "));} 

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
    changeValue = false;
    initPage = false;
  }

  if(menuItemPrintable(1,1)){display1.print(F("Target Temp =      "));}
  if(menuItemPrintable(1,2)){display1.print(F("Back               "));}

  if(menuItemPrintable(12,1)){printUint32_tAtWidth(settings.targetTemp, 3, 'C');}

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
    incrementDecrementDouble(&settings.targetTemp, 1.0, 15.0, settings.maxElementTemp);
  else 
    doPointerNavigation(); 

  
}
void page_MENU_MISC(){//=================================================MISC==========================================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("MISC"), 7);
    changeValues [10];
    initPage = false;
  }
   
  if(menuItemPrintable(1,1)){display1.print(F("POWER        =     "));}
  if(menuItemPrintable(1,2)){display1.print(F("RAW          =     "));}
  if(menuItemPrintable(1,3)){display1.print(F("HotCalli     =     "));}
  if(menuItemPrintable(1,4)){display1.print(F("ColdCalli    =     "));}
  if(menuItemPrintable(1,5)){display1.print(F("FilterWeight =     "));}
  if(menuItemPrintable(1,6)){display1.print(F("max_Ele_temp =     "));}
  if(menuItemPrintable(1,7)){display1.print(F("Back               "));}

  if(menuItemPrintable(12,1)){printOnOff(settings.power);}
  if(menuItemPrintable(12,2)){printDoubleAtWidth(filtered, 3, ' ');}
  if(menuItemPrintable(12,3)){printDoubleAtWidth(settings.RawLow, 3, ' ');}
  if(menuItemPrintable(12,4)){printDoubleAtWidth(settings.RawHigh, 3, ' ');}
  if(menuItemPrintable(12,5)){printDoubleAtWidth(settings.filterWeight, 3, ' ');}
  if(menuItemPrintable(12,6)){printUint32_tAtWidth(settings.maxElementTemp, 4, 'C');}
      
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
      case 6: currPage = MENU_ROOT; sets_Save(); initPage = true; return;
    }
  }

  if(changeValues[0])
  {
    *&settings.power = !*&settings.power;
    changeValues [0] = false;
    updateItemValue = true; 
  }
  else if(changeValues[1])incrementDecrementInt(&settings.RawLow, 1, 0, 400);
  else if(changeValues[2])incrementDecrementInt(&settings.RawHigh, 1, 3800, 4096);
  else if(changeValues[3])incrementDecrementFloat(&settings.filterWeight, 0.1f, 0.0f, 0.8f);
  else if(changeValues[4])incrementDecrementInt(&settings.maxElementTemp, 1, 15, 100);
  else 
    doPointerNavigation(); 
}
void page_MENU_PID(){//=================================================PID===============================================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("PID"), 3);
    changeValues [10]; 
    initPage = false;
  }

  if(menuItemPrintable(1,1)){display1.print(F("Kp_ele   =           "));}
  if(menuItemPrintable(1,2)){display1.print(F("Ki_ele   =           "));}
  if(menuItemPrintable(1,3)){display1.print(F("Back                 "));}

  if(menuItemPrintable(12,1)){printUint32_tAtWidth(settings.Kp_element, 3, ' ');}
  if(menuItemPrintable(12,2)){printDoubleAtWidth(settings.Ki_element, 3, ' ');}
     
  if(btnOk.PressReleased())
  {
    FlashPointer();
    
    switch (cursorPos)
    {
      case 0: changeValues [0] = !changeValues [0]; edditing = !edditing; break;
      case 1: changeValues [1] = !changeValues [1]; edditing = !edditing; break;
      case 2: currPage = MENU_ROOT; sets_Save(); initPage = true; return;
    }
  }
       if(changeValues[0])incrementDecrementDouble(&settings.Kp_element, 1.0, 0.0, 1000.0);
  else if(changeValues[1])incrementDecrementDouble(&settings.Ki_element, 0.1, 0.0, 200.0);
  else 
    doPointerNavigation(); 
}
void page_MENU_TIME(){//=================================================MASH_PROGRAM====================================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("Mash Program"), 9);
    changeValues [10];
    initPage = false;
  }

  if(menuItemPrintable(1,1)){display1.print(F("Manual =            "));}
  if(menuItemPrintable(1,3)){display1.print(F("Temp1  =            "));}
  if(menuItemPrintable(1,4)){display1.print(F("Time1  =            "));}
  if(menuItemPrintable(1,5)){display1.print(F("Temp2  =            "));}
  if(menuItemPrintable(1,6)){display1.print(F("Time2  =            "));}
  if(menuItemPrintable(1,7)){display1.print(F("Temp3  =            "));}
  if(menuItemPrintable(1,8)){display1.print(F("Time3  =            "));}
  if(menuItemPrintable(1,9)){display1.print(F("Back                "));}

  if(menuItemPrintable(12,1)){printOnOff(settings.manualMode);}
  if(menuItemPrintable(1,2)){printDoubleAtWidth(passedTimeS/60.0, 3, 'm');}
  if(menuItemPrintable(12,2)){printUint32_tAtWidth(settings.targetTemp, 3, 'C');}
  if(menuItemPrintable(12,3)){printUint32_tAtWidth(settings.temp1, 3, 'C');}
  if(menuItemPrintable(12,4)){printUint32_tAtWidth(settings.time1, 3, 's');}
  if(menuItemPrintable(12,5)){printUint32_tAtWidth(settings.temp2, 3, 'C');}  
  if(menuItemPrintable(12,6)){printUint32_tAtWidth(settings.time2, 3, 's');}
  if(menuItemPrintable(12,7)){printUint32_tAtWidth(settings.temp3, 3, 'C');}
  if(menuItemPrintable(12,8)){printUint32_tAtWidth(settings.time3, 3, 's');}
   
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
  else if(changeValues[1])incrementDecrementInt(&settings.temp1, 1, 15, settings.maxElementTemp);
  else if(changeValues[2])incrementDecrementInt(&settings.time1, 1, 0, 90);
  else if(changeValues[3])incrementDecrementInt(&settings.temp2, 1, 15, settings.maxElementTemp);
  else if(changeValues[4])incrementDecrementInt(&settings.time2, 1, 0, 90);
  else if(changeValues[5])incrementDecrementInt(&settings.temp3, 1, 15, settings.maxElementTemp);
  else if(changeValues[6])incrementDecrementInt(&settings.time3, 1, 0, 90);
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
    timeLastTouched = millis(); // Update last touched time
    
    // Serial.print("Direction: ");
    // Serial.print(direction);
    // Serial.print(",\t\tNew Cursor Position: ");
    // Serial.print(newCursorPos);
    // Serial.print(",\t\tDisplay Offset: ");
    // Serial.println(dispOffset);
  }
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
        timeLastTouched = millis();
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
  timeLastTouched = millis();
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
  if(settings.power)
  {
    digitalWrite(ledOnPin, HIGH);

    previousPassedTimeS = passedTimeS;

    bool isManual = settings.manualMode;
    passedTimeS = isManual ? 0 : (millis() - tS) / 1000;

    if (isManual) {
      tS = millis();
    }

    settings.targetTemp = isManual ? settings.targetTemp :
    (passedTimeS < settings.times[0]) ? settings.temps[0] :
    (passedTimeS < settings.times[0] + settings.times[1]) ? settings.temps[1] :
    (passedTimeS < settings.times[0] + settings.times[1] + settings.times[2]) ? settings.temps[2] :
    15.0;

    PID_elementTemp.SetTunings(settings.Kp_element, settings.Ki_element, 0);
    PID_elementTemp.SetMode(AUTOMATIC);
    updateAllItems = PID_elementTemp.Compute();

    previousDutyCycle = DutyCycle;
    DutyCycle = Output_mashTemp;  
    ledcWrite(ledChannel, DutyCycle); 
    // Serial.print("DutyCycle: ");
    // Serial.println(DutyCycle);

    client.loop();
    if(TopicArrived)
    {
      // Serial.print("Message arrived: ");
      // Serial.println(mqttpayload);
      //settings.targetTemp = atof(mqttpayload);
      TopicArrived = false;
    }
  }
  else
  {
    digitalWrite(ledOnPin, LOW);
    ledcWrite(ledChannel, 0);
  }
}

void updateSensorValues() {

  unsigned long currentTime = millis();
  if (currentTime - lastReadTime >= readInterval)
  {
    lastReadTime = currentTime;

    Temp_C = convertRawToCelsius(analogRead(NTC_PIN));
    Temp_C = round(Temp_C * 10.0) / 10.0; // Round to 1 decimal place
    
    Serial.print("current_mashTemp:\t");
    Serial.print(current_mashTemp, 1);
    Serial.print(" °C, ");
    Serial.print("Temp_C:\t");
    Serial.println(Temp_C, 1);

  }

  previous_mashTemp = current_mashTemp;
  current_mashTemp = Temp_C;
}


void updateDisp2()
{
  display2.clearBuffer();
  if(menuItemPrintableDisp2(1,1)){display2.print(F("TargetTemp  = "));}
  if(menuItemPrintableDisp2(1,2)){display2.print(F("MashTemp    = "));}
  if(menuItemPrintableDisp2(1,3)){display2.print(F("DC          = "));}

  if(menuItemPrintableDisp2(12,1)){printUint32_tAtWidthDisplay2(settings.targetTemp, 3, 'C');}
  if(menuItemPrintableDisp2(12,2)){printUint32_tAtWidthDisplay2(current_mashTemp, 3, 'C');}
  if(menuItemPrintableDisp2(12,3)){printUint32_tAtWidthDisplay2(DutyCycle/4095.0 * 100.0, 3, '%');}
  display2.sendBuffer();
  display2.clearBuffer();
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
  client.connect("Reflow_Hot_Plate", privates.brokerUser, privates.brokerPass);
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
    if (client.connect("Reflow_Hot_Plate", privates.brokerUser, privates.brokerPass)) {
      Serial.println("MQTT connected!");
    }
  }
}

void publishMessage() //Homeassistant
{
    publishFloat(privates.topicMashTemp, current_mashTemp);
    publishFloat(privates.topicTimePassed, passedTimeS/60.0);
    publishFloat(privates.topicTargetTemp, settings.targetTemp);
    publishFloat(privates.topicDutyCycle, DutyCycle/4096.0 * 100.0);
    publishFloat(privates.topicKp, settings.Kp_element);
    publishFloat(privates.topicKi, settings.Ki_element);
}

void publishFloat(const char* topic, float value) {
  //int "%d", float "%f", bool "%s", obs vikitgt!
  snprintf(messages, sizeof(messages), "%.1f", value); // 2 decimal places
  client.publish(topic, messages);
}

void publishString(const char* topic, String value) {
  snprintf(messages, sizeof(messages), "%s", value);
  client.publish(topic, messages);
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

float convertRawToCelsius(float raw) {

  filtered = Filter(raw, filtered);
  filtered = constrain(map((int)filtered, settings.RawLow, settings.RawHigh, 0, 4095), 0, 4095);
  float vOut = (filtered / 4095.0) * 3.3;    
  float rNTC = (R_ref * vOut) / (3.3 - vOut); 
  // Beta-formeln
  float invT = (1.0 / T_0) + (1.0 / BETA) * log(rNTC / R_NTC);
  float T = 1.0 / invT;    // Kelvin
  return T - 273.15;       // Celsius
}

float Filter(float New, float Current) //Moisture sensor
{
  return (1.0 - settings.filterWeight/100.0) * New + settings.filterWeight/100.0 * Current;
}