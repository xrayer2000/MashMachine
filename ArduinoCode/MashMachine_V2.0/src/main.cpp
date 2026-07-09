#include <Arduino.h>
#include <mash_config.h> //config.h
#include <Wire.h> //temperatur
// #include <Adafruit_GFX.h> //oled
// #include <Adafruit_SSD1306.h> //oled
#include <U8g2lib.h> //oled
// #include <Adafruit_I2CDevice.h>
#include <PI_Controller.h> //PI controller with heat loss compensation
#include "Controllers/LQG_Controller.h"
#include "Controllers/Boil_Controller.h"
#include "Linear_SSM_Models/Model_50C.h"
#include "Linear_SSM_Models/Model_65C.h"
#include "Linear_SSM_Models/Model_75C.h"
#include "Linear_SSM_Models/Model_Boil.h"
#include "Excitation.h" //identification signal generator
#include <PressButton.h> //Interface
#include <RotaryEncoderAccel.h> //Interface
#include <EEPROM.h> //Save Settings
#include <WiFi.h> //Home assistant
#include <PubSubClient.h> //Home assistant
#include <Privates.h> //Homeassistant 
#include <ArduinoJson.h> //Homeassistant
#include <math.h> //math
#include <Adafruit_MAX31865.h> //temperatur
#include <OneWire.h> //temperatur
#include <DallasTemperature.h> //temperatur DS18B20 for oled temp
#include <HTTPClient.h>      //InfluxDB
#include <time.h> //time for InfluxDB

// Hop icon, 48x48px
static const unsigned char hop_logo[] PROGMEM = {
  0x00, 0x00, 0x00, 0xC0, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x70, 0xEF, 0x01, 0x00, 0x00, 0x20, 0x78,
  0x07, 0x00, 0x00, 0x00, 0x20, 0x38, 0x60, 0x0C, 0x00, 0x00, 0x20, 0x38, 0xE0, 0x0D, 0x00, 0x00,
  0x20, 0x82, 0xDF, 0x05, 0x00, 0x00, 0x00, 0x03, 0xBF, 0x01, 0x00, 0x00, 0x00, 0xFB, 0xBC, 0x04,
  0x00, 0x00, 0x00, 0xFF, 0xDD, 0x06, 0x00, 0x38, 0x00, 0xFF, 0x1D, 0x02, 0x00, 0xF8, 0x00, 0xFE,
  0x0D, 0x01, 0x00, 0xE0, 0x01, 0xFE, 0x26, 0x00, 0xC0, 0xE7, 0x01, 0xFC, 0xF0, 0x00, 0xE0, 0xF3,
  0x03, 0x7C, 0x78, 0x00, 0xF0, 0xF3, 0x07, 0x1C, 0x01, 0x00, 0xF0, 0xFB, 0x01, 0x9C, 0x1F, 0x00,
  0x08, 0x78, 0x00, 0xD8, 0x1F, 0x00, 0x60, 0x38, 0xFF, 0x17, 0x07, 0x00, 0x30, 0x9D, 0xFF, 0x0F,
  0x98, 0x00, 0x38, 0xDD, 0xFF, 0x0F, 0xCC, 0x03, 0xBC, 0x4D, 0xFE, 0x19, 0xCC, 0x07, 0x9E, 0x0D,
  0xFE, 0x01, 0x1E, 0x00, 0xC2, 0x4B, 0xFE, 0x09, 0x3E, 0x0C, 0xC0, 0xE3, 0xFE, 0x19, 0x7F, 0x1C,
  0xD6, 0xE7, 0xFC, 0x3D, 0x7F, 0x19, 0xD2, 0xF7, 0xFC, 0x3C, 0x7F, 0x03, 0xD3, 0xF7, 0xFC, 0xFC,
  0x79, 0x47, 0x9B, 0x73, 0xF8, 0x78, 0x81, 0x57, 0x39, 0x38, 0x7A, 0x71, 0xEE, 0x97, 0xF8, 0x19,
  0x36, 0x61, 0x0E, 0x33, 0xFA, 0x50, 0x8E, 0x69, 0x0C, 0x7C, 0x02, 0xD0, 0xFE, 0x49, 0xEC, 0xC0,
  0x06, 0xC3, 0xFC, 0x1D, 0xD8, 0xC0, 0xCE, 0xC1, 0xFC, 0x1C, 0x80, 0x1D, 0xE6, 0xE0, 0xF8, 0x1C,
  0x00, 0x00, 0x00, 0xE0, 0x78, 0x18, 0x00, 0x00, 0x00, 0x40, 0x70, 0x18, 0x00, 0x00, 0x00, 0x40,
  0xA4, 0x18, 0x00, 0x00, 0x00, 0x00, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD, 0x04, 0x00, 0x00,
  0x00, 0x00, 0xF9, 0x06, 0x00, 0x00, 0x00, 0x00, 0x7B, 0x06, 0x00, 0x00, 0x00, 0x00, 0x33, 0x03,
  0x00, 0x00, 0x00, 0x00, 0x0B, 0x03, 0x00, 0x00, 0x00, 0x00, 0x5A, 0x01, 0x00, 0x00, 0x00, 0x00,
  0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
};

//IO-pins
#define dutyCycleOutPin 26 //    green LED
#define ledOnPin 14 //           orange LED
#define ledWaterDetectedPin 4 // red LED
#define ledWifiPin 17 //         blue LED
//#define ledMqttPin 23 //         yellow LED
//rotary encoder
#define outputA 32//32
#define outputB 33//33
#define confirmBtnPin 25 // pin 25 ESP32 - ok
// // MAX31865 PT100
#define MAX31865_CS 5 //MAX31865 chip select
#define MAX31865_SO 19 //MAX31865 serial out
#define MAX31865_SCK 18 //MAX31865 serial clock
#define MAX31865_SI 23 //MAX31865 serial in
//Dallas Temperature - DS18B20
#define ONE_WIRE_BUS 16// 16 //ESP32 - OneWire bus
//Pump
#define pumpPin 27 //ESP32 - Pump
//Larm
#define alarmPin 13//13 // ESP32 - alarm

//Water indicator
//#define waterDetectedPin adc2 ads1115
#define waterDetectedPin 34
//-----------------------------------------------------------------------
// OLED display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
//-----------------------------------------------------------------------
//Settings
#define PACING_MC 30 //25
#define FLASH_RST_CNT 3 //30
#define SETTINGS_CHKVAL 3647 
#define SHIFT_UP 0 //240/8
//display1
uint8_t DISP_ITEM_ROWS; //4
uint8_t DISP_CHAR_WIDTH;
uint8_t CHAR_X;
uint8_t CHAR_Y;
//display2
uint8_t DISP2_ITEM_ROWS; //4
uint8_t DISP2_CHAR_WIDTH;
uint8_t DISP2_CHAR_X;
uint8_t DISP2_CHAR_Y;

//-----------------------------------------------------------------------
//Varibles

//Homeassistant
//-----------------------------------------------------------------------
Privates privates; 
WiFiClient espClient; 
PubSubClient client(espClient); 
char messages[50]; 
volatile  bool TopicArrived = false;
const     int mqttpayloadSize = 16;
char mqttpayload [mqttpayloadSize] = {'\0'};
String mqtttopic;
//-----------------------------------------------------------------------
//rotary encoder
RotaryEncoderAccel encoder(outputA, outputB);  // GPIO32 och GPIO33 på ESP32
// ISR för båda pins, som anropas vid ändring (rising/falling)
void IRAM_ATTR handleInterrupt() {
  encoder.tick();
}
//-----------------------------------------------------------------------
// Buttons
PressButton btnOk(confirmBtnPin, 10); //debounce (ISR-attached inside library)
//-----------------------------------------------------------------------
//System Mode
enum SystemMode {
  MODE_CONTROL_LQG,
  MODE_CONTROL_PI,
  MODE_IDENTIFICATION,
  MODE_MANUAL
};
//Identification type
enum IdentificationType {
  IDENT_SINE,
  IDENT_MULTI_SINE,
  IDENT_STEP_RESPONSE
};

const char* identificationTypeToString(IdentificationType type);
const char* systemModeToString(SystemMode mode);

Excitation excitation;

enum BrewingLocation {
  OutDoor,
  InDoor
};

const char* brewingLocationToString(BrewingLocation location);
void updateAmbientSource(BrewingLocation loc);

//Menu structure
enum pageType{
  MENU_ROOT,
  MENU_MODE,
  MENU_CONTROL_MODE_LQG,
  MENU_CONTROL_MODE_PI,
  MENU_IDENT_MODE,
  MENU_IDENT_TYPE,
  MENU_IDENT_SINE,
  MENU_IDENT_MULTI_SINE,
  MENU_IDENT_STEP_RESPONSE,
  MENU_MANUAL_MODE,
  MENU_MISC,
  MENU_BREWING_LOCATION,
  MENU_MASH_PROGRAM,
  MENU_LAUTERING_PROGRAM,
  MENU_HOPS_PROGRAM,
  MENU_WHIRLPOOL_PROGRAM
};
enum pageType currPage = MENU_ROOT;
void page_MenuRoot();
void page_MENU_MODE();
void page_MENU_CONTROL_MODE_LQG();
void page_MENU_CONTROL_MODE_PI();
void page_MENU_IDENT_MODE();
void page_MENU_IDENT_TYPE();
void page_MENU_IDENT_SINE();
void page_MENU_IDENT_MULTI_SINE();
void page_MENU_IDENT_STEP_RESPONSE();
void page_MENU_MANUAL_MODE();
void page_MENU_MISC();
void page_MENU_BREWING_LOCATION();
void page_MENU_MASH_PROGRAM();
void page_MENU_LAUTERING_PROGRAM();
void page_MENU_HOPS_PROGRAM();
void page_MENU_WHIRLPOOL_PROGRAM();
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
void incrementDecrementFloat(float *v, float amount, float min, float max);
void incrementDecrementDouble(double *v, double amount, double min, double max);
void renderCurrentPage();
void doPointerNavigation();
bool menuItemPrintable(uint8_t xPos, uint8_t yPos);
//-----------------------------------------------------------------------
//Print tools
void printPointer();
void printOnOff(bool val);
void printInt32_tAtWidth(int32_t value, uint8_t width, const char* c);
void printDoubleAtWidth(double value, uint8_t width, const char* c, uint8_t decimals = 1);
//-----------------------------------------------------------------------
//Settings
#pragma pack(1) //memory alignment
struct Mysettings{
  double Kp_mash = 32.0;
  double Ki_mash = 0.00;
  double k_heatLoss = 4.9; //65C
  double deadband1 = 4.0;
  double deadband2 = 0.5;
  double gamma = 0.9;
  double targetTemp = 25;

  SystemMode systemMode = MODE_CONTROL_LQG;

  IdentificationType identificationType;
  bool start_stop = 1;
  int16_t u0 = 10;
  int16_t amplitude = 5;
  int16_t period = 2;
  int16_t duration = 120;

  int16_t amplitude2 = 5;
  int16_t period2 = 2;

  double manualPower = 0;

  BrewingLocation brewingLocation = OutDoor;

  boolean startMashProgram = 0;
  boolean startLauteringProgram = 0;
  boolean startHopProgram = 0;
  boolean startWhirlpoolProgram = 0;

  int16_t mashTemps[3] = {66,76,78};
  int16_t mashTimes[3] = {60,15,15};

  int16_t lauteringTemp = 76;
  int16_t lauteringTime = 30;
  
  int16_t hopTimes[3] = {60, 15, 1};
  int16_t boilTime = 60;
  int16_t boilTemp = 100;
  int16_t hopIndex = -1;

  int16_t whirlpoolTemp = 75;
  int16_t whirlpoolTime = 20;

  int16_t timeBeforeDisable = 2;

  boolean power = true;
  boolean pump = false;
  int16_t maxMashTemp = 100; //220

  double filter_adc1 = 0.00f;
  double filterDC = 0.30f;

  int16_t alarmVolume = 4;
  int16_t alarmTime = 3;

  uint16_t settingsCheckValue = SETTINGS_CHKVAL;
};

Mysettings settings;
Mysettings oldSettings;
void sets_SetDeafault();
void sets_Load();
void sets_Save();
bool pendingMashStart = false;
bool pendingLauterStart = false;
bool pendingHopStart = false;
bool pendingWhirlpoolStart = false;
bool prevStartMashProgram_ = false;
bool prevStartLauteringProgram_ = false;
bool prevStartHopProgram_ = false;
bool prevStartWhirlpoolProgram_ = false;
//-----------------------------------------------------------------------
//Time
unsigned long previousTime = 0; 
unsigned long currentTime;
double tS;
long passedTimeS_mashProgram = 0;
long passedTimeS_lauteringProgram = 0;
long passedTimeS_hopProgram = 0;
long passedTimeS_whirlpoolProgram = 0;
// Mash step state
int8_t currentMashStep = 0;
bool mashStepAtTemp = false;
unsigned long mashStepTimerStartMs = 0;
unsigned long mashCompletedTimeMs = 0;

unsigned long previousSensorRead = 0;
const unsigned long saveInterval = 60000; // 60 000 ms = 60 sekunder
unsigned long lastEditTime = 0;
//Oled temperature sensor DS18B20
double current_oledTemp = 0;
//-----------------------------------------------------------------------
//CONTROLLER
double Output_mashTemp;
double current_mashTemp;
double previous_mashTemp;
double rawTemp;

double current_airTemp;
double outdoorTemp; //received from home assistant (Norwegian Meteorological Institute)
double previous_outdoorTemp;
double indoorTemp; //received from home assistant (STM32f411 and esp32 xiao with bmp280 sensor)
double previous_indoorTemp;
double* ambientTempPtr = &current_airTemp;
double previous_airTemp;

double DutyCycle = 0;
double previousDutyCycle = 0;
//                 y(t), u(t), r(t) 
// PID PID_mashTemp(&current_mashTemp, &Output_mashTemp, &settings.targetTemp, &current_airTemp, settings.Kp_mash, settings.Ki_mash, 0, 0, DIRECT); //P_ON_M
PI_Controller mashCtrl_PI(&current_mashTemp, &Output_mashTemp, &settings.targetTemp, ambientTempPtr); //PI controller with heat loss compensation
double lastKp = NAN;
double lastKi = NAN;
double lastKHeat = NAN;
double lastDeadband1 = NAN;
double lastDeadband2 = NAN;
double lastGamma = NAN;
LQGController mashCtrlLQG(&current_mashTemp, &Output_mashTemp, &settings.targetTemp, ambientTempPtr);     //LQG controller
static ModelID activeModel = MODEL_65; 
double currentHeaterPower_W = 0.0;
BoilController boilCtrl(modelBoil); //Boil controller with feedforward from boil model
//-----------------------------------------------------------------------
// setting PWM properties
int freq = 240; //5
const int DC_Channel = 0;
const int resolution = 12;
//-----------------------------------------------------------------------
// Oled - Adafruit_SSD1306
U8G2_SH1106_128X64_NONAME_F_HW_I2C display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C display2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
u16_t timeLastTouched = 0;
bool displaySleeping = false;
//Oled - 2.42"
//-----------------------------------------------------------------------
double passedTime, previousPassedTime1, previousPassedTime2 = 0;
bool initPage = true;
bool changeValues [20];
//-----------------------------------------------------------------------
//Dallas Temperature - Air temperature
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature dallasTemp(&oneWire);
//-----------------------------------------------------------------------
//MAX31865 PT100 - Mash temperature
Adafruit_MAX31865 PT100(MAX31865_CS, MAX31865_SI, MAX31865_SO, MAX31865_SCK);
double tempPT100 = 0;
double filteredTempPT100 = 0;
unsigned long lastReadTime = 0;
const unsigned long readInterval = 1000;  // ms
// ====== CONFIG ======
#define RREF      435.8
#define RNOMINAL  100.0
// ---- Replace these with your measured values ----
double const CAL_T1  = 7.1;     // Real temperature point 1 (ice bath)
double const CAL_T2  = 100.0;   // Real temperature point 2 (boiling water)

double const CAL_M1  = 7.5;     // Measured temperature at T1
double const CAL_M2  = 95.3;   // Measured temperature at T2
// ================================================
// Calibration coefficients (computed automatically)5
double a;
double b;

//-----------------------------------------------------------------------
//Alarm/Notifications
bool mashFinishedNotified = false;
bool lauteringFinishedNotified = false;
bool hopFinishedNotified[3] = {false, false, false};
bool whirlpoolFinishedNotified = false;

// Reminder timers - repeat alarm every 5 min until the program is switched
unsigned long lastMashReminderTime = 0;
unsigned long lastLauteringReminderTime = 0;
unsigned long lastWhirlpoolReminderTime = 0;
const unsigned long REMINDER_INTERVAL_MS = 5UL * 60UL * 1000UL; // 5 minutes
//-----------------------------------------------------------------------
//Water indicator
bool waterDetected = true;
bool tooHotForPump = false;
bool pumpWater = false; 
//-----------------------------------------------------------------------
// Loop time measurement variables
static uint32_t last  = 0;
uint32_t now  = 0;
uint32_t loopTime  = 0;
uint32_t sumLoopTime = 0;
uint32_t loopCount = 0;
uint32_t avgLoopTime = 0;

// ============================================================
// Forward declarations
// ============================================================

float roundTo(float value, int decimals);

double Filter(double input,double previous,double alpha,double maxValue);

void setupWiFi();
void setupMQTT();
void setupTime();

void reconnectWiFi();
void reconnectMQTT();

void handleMQTT();

void subscribeMQTT();
void publishWifiMqttStatus();
void publishMessage();

void resetAllAlarmsOnBoot();
void updateSettings();
void updateSensorValues();
void updateDisp2();

void updateModelForCurrentTemp(LQGController& controller,double T,ModelID& activeModel);
bool writeInflux(float mashTemp,float powerW,double ambientTemp);
void callback(char* topic,byte* payload,unsigned int length);
void FlashPointer();
void printStringAtWidth(const char* str,uint8_t width);
void incrementDecrementInt(int16_t* value,int16_t amount,int16_t minVal, int16_t maxVal);
void incrementDecrementDouble(double* value,double amount,double minVal,double maxVal);


void setup() {//=================================================SETUP=======================================================
  Serial.begin(115200);

  // WiFi.disconnect(true);
  // WiFi.mode(WIFI_OFF);

  PT100.begin(MAX31865_3WIRE);
   // Calculate calibration constants
  a = (CAL_T2 - CAL_T1) / (CAL_M2 - CAL_M1);
  b = CAL_T1 - a * CAL_M1;

  currentTime = millis();

  //initialize Dallas Temperature, oled temperature sensor
  dallasTemp.begin();
  dallasTemp.setWaitForConversion(false);
  dallasTemp.requestTemperatures(); 
  current_oledTemp = dallasTemp.getTempCByIndex(0);
  Serial.print("Dallas sensors found: ");
  Serial.println(dallasTemp.getDeviceCount());

  // configure LED PWM functionalitites
  ledcSetup(DC_Channel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(dutyCycleOutPin, DC_Channel);

  pinMode(ledOnPin, OUTPUT);
  pinMode(ledWaterDetectedPin, OUTPUT);
  pinMode(ledWifiPin, OUTPUT);
  // pinMode(ledMqttPin, OUTPUT);
  pinMode(confirmBtnPin, INPUT_PULLUP);
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  //pinMode(NTC_PIN, INPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(alarmPin, OUTPUT);
  pinMode(waterDetectedPin, INPUT_PULLUP);

  // Initialize encoder interrupts (confirm button interrupt disabled)
  attachInterrupt(digitalPinToInterrupt(outputA), handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), handleInterrupt, CHANGE);

  Wire.begin(21, 22);           // pins only

  Serial.println("Scanning Wire...");
  for (uint8_t addr = 1; addr < 127; addr++)
  {
      Wire.beginTransmission(addr);
      uint8_t error = Wire.endTransmission();
      if (error == 0)
      {
          Serial.print("Found device at 0x");
          Serial.println(addr, HEX);
      }
  }
  Serial.println("Wire done");

  display1.setI2CAddress(0x3C << 1); 
  display1.begin();  
  display1.setBusClock(1000000);   
  display1.setFont(u8g2_font_6x12_mr);	// choose a suitable font
  display1.clearBuffer();					// clear the internal memory
  display1.setCursor(0, 0);

  DISP_CHAR_WIDTH = SCREEN_WIDTH / (display1.getMaxCharWidth()  + 0);          // hur många tecken per rad: display1.getMaxCharWidth();
  DISP_ITEM_ROWS =  SCREEN_HEIGHT / (display1.getMaxCharHeight()  + 3);         // max rader som får plats
  
  CHAR_X = display1.getMaxCharWidth() + 2; //margin
  CHAR_Y = display1.getMaxCharHeight() + 2; //margin

  display2.setI2CAddress(0x3D << 1);
  display2.begin();  
  display2.setBusClock(1000000); 
  display2.setFont(u8g2_font_6x12_mr);	// choose a suitable font  
  display2.clearBuffer();					// clear the internal memory
  display2.setCursor(0, 0);

  DISP2_CHAR_WIDTH = SCREEN_WIDTH / (display2.getMaxCharWidth()  + 2);          // hur många tecken per rad: display2.getMaxCharWidth();
  DISP2_ITEM_ROWS =  4;         // max rader som får plats
  
  DISP2_CHAR_X = display2.getMaxCharWidth() + 0; //margin
  DISP2_CHAR_Y = (SCREEN_HEIGHT / DISP2_ITEM_ROWS) + 0; //margin

  //---------------------------------------------------------------------------------------
  //Welcome Screen 3 seconds
  display1.clearBuffer();
  display1.drawXBM(40, 4, 48, 48, hop_logo);   // centered on 128px wide display
  display1.setFont(u8g2_font_6x10_tr);
  display1.drawStr(5, 63, "Emil's MashMachine");
  display1.sendBuffer();

  display1.setFont(u8g2_font_6x10_tr);	// choose a suitable font
  display1.clearBuffer();					// clear the internal memory
  display1.setCursor(0, 0);
  //----------------------------------------------------------------------------------------

  EEPROM.begin(sizeof(settings));
  sets_Load();

  //PT100 Temperature probe
  PT100.readRTD();
  rawTemp = PT100.temperature(RNOMINAL, RREF);

  // Apply two-point calibration
  
  double calibratedTemp = a * rawTemp + b;
  calibratedTemp = roundTo(calibratedTemp, 3); // Round to 3 decimal places
  tempPT100 = calibratedTemp;
  filteredTempPT100 = calibratedTemp;

  current_mashTemp = tempPT100; //Temp_C;
  previous_mashTemp = tempPT100; //Temp_C;

  updateAmbientSource(settings.brewingLocation);

  setupWiFi(); //Home assistant
  setupMQTT(); //Home assistant
  resetAllAlarmsOnBoot(); //Home assistant

  WiFi.onEvent([](WiFiEvent_t event) {
    if (event == SYSTEM_EVENT_STA_DISCONNECTED) {
      if (client.connected()) {
        client.publish("esp32/wifi", "offline", true);
      }
    }
  });

  setupTime(); //NTP time for InfluxDB

  //(PI controller)
  mashCtrl_PI.SetSampleTime(1000);   // ms
  // --- Output range ---
  mashCtrl_PI.SetHeaterPowerLimits(0.0, heaterRatedPower_W);
  // --- Tunings ---
  mashCtrl_PI.SetTunings(settings.Kp_mash, settings.Ki_mash, settings.k_heatLoss);
  // --- Safety / behavior ---
  mashCtrl_PI.SetCaptureBands(settings.deadband1, settings.deadband2, settings.gamma); // °C  // Switch from full power to regulation at ±5 °C
  mashCtrl_PI.SetIntegralLimit(0.25 * heaterRatedPower_W);
  // Start with feedforward only
  double Q0 = settings.k_heatLoss * (current_mashTemp - current_airTemp);
  mashCtrl_PI.Reset(Q0);

  //LQG Controller 
  // 1. Initiera LQG exakt där systemet är
  mashCtrlLQG.SetHeaterPowerLimits(0.0, heaterRatedPower_W);
  
  mashCtrlLQG.loadModel(model65);        // 1. modell (MUST BE FIRST!)
  mashCtrlLQG.InitEstimator();           // 2. estimator-tillstånd
  
  // 2. Kör feedforward först (ONLY AFTER model is loaded!)
  Output_mashTemp = mashCtrlLQG.model_->kLoss_W_per_degC * (current_mashTemp - current_airTemp);
  Output_mashTemp = constrain(Output_mashTemp, 0, 4095);

  // 3. Beräkna faktisk effekt i W
  currentHeaterPower_W = Output_mashTemp * PWM_to_W;

  // 4. Reset med rätt initialvärde
  mashCtrlLQG.Init(currentHeaterPower_W); // 5. bumpless start

  // settings.pump = false;


}

void loop() { //=================================================LOOP=======================================================


  // loop time measurement
  // ---------------------------------------------------------
  now = micros();

  loopCount++;

  if (loopCount == 1)
      last = now;

  if (loopCount >= 100)
  {
      sumLoopTime = now - last;
      avgLoopTime = sumLoopTime / (loopCount - 1);

      if (avgLoopTime == 0) avgLoopTime = 1;

      loopCount = 0;
      last = now;
  }

  uint32_t freq = (avgLoopTime > 0) ? (1000000UL / avgLoopTime) : 0;

  // Serial.printf(
  //   "Loop:%4lu | LoopFreq:%6lu\n",
  //   avgLoopTime,
  //   freq
  // );
  // ---------------------------------------------------------

  passedTime = millis() * 0.001f; 

  float minutesSinceLastAction = (passedTime * 0.0166667f) - timeLastTouched;
  bool shouldSleep = (minutesSinceLastAction > settings.timeBeforeDisable);

  if (shouldSleep != displaySleeping)
  {
      displaySleeping = shouldSleep;
      display1.setPowerSave(displaySleeping);
      display2.setPowerSave(displaySleeping);
  }
    
  const float UPDATE_INTERVAL1 = 0.10;
  const float UPDATE_INTERVAL2 = 0.1;
  bool shouldUpdate1 = (passedTime - previousPassedTime1 >= UPDATE_INTERVAL1);
  bool shouldUpdate2 = (passedTime - previousPassedTime2 >= UPDATE_INTERVAL2);
  updateSettings();
  
  if(shouldUpdate1)
  {
    updateSensorValues(); 
    
    previousPassedTime1 = passedTime;

    // ===================================
    // Start Stop for Identification
    // ===================================

    static bool prevStartStop = false;
    static unsigned long identStartMs = 0;

    if(settings.systemMode == MODE_IDENTIFICATION)
    {
        if(settings.start_stop && !prevStartStop)
        {
            if(settings.identificationType == IDENT_SINE)
            {
                excitation.setType(Excitation::SINE);
                excitation.setSine(settings.u0,
                                  settings.amplitude,
                                  settings.period * 60.0);
            }
            else if(settings.identificationType == IDENT_MULTI_SINE)
            {
                excitation.setType(Excitation::MULTISINE);
                excitation.setMultiSine(settings.u0,
                                        settings.amplitude,  settings.period  * 60.0,
                                        settings.amplitude2, settings.period2 * 60.0);
            }
            else
            {
                excitation.setType(Excitation::STEP);
                excitation.setStep(settings.u0);
            }

            excitation.start();
            identStartMs = millis();   // store start time
        }

        // ---- Automatic stop after duration ----
        if(settings.start_stop)
        {
            double duration_s = settings.duration * 60.0;  // minutes → seconds

            if(duration_s > 0)
            {
                unsigned long elapsedMs = millis() - identStartMs;

                if(elapsedMs >= (unsigned long)(duration_s * 1000.0))
                {
                    settings.start_stop = false;
                    excitation.stop();
                }
            }
        }

        if(!settings.start_stop && prevStartStop)
        {
            excitation.stop();
        }
    }

    prevStartStop = settings.start_stop;

    float Q_W = 0.0;
    // =========================
    // MODE SUPERVISOR
    // =========================

    updateModelForCurrentTemp(mashCtrlLQG, current_mashTemp, activeModel);

    switch(settings.systemMode)
    {
      case MODE_CONTROL_LQG:
          
        if (activeModel == MODEL_Boil)
        {
          boilCtrl.setInputs(current_mashTemp, *ambientTempPtr, settings.boilTemp);
          boilCtrl.Compute();
          Q_W = boilCtrl.GetCurrentPower_W();
        }
        else
        {
          mashCtrlLQG.Compute();
          Q_W = mashCtrlLQG.GetCurrentPower_W();
        }

        break;

      case MODE_CONTROL_PI:

        if (activeModel == MODEL_Boil)
        {
          boilCtrl.setInputs(current_mashTemp, *ambientTempPtr, settings.boilTemp);
          boilCtrl.Compute();
          Q_W = boilCtrl.GetCurrentPower_W();
        }
        else
        {
          mashCtrl_PI.Compute();
          Q_W = mashCtrl_PI.GetCurrentPower_W();
        }
          break;

     case MODE_IDENTIFICATION:

        if(settings.start_stop)
        {
            switch(settings.identificationType)
            {
                case IDENT_SINE:
                case IDENT_MULTI_SINE:
                case IDENT_STEP_RESPONSE:
                    Q_W = excitation.compute();
                    break;

                default:
                    Q_W = 0.0;
                    break;
            }
        }
        else
        {
            Q_W = 0.0;
        }
        break;

      case MODE_MANUAL:
          Q_W = settings.manualPower;
          break;
    }

    // Saturate power
    Q_W = constrain(Q_W, 0.0, heaterRatedPower_W);

    // Convert to PWM
    float rawDuty = Q_W * W_to_PWM;

    if(settings.power && waterDetected) 
    {
      previousDutyCycle = DutyCycle;
      DutyCycle = Filter(rawDuty , previousDutyCycle, settings.filterDC, 4095.0); 
    }
    else 
    {
      DutyCycle = 0;
    }
    // --- Apply output ---
    ledcWrite(DC_Channel, DutyCycle);
    // Serial.print(", DutyCycle: ");
    // Serial.print(DutyCycle);
  }

  if (updateAllItems)
    display1.clearBuffer();

  renderCurrentPage();

  if (updateAllItems || updateItemValue)
  {
    printPointer(); // added
    display1.sendBuffer();                        //välldigt långsam
    updateAllItems = false;
    updateItemValue = false;
  }

  // printPointer();     // this one take long time
  if (current_mashTemp != previous_mashTemp || current_airTemp != previous_airTemp || DutyCycle != previousDutyCycle || settings.targetTemp != oldSettings.targetTemp)
  {
    updateDisp2();
    previousPassedTime2 = passedTime;
  } 
  
  //Homeassistant
  if(passedTime - previousTime >= 1.0) {publishMessage(); previousTime = passedTime;}

  detectedRotation = encoder.getDirection() != RotaryEncoderAccel::Direction::NOROTATION;
  if(detectedRotation)
  {
    lastEditTime = millis();
  }
  // Consume confirm button timestamp from library ISR and update touch timers
  {
    uint32_t t = btnOk.consumeTouchMs();
    if (t != 0) {
      timeLastTouched = t/1000.0/60.0;
      lastEditTime = t;
    }
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

    initMenuPage(F("MAIN MENU"), 6);
    initPage = false;
  }

  if(btnOk.Pressed())
  {
    FlashPointer();
    root_pntrPos = cursorPos;
    root_dispOffset = dispOffset;
    initPage = true;
    switch (cursorPos)
    {
      case 0: currPage = MENU_MODE;               changeValues[0] = false; edditing = false; return;
      case 1: currPage = MENU_MISC;               changeValues[1] = false; edditing = false; return;
      case 2: currPage = MENU_MASH_PROGRAM;       changeValues[2] = false; edditing = false; return;
      case 3: currPage = MENU_LAUTERING_PROGRAM;  changeValues[3] = false; edditing = false; return;
      case 4: currPage = MENU_HOPS_PROGRAM;       changeValues[4] = false; edditing = false; return;
      case 5: currPage = MENU_WHIRLPOOL_PROGRAM;  changeValues[5] = false; edditing = false; return;
    }
  }
  else
    doPointerNavigation();

  if(!(updateAllItems || updateItemValue)) return;

  for(uint8_t i = 1; i <= 6; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("MODE ->            ")); break;
        case 2: display1.print(F("MISC               ")); break;
        case 3: display1.print(F("MASH PROGRAM       ")); break;
        case 4: display1.print(F("LAUTERING PROGRAM  ")); break;
        case 5: display1.print(F("HOPS PROGRAM       ")); break;
        case 6: display1.print(F("WHIRLPOOL PROGRAM  ")); break;
      }
    }

    if(menuItemPrintable(7, i))
    {
      switch(i)
      {
        case 1: printStringAtWidth(systemModeToString(settings.systemMode), 4); break;
      }
    }
  }
}

void page_MENU_MODE(){//=================================================MODE============================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("MODE"), 6);
    initPage = false;
  }

  if(btnOk.Pressed())
  {
    switch (cursorPos)
    {
      case 0: FlashPointer(); initPage = true; currPage = MENU_CONTROL_MODE_LQG; return;
      case 1: FlashPointer(); initPage = true; currPage = MENU_CONTROL_MODE_PI; return;
      case 2: FlashPointer(); initPage = true; currPage = MENU_IDENT_MODE; return;
      case 3: FlashPointer(); initPage = true; currPage = MENU_MANUAL_MODE; return;
      case 5: FlashPointer(); initPage = true; currPage = MENU_ROOT; sets_Save(); return;
      default: return;
    }
  }
  else
    doPointerNavigation();


  if(!(updateAllItems || updateItemValue)) return;

  for(uint8_t i = 1; i <= 6; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {

        case 1: display1.print(F("CONTROL MODE (LQG)")); break;
        case 2: display1.print(F("CONTROL MODE (PI) ")); break;
        case 3: display1.print(F("IDENT MODE        ")); break;
        case 4: display1.print(F("MANUAL MODE       ")); break;
        case 5: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 6: display1.print(F("Back              ")); break;
      }
    }
  }
}

void page_MENU_CONTROL_MODE_LQG(){//=================================================CONTROL_MODE_LQG============================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("CONTROL MODE (LQG)"), 8);
    memset(changeValues, 0, sizeof(changeValues));
    initPage = false;
  }

  if(btnOk.Pressed())
  {
    FlashPointer();
    changeValues[cursorPos] = !changeValues[cursorPos];
    edditing = !edditing;
  }

       if(changeValues[0]){*&settings.systemMode = MODE_CONTROL_LQG; changeValues [0] = false; updateItemValue = true; }
  else if(changeValues[2]){incrementDecrementDouble(&settings.targetTemp, 1.0, 15.0, settings.maxMashTemp); settings.targetTemp = round(settings.targetTemp);}
  else if(changeValues[3])incrementDecrementDouble(&settings.deadband1, 0.5, settings.deadband2, 20.0);
  else if(changeValues[4])incrementDecrementDouble(&settings.deadband2, 0.1, 0.0, settings.deadband1);
  else if(changeValues[5])incrementDecrementDouble(&settings.gamma, 0.01, 0.0, 1.0);
  else if(changeValues[7]) {currPage = MENU_MODE; sets_Save(); changeValues[7] = false; initPage = true;}
  else 
    doPointerNavigation(); 

  if(!(updateAllItems || updateItemValue)) return;

  display1.drawVLine(9 * CHAR_X, 0, 64);

  for(uint8_t i = 1; i <= 8; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("Activate            ")); break;
        case 2: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 3: display1.print(F("Setpoint           ")); break;
        case 4: display1.print(F("Band 1             ")); break;
        case 5: display1.print(F("Band 2             ")); break;
        case 6: display1.print(F("Gamma              ")); break;
        case 7: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 8: display1.print(F("Back                ")); break;
      }
    }
    if(menuItemPrintable(10, i))
    {
      switch(i)
      {
        case 1: printOnOff(settings.systemMode == MODE_CONTROL_LQG); break;
        case 3: printInt32_tAtWidth((uint32_t)settings.targetTemp, 3, "C"); break;
        case 4: printDoubleAtWidth(settings.deadband1, 4, "C"); break;
        case 5: printDoubleAtWidth(settings.deadband2, 3, "C"); break;
        case 6: printDoubleAtWidth(settings.gamma, 4, " ", 2); break;
      }
    }
  }
}
void page_MENU_CONTROL_MODE_PI(){//=================================================CONTROL_MODE_PI============================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("CONTROL MODE (PI)"), 11);
    memset(changeValues, 0, sizeof(changeValues));
    initPage = false;
  }

  if(btnOk.Pressed())
  {
    FlashPointer();
    changeValues[cursorPos] = !changeValues[cursorPos];
    edditing = !edditing;
  }

       if(changeValues[0]){*&settings.systemMode = MODE_CONTROL_PI; changeValues [0] = false; updateItemValue = true; }
  else if(changeValues[2]){incrementDecrementDouble(&settings.targetTemp, 1.0, 15.0, settings.maxMashTemp); settings.targetTemp = round(settings.targetTemp);}
  else if(changeValues[3])incrementDecrementDouble(&settings.Kp_mash, 0.5, 0.0, 500.0);
  else if(changeValues[4])incrementDecrementDouble(&settings.Ki_mash, 0.01, 0.0, 10.0);
  else if(changeValues[5])incrementDecrementDouble(&settings.k_heatLoss, 0.1, 0.0, 25.0);
  else if(changeValues[6])incrementDecrementDouble(&settings.deadband1, 0.5, settings.deadband2, 20.0);
  else if(changeValues[7])incrementDecrementDouble(&settings.deadband2, 0.1, 0.0, settings.deadband1);
  else if(changeValues[8])incrementDecrementDouble(&settings.gamma, 0.01, 0.0, 1.0);
  else if(changeValues[10]) {currPage = MENU_MODE; sets_Save(); changeValues[10] = false; initPage = true;}
  else 
    doPointerNavigation(); 

  if(!(updateAllItems || updateItemValue)) return;

  display1.drawVLine(9 * CHAR_X, 0, 64);

  for(uint8_t i = 1; i <= 11; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("Activate           ")); break;
        case 2: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 3: display1.print(F("Setpoint           ")); break;
        case 4: display1.print(F("Kp                 ")); break;
        case 5: display1.print(F("Ki                 ")); break;
        case 6: display1.print(F("k_heat             ")); break;
        case 7: display1.print(F("Band 1             ")); break;
        case 8: display1.print(F("Band 2             ")); break;
        case 9: display1.print(F("Gamma              ")); break;
        case 10: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 11: display1.print(F("Back                ")); break;
      }
    }
    if(menuItemPrintable(10, i))
    {
      switch(i)
      {
        case 1: printOnOff(settings.systemMode == MODE_CONTROL_PI); break;
        case 3: printInt32_tAtWidth((uint32_t)settings.targetTemp, 3, "C"); break;
        case 4: printDoubleAtWidth(settings.Kp_mash, 4, "W/C"); break;
        case 5: printDoubleAtWidth(settings.Ki_mash, 4, "W/Cs", 2); break;
        case 6: printDoubleAtWidth(settings.k_heatLoss, 4, "W/C"); break;
        case 7: printDoubleAtWidth(settings.deadband1, 4, "C"); break;
        case 8: printDoubleAtWidth(settings.deadband2, 3, "C"); break;
        case 9: printDoubleAtWidth(settings.gamma, 4, " ", 2); break;
      }
    }
  }
}
void page_MENU_IDENT_MODE(){//=================================================IDENT_MODE============================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("IDENT MODE"), 6);
    memset(changeValues, 0, sizeof(changeValues));
    initPage = false;
  }

  if(btnOk.Pressed())
  {
    FlashPointer();
    changeValues[cursorPos] = !changeValues[cursorPos];
    edditing = !edditing;
  }

       if(changeValues[0]){*&settings.systemMode = MODE_IDENTIFICATION; changeValues [0] = false; updateItemValue = true; }
  else if(changeValues[2]){currPage = MENU_IDENT_TYPE; changeValues[2] = false; initPage = true; return;}
  else if(changeValues[3]){*&settings.start_stop = !*&settings.start_stop; changeValues [3] = false; updateItemValue = true; }
  else if(changeValues[5]) {currPage = MENU_MODE; sets_Save(); changeValues[5] = false; initPage = true;}
  else 
    doPointerNavigation(); 

  if(!(updateAllItems || updateItemValue)) return;

  display1.drawVLine(10 * CHAR_X, 0, 64);

  for(uint8_t i = 1; i <= 6; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("Activate            ")); break;
        case 2: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 3: display1.print(F("Excit Type          ")); break;
        case 4: display1.print(F("Start               ")); break;
        case 5: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 6: display1.print(F("Back                ")); break;
      }
    }
    if(menuItemPrintable(11, i))
    {
      switch(i)
      {
        case 1: printOnOff(settings.systemMode == MODE_IDENTIFICATION); break;
        case 3: printStringAtWidth(identificationTypeToString(settings.identificationType), 4); break;
        case 4: printOnOff(settings.start_stop); break; 
      }
    }
  }


}
void page_MENU_IDENT_TYPE(){//=================================================IDENT_TYPE============================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("IDENT TYPE"), 4);
    initPage = false;
  }

  if(btnOk.Pressed())
  {
    FlashPointer();
    
    switch (cursorPos)
    {
      case 0: settings.identificationType = IDENT_SINE;             currPage = MENU_IDENT_SINE; initPage = true; return;
      case 1: settings.identificationType = IDENT_MULTI_SINE;       currPage = MENU_IDENT_MULTI_SINE; initPage = true; return;
      case 2: settings.identificationType = IDENT_STEP_RESPONSE;    currPage = MENU_IDENT_STEP_RESPONSE; initPage = true; return;
      case 3: currPage = MENU_IDENT_MODE; sets_Save();                                          initPage = true; return;
    }
  }

  doPointerNavigation(); 

  
  if(menuItemPrintable(1,1)){display1.print(F("Sine wave       "));}
  if(menuItemPrintable(1,2)){display1.print(F("MultiSine wave  "));}
  if(menuItemPrintable(1,3)){display1.print(F("Step Resp       "));}
  if(menuItemPrintable(1,4)){display1.print(F("Back            "));}
}

void page_MENU_IDENT_SINE(){//=================================================IDENT_SINE============================================

  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("IDENT SINE"), 6);
    memset(changeValues, 0, sizeof(changeValues));
    initPage = false;
  }

  
  if(btnOk.Pressed())
  {
    FlashPointer();
    changeValues[cursorPos] = !changeValues[cursorPos];
    edditing = !edditing;
  }
       if(changeValues[0]){incrementDecrementInt(&settings.u0, 1, 0.0, heaterRatedPower_W); excitation.setSine(settings.u0, settings.amplitude, settings.period * 60.0);}
  else if(changeValues[1]){incrementDecrementInt(&settings.amplitude, 1.0, 0.0, 400.0); excitation.setSine(settings.u0, settings.amplitude, settings.period * 60.0);}
  else if(changeValues[2]){incrementDecrementInt(&settings.period, 1.0, 5.0, 60.0); excitation.setSine(settings.u0, settings.amplitude, settings.period * 60.0);}
  else if(changeValues[3])incrementDecrementInt(&settings.duration, 5.0, 0.0, 300.0);
  else if(changeValues[5]) {currPage = MENU_IDENT_TYPE; sets_Save(); changeValues[5] = false; initPage = true;}
  else 
    doPointerNavigation(); 

  if(!(updateAllItems || updateItemValue)) return;

  display1.drawVLine(10 * CHAR_X, 0, 64);

  for(uint8_t i = 1; i <= 6; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("Base Power          ")); break;
        case 2: display1.print(F("Amplitude           ")); break;
        case 3: display1.print(F("Period              ")); break;
        case 4: display1.print(F("Duration            ")); break;
        case 5: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 6: display1.print(F("Back                ")); break;
      }
    }
    if(menuItemPrintable(11, i))
    {
      switch(i)
      {
        case 1: printInt32_tAtWidth(settings.u0, 4, "W"); break;
        case 2: printInt32_tAtWidth(settings.amplitude, 4, "W"); break;
        case 3: printInt32_tAtWidth(settings.period, 4, "m"); break;
        case 4: printInt32_tAtWidth(settings.duration, 4, "m"); break;
      }
    }
  }
}

void page_MENU_IDENT_MULTI_SINE(){//=================================================IDENT_MULTI_SINE============================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("IDENT MULTI SINE"), 8);
    memset(changeValues, 0, sizeof(changeValues));
    initPage = false;
  }

  
  if(btnOk.Pressed())
  {
    FlashPointer();
    changeValues[cursorPos] = !changeValues[cursorPos];
    edditing = !edditing;
  }

    auto applyMultiSine = [&](){
    excitation.setMultiSine(settings.u0,
                            settings.amplitude,  settings.period  * 60.0,
                            settings.amplitude2, settings.period2 * 60.0);
  };

       if(changeValues[0]){incrementDecrementInt(&settings.u0, 1, 0.0, heaterRatedPower_W); applyMultiSine();}
  else if(changeValues[1]){incrementDecrementInt(&settings.amplitude, 1.0, 0.0, 400.0);     applyMultiSine();}
  else if(changeValues[2]){incrementDecrementInt(&settings.period, 1.0, 5.0, 60.0);         applyMultiSine();}
  else if(changeValues[3]){incrementDecrementInt(&settings.amplitude2, 1.0, 0.0, 400.0);    applyMultiSine();}
  else if(changeValues[4]){incrementDecrementInt(&settings.period2, 1.0, 5.0, 60.0);        applyMultiSine();}
  else if(changeValues[5])incrementDecrementInt(&settings.duration, 5.0, 0.0, 300.0);
  else if(changeValues[7]) {currPage = MENU_IDENT_TYPE; sets_Save(); changeValues[7] = false; initPage = true;}
  else 
    doPointerNavigation(); 

  if(!(updateAllItems || updateItemValue)) return;

  display1.drawVLine(10 * CHAR_X, 0, 64);

  for(uint8_t i = 1; i <= 8; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("Base Power          ")); break;
        case 2: display1.print(F("Amplitude           ")); break;
        case 3: display1.print(F("Period              ")); break;
        case 4: display1.print(F("Amplitude 2         ")); break;
        case 5: display1.print(F("Period 2            ")); break;
        case 6: display1.print(F("Duration            ")); break;
        case 7: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 8: display1.print(F("Back                ")); break;
      }
    }
    if(menuItemPrintable(11, i))
    {
      switch(i)
      {
        case 1: printInt32_tAtWidth(settings.u0, 4, "W"); break;
        case 2: printInt32_tAtWidth(settings.amplitude, 4, "W"); break;
        case 3: printInt32_tAtWidth(settings.period, 4, "m"); break;
        case 4: printInt32_tAtWidth(settings.amplitude2, 4, "W"); break;
        case 5: printInt32_tAtWidth(settings.period2, 4, "m"); break;
        case 6: printInt32_tAtWidth(settings.duration, 4, "m"); break;
      }
    }
  }
}
void page_MENU_IDENT_STEP_RESPONSE(){//=================================================IDENT_STEP_RESPONSE============================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("IDENT STEP RESPONSE"), 4);
    memset(changeValues, 0, sizeof(changeValues));
    initPage = false;
  }

  if(btnOk.Pressed())
  {
    FlashPointer();
    changeValues[cursorPos] = !changeValues[cursorPos];
    edditing = !edditing;
  }

       if(changeValues[0]){ incrementDecrementInt(&settings.u0,       1.0, 0.0, heaterRatedPower_W); excitation.setStep(settings.u0); }
  else if(changeValues[1])  incrementDecrementInt(&settings.duration,  5.0, 0.0, 300.0);
  else if(changeValues[3]){ currPage = MENU_IDENT_TYPE; sets_Save(); changeValues[2] = false; initPage = true; }
  else
    doPointerNavigation();

  if(!(updateAllItems || updateItemValue)) return;

  display1.drawVLine(10 * CHAR_X, 0, 64);

  for(uint8_t i = 1; i <= 4; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("Step Power          ")); break;
        case 2: display1.print(F("Duration            ")); break;
        case 3: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 4: display1.print(F("Back                ")); break;
      }
    }
    if(menuItemPrintable(11, i))
    {
      switch(i)
      {
        case 1: printInt32_tAtWidth(settings.u0,       4, "W"); break;
        case 2: printInt32_tAtWidth(settings.duration, 4, "m"); break;
      }
    }
  }
}
void page_MENU_MANUAL_MODE(){//=================================================MANUAL_MODE============================================

  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("MANUAL MODE"), 3);
    memset(changeValues, 0, sizeof(changeValues));
    initPage = false;
  }
  
  if(btnOk.Pressed())
  {
    FlashPointer();

    switch (cursorPos)
    {
      case 0: changeValues[0] = !changeValues[0]; edditing = !edditing; break;
      case 1: changeValues[1] = !changeValues[1]; edditing = !edditing; break;
      case 2: currPage = MENU_MODE; sets_Save(); initPage = true; return;
    }
  }

       if(changeValues[0]){*&settings.systemMode = MODE_MANUAL; changeValues [0] = false; updateItemValue = true; }
  else if(changeValues[1]) incrementDecrementDouble(&settings.manualPower, 1.0, 0.0, heaterRatedPower_W);
  else 
    doPointerNavigation();

  if(menuItemPrintable(1,1)){display1.print(F("Activate               "));}
  if(menuItemPrintable(1,2)){display1.print(F("Power                  "));}
  if(menuItemPrintable(1,3)){display1.print(F("Back                   "));}

  if(menuItemPrintable(10,1)){printOnOff(settings.systemMode == MODE_MANUAL);}
  if(menuItemPrintable(10,2)){printDoubleAtWidth(settings.manualPower, 4, "W", 1);}

}

void page_MENU_MISC(){//=================================================MISC==========================================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("MISC"), 11);
    memset(changeValues, 0, sizeof(changeValues));
    initPage = false;
  }

  if(btnOk.Pressed())
  {
    FlashPointer();
    changeValues[cursorPos] = !changeValues[cursorPos];
    edditing = !edditing;
  }

       if(changeValues[0]){*&settings.power = !*&settings.power; changeValues [0] = false; updateItemValue = true; }
  else if(changeValues[1] && waterDetected){*&settings.pump = !*&settings.pump; changeValues [1] = false; updateItemValue = true; }
  else if(changeValues[2])incrementDecrementDouble(&settings.filter_adc1, 0.01f, 0.0f, 0.99f);
  else if(changeValues[3])incrementDecrementDouble(&settings.filterDC, 0.01f, 0.0f, 0.99f);
  else if(changeValues[4])incrementDecrementInt(&settings.maxMashTemp, 1, 5, 110);
  else if(changeValues[5])incrementDecrementInt(&settings.timeBeforeDisable, 1, 1, 90);
  else if(changeValues[6])incrementDecrementInt(&settings.alarmVolume, 1, 0, 100);
  else if(changeValues[7])incrementDecrementInt(&settings.alarmTime, 1, 1, 30);
  else if(changeValues[8]){currPage = MENU_BREWING_LOCATION; changeValues[8] = false; initPage = true; return;}
  else if(changeValues[10]){currPage = MENU_ROOT; sets_Save(); initPage = true; changeValues[10] = false; return;}
  else 
    doPointerNavigation(); 

  if(!(updateAllItems | updateItemValue)) return;
   
  display1.drawVLine(11 * CHAR_X, 0, 64);

  for(uint8_t i = 1; i <= 11; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("Power             ")); break;  
        case 2: display1.print(F("Pump              ")); break;
        case 3: display1.print(F("Filter Temp       ")); break;
        case 4: display1.print(F("Filter DC         ")); break;
        case 5: display1.print(F("Max Temp          ")); break;
        case 6: display1.print(F("Disable disp      ")); break;
        case 7: display1.print(F("Alarm Volume      ")); break;
        case 8: display1.print(F("Alarm Time        ")); break;
        case 9: display1.print(F("Location          ")); break;
        case 10: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 11: display1.print(F("Back              ")); break;
      }
    }
    if(menuItemPrintable(12, i))
    {
      switch(i)
      {
        case 1: printOnOff(settings.power); break;
        case 2: printOnOff(settings.pump); break;
        case 3: printDoubleAtWidth(settings.filter_adc1, 3, " ", 2); break;
        case 4: printDoubleAtWidth(settings.filterDC, 3, " ", 2); break;
        case 5: printInt32_tAtWidth(settings.maxMashTemp, 3, "C"); break;
        case 6: printInt32_tAtWidth(settings.timeBeforeDisable, 3, "m"); break;
        case 7: printInt32_tAtWidth(settings.alarmVolume, 3, "%"); break;
        case 8: printInt32_tAtWidth(settings.alarmTime, 3, "s"); break;
        case 9: printStringAtWidth(brewingLocationToString(settings.brewingLocation), 4); break;
      }
    }
  }
}

void page_MENU_BREWING_LOCATION(){//=================================================BREWING_LOCATION============================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("BREWING LOCATION"), 3);
    initPage = false;
  }

  if(btnOk.Pressed())
  {
    FlashPointer();
    
    switch (cursorPos)
    {
      case 0: settings.brewingLocation = OutDoor; currPage = MENU_MISC; initPage = true; return;
      case 1: settings.brewingLocation = InDoor;  currPage = MENU_MISC; initPage = true; return;
      case 2: currPage = MENU_MISC; sets_Save(); initPage = true; return;
    }
  }

  doPointerNavigation(); 

  if(menuItemPrintable(1,1)){display1.print(F("Outdoor             "));}
  if(menuItemPrintable(1,2)){display1.print(F("Indoor              "));}
  if(menuItemPrintable(1,3)){display1.print(F("Back                "));}
}

void page_MENU_MASH_PROGRAM(){//=================================================MASH_PROGRAM====================================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("Mash Program"), 11);
    memset(changeValues, 0, sizeof(changeValues));
    initPage = false;
  }

  if(btnOk.Pressed())
  {
    FlashPointer();
    changeValues[cursorPos] = !changeValues[cursorPos];
    edditing = !edditing;
  }

  if(changeValues[0])
  {
    if(settings.startMashProgram)
    {
      // Already running — stop immediately
      settings.startMashProgram = false;
      pendingMashStart = false;
      settings.startLauteringProgram = false;
      settings.startHopProgram = false;
      settings.startWhirlpoolProgram = false;
      settings.pump = false;
    }
    else
    {
      // Arm/disarm
      pendingMashStart = !pendingMashStart;
      pendingLauterStart = false;
      pendingHopStart = false;
      pendingWhirlpoolStart = false;
      settings.startLauteringProgram = false;
      settings.startHopProgram = false;
      settings.startWhirlpoolProgram = false;
      if(waterDetected) settings.pump = true;
      if(pendingMashStart) settings.targetTemp = settings.mashTemps[0];
    }
    changeValues [0] = false;
    updateItemValue = true; 
  } 
  else if(changeValues[3])incrementDecrementInt(&settings.mashTemps[0], 1, 15, settings.maxMashTemp);
  else if(changeValues[4])incrementDecrementInt(&settings.mashTemps[1], 1, 15, settings.maxMashTemp);
  else if(changeValues[5])incrementDecrementInt(&settings.mashTemps[2], 1, 15, settings.maxMashTemp);
  else if(changeValues[6])incrementDecrementInt(&settings.mashTimes[0], 1, 0, 90);
  else if(changeValues[7])incrementDecrementInt(&settings.mashTimes[1], 1, 0, 90);
  else if(changeValues[8])incrementDecrementInt(&settings.mashTimes[2], 1, 0, 90);
  else if(changeValues[10])
  {
    currPage = MENU_ROOT; 
    sets_Save(); 
    initPage = true; 
    changeValues [10] = false;
    updateItemValue = true; 
  }
  else 
    doPointerNavigation();

  static uint32_t lastUpdate = 0;

  if (passedTimeS_mashProgram - lastUpdate >= 6)
  {
      updateAllItems = true;
      lastUpdate = passedTimeS_mashProgram;
  }

  if(!(updateAllItems || updateItemValue)) return;

  display1.drawVLine(8 * CHAR_X, 0, 64);

  for(uint8_t i = 1; i <= 11; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("Timer         ")); break;
        case 2: printDoubleAtWidth(passedTimeS_mashProgram/60.0, 3, "m");  break;
        case 3: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 4: display1.print(F("Temp 1               ")); break;
        case 5: display1.print(F("Temp 2               ")); break;
        case 6: display1.print(F("Temp 3               ")); break;
        case 7: display1.print(F("Time 1               ")); break;
        case 8: display1.print(F("Time 2               ")); break;
        case 9: display1.print(F("Time 3               ")); break;
        case 10: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 11: display1.print(F("Back                 ")); break;
      }
    }
    if(menuItemPrintable(9, i))
    {
      switch(i)
      {
        case 1:      if(settings.startMashProgram) printOnOff(true);
                else if(pendingMashStart) display1.print(F("ARM "));
                else printOnOff(false); break;
        case 2: printInt32_tAtWidth(settings.targetTemp, 3, "C"); break;

        case 4: printInt32_tAtWidth(settings.mashTemps[0], 3, "C"); break;
        case 5: printInt32_tAtWidth(settings.mashTemps[1], 3, "C"); break;  
        case 6: printInt32_tAtWidth(settings.mashTemps[2], 3, "C"); break;

      }
    }

    if(menuItemPrintable(12, i))
    {
      switch(i)
      {
        case 4: printInt32_tAtWidth(settings.mashTimes[0], 3, "m"); break;
        case 5: printInt32_tAtWidth(settings.mashTimes[1], 3, "m"); break;
        case 6: printInt32_tAtWidth(settings.mashTimes[2], 3, "m"); break;
      }
    }
  }
}

void page_MENU_LAUTERING_PROGRAM(){//=================================================LAUTERING_PROGRAM====================================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("Lautering Program"), 7);
    memset(changeValues, 0, sizeof(changeValues));
    initPage = false;
  }
 
  if(btnOk.Pressed())
  {
    FlashPointer();
    changeValues[cursorPos] = !changeValues[cursorPos];
    edditing = !edditing;
  }
 
  if(changeValues[0])
  {
    if(settings.startLauteringProgram)
    {
      // Already running — stop immediately
      settings.startLauteringProgram = false;
      pendingLauterStart = false;
      settings.pump = false;
    }
    else
    {
      // Arm/disarm
      pendingLauterStart = !pendingLauterStart;
      pendingMashStart = false;
      pendingHopStart = false;
      pendingWhirlpoolStart = false;
      settings.startMashProgram = false;
      settings.startHopProgram = false;
      settings.startWhirlpoolProgram = false;
      if(waterDetected) settings.pump = true;
      if(pendingLauterStart) settings.targetTemp = settings.lauteringTemp;
    }
    changeValues[0] = false;
    updateItemValue = true;
  }
 
  else if(changeValues[3])incrementDecrementInt(&settings.lauteringTime, 1, 1, 120);
  else if(changeValues[4]){incrementDecrementInt(&settings.lauteringTemp, 1, 15, settings.maxMashTemp); updateAllItems = true;}
  else if(changeValues[6])
  {
    currPage = MENU_ROOT; 
    sets_Save(); 
    initPage = true; 
    changeValues [6] = false;
    updateItemValue = true; 
  }
  else 
    doPointerNavigation();
 
  static uint32_t lastUpdate = 0;
 
  if (passedTimeS_lauteringProgram - lastUpdate >= 6)
  {
      updateAllItems = true;
      lastUpdate = passedTimeS_lauteringProgram;
  }
 
  if(!(updateAllItems || updateItemValue)) return; 
 
  display1.drawVLine(12 * CHAR_X, 0, 64);
 
  for(uint8_t i = 1; i <= 7; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("Timer             ")); break;
        case 2: printDoubleAtWidth(passedTimeS_lauteringProgram/60.0, 3, "m");  break;
        case 3: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 4: display1.print(F("Lautering Time          ")); break;
        case 5: display1.print(F("Lautering Temp          ")); break;
        case 6: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 7: display1.print(F("Back                    ")); break;
      }
    }
    if(menuItemPrintable(13, i))
    {
      switch(i)
      {
        case 1:
            if(settings.startLauteringProgram) printOnOff(true);
            else if(pendingLauterStart) display1.print(F("ARM "));
            else printOnOff(false);
            break;
        case 2: printInt32_tAtWidth(settings.targetTemp, 3, "C"); break;
        case 4: printInt32_tAtWidth(settings.lauteringTime, 3, "m"); break;
        case 5: printInt32_tAtWidth(settings.lauteringTemp, 3, "C"); break;
      }
    }
  }
}

void page_MENU_HOPS_PROGRAM(){//=================================================HOPS_PROGRAM====================================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("Hops Program"), 11);
    memset(changeValues, 0, sizeof(changeValues));
    initPage = false;
  }
  
  if(btnOk.Pressed())
  {
    FlashPointer();
    changeValues[cursorPos] = !changeValues[cursorPos];
    edditing = !edditing;
  }

  if(changeValues[0])
  {
    if(settings.startHopProgram)
    {
      settings.startHopProgram = false;
      pendingHopStart = false;
      settings.pump = false;
    }
    else
    {
      pendingHopStart = !pendingHopStart;
      pendingMashStart = false;
      pendingLauterStart = false;
      pendingWhirlpoolStart = false;
      settings.startMashProgram = false;
      settings.startLauteringProgram = false;
      settings.startWhirlpoolProgram = false;
      if(pendingHopStart) settings.targetTemp = settings.boilTemp;
    }
    changeValues[0] = false;
    updateItemValue = true;
  }

  else if(changeValues[3])incrementDecrementInt(&settings.boilTime, 1, 5, 120);
  else if(changeValues[4]){incrementDecrementInt(&settings.boilTemp, 1, 90, settings.maxMashTemp); updateAllItems = true;}
  else if(changeValues[6])incrementDecrementInt(&settings.hopTimes[0], 1, 0, 90);
  else if(changeValues[7])incrementDecrementInt(&settings.hopTimes[1], 1, 0, 90);
  else if(changeValues[8])incrementDecrementInt(&settings.hopTimes[2], 1, 0, 90);
  else if(changeValues[10])
  {
    currPage = MENU_ROOT; 
    sets_Save(); 
    initPage = true; 
    changeValues [10] = false;
    updateItemValue = true; 
  }
  else 
    doPointerNavigation();

  static uint32_t lastUpdate = 0;

  if (passedTimeS_hopProgram - lastUpdate >= 6)
  {
      updateAllItems = true;
      lastUpdate = passedTimeS_hopProgram;
  }

  if(!(updateAllItems || updateItemValue)) return; 

  display1.drawVLine(10 * CHAR_X, 0, 64);

  for(uint8_t i = 1; i <= 11; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("Timer             ")); break;
        case 2: printDoubleAtWidth(passedTimeS_hopProgram/60.0, 3, "m");  break;
        case 3: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 4: display1.print(F("Boil Time               ")); break;
        case 5: display1.print(F("Boil Temp               ")); break;
        case 6: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 7: display1.print(F("Hop Time 1              ")); break;
        case 8: display1.print(F("Hop Time 2              ")); break;
        case 9: display1.print(F("Hop Time 3              ")); break;
        case 10: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 11: display1.print(F("Back                    ")); break;
      }
    }
    if(menuItemPrintable(12, i))
    {
      switch(i)
      {
        case 1:
          if(settings.startHopProgram) printOnOff(true);
          else if(pendingHopStart) display1.print(F("ARM "));
          else printOnOff(false);
          break;
        case 2: printInt32_tAtWidth(settings.targetTemp, 3, "C"); break;
        case 4: printInt32_tAtWidth(settings.boilTime, 3, "m"); break;
        case 5: printInt32_tAtWidth(settings.boilTemp, 3, "C"); break;
        case 7: printInt32_tAtWidth(settings.hopTimes[0], 3, "m"); break;
        case 8: printInt32_tAtWidth(settings.hopTimes[1], 3, "m"); break;
        case 9: printInt32_tAtWidth(settings.hopTimes[2], 3, "m"); break;
      }
    }
  }
}

void page_MENU_WHIRLPOOL_PROGRAM()//=================================================WHIRLPOOL_PROGRAM====================================================
{
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("Whirlpool Program"), 7);
    memset(changeValues, 0, sizeof(changeValues));
    initPage = false;
  }
  
  if(btnOk.Pressed())
  {
    FlashPointer();
    changeValues[cursorPos] = !changeValues[cursorPos];
    edditing = !edditing;
  }

  if(changeValues[0])
  {
    if(settings.startWhirlpoolProgram)
    {
      settings.startWhirlpoolProgram = false;
      pendingWhirlpoolStart = false;
      settings.pump = false;
    }
    else
    {
      pendingWhirlpoolStart = !pendingWhirlpoolStart;
      pendingMashStart = false;
      pendingLauterStart = false;
      pendingHopStart = false;
      settings.startMashProgram = false;
      settings.startLauteringProgram = false;
      settings.startHopProgram = false;
      if(pendingWhirlpoolStart) settings.targetTemp = settings.whirlpoolTemp;
    }
    changeValues[0] = false;
    updateItemValue = true;
  }

  else if(changeValues[3])incrementDecrementInt(&settings.whirlpoolTime, 1, 1, 120);
  else if(changeValues[4]){incrementDecrementInt(&settings.whirlpoolTemp, 1, 60, settings.maxMashTemp); updateAllItems = true;}
  else if(changeValues[6])
  {
    currPage = MENU_ROOT; 
    sets_Save(); 
    initPage = true; 
    changeValues [6] = false;
    updateItemValue = true; 
  }
  else 
    doPointerNavigation();

  static uint32_t lastUpdate = 0;

  if (passedTimeS_whirlpoolProgram - lastUpdate >= 6)
  {
      updateAllItems = true;
      lastUpdate = passedTimeS_whirlpoolProgram;
  }

  if(!(updateAllItems || updateItemValue)) return; 

  display1.drawVLine(12 * CHAR_X, 0, 64);

  for(uint8_t i = 1; i <= 7; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("Timer             ")); break;
        case 2: printDoubleAtWidth(passedTimeS_whirlpoolProgram/60.0, 3, "m");  break;
        case 3: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 4: display1.print(F("Whirlpool Time               ")); break;
        case 5: display1.print(F("Whirlpool Temp               ")); break;
        case 6: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 7: display1.print(F("Back                    ")); break;
      }
    }
    if(menuItemPrintable(13, i))
    {
      switch(i)
      {
        case 1:
            if(settings.startWhirlpoolProgram) printOnOff(true);
            else if(pendingWhirlpoolStart) display1.print(F("ARM "));
            else printOnOff(false);
            break;
        case 2: printInt32_tAtWidth(settings.targetTemp, 3, "C"); break;
        case 4: printInt32_tAtWidth(settings.whirlpoolTime, 3, "m"); break;
        case 5: printInt32_tAtWidth(settings.whirlpoolTemp, 3, "C"); break;
      }
    }
  }
}

void initMenuPage(String title, uint8_t itemCount){//=======================TOOLS - menu Internals==================================================
  display1.clearBuffer();
  uint8_t fillCnt = (DISP_CHAR_WIDTH - title.length()) / 2;
  display1.setCursor(CHAR_X * fillCnt, 0);
  display1.print(title);
  printPointer();
 
  itemCnt = itemCount;
  flashCntr = 0;
  flashIsOn = false;
  updateAllItems = true;
}

void renderCurrentPage()
{
  switch (currPage)
  {
    case MENU_ROOT: page_MenuRoot(); break;
    case MENU_MODE: page_MENU_MODE(); break;
    case MENU_CONTROL_MODE_LQG: page_MENU_CONTROL_MODE_LQG(); break;
    case MENU_CONTROL_MODE_PI: page_MENU_CONTROL_MODE_PI(); break;
    case MENU_IDENT_MODE: page_MENU_IDENT_MODE(); break;
    case MENU_IDENT_TYPE: page_MENU_IDENT_TYPE(); break;
    case MENU_IDENT_SINE: page_MENU_IDENT_SINE(); break;
    case MENU_IDENT_MULTI_SINE: page_MENU_IDENT_MULTI_SINE(); break;
    case MENU_IDENT_STEP_RESPONSE: page_MENU_IDENT_STEP_RESPONSE(); break;
    case MENU_MANUAL_MODE: page_MENU_MANUAL_MODE(); break;
    case MENU_MISC: page_MENU_MISC(); break;
    case MENU_BREWING_LOCATION: page_MENU_BREWING_LOCATION(); break;
    case MENU_MASH_PROGRAM: page_MENU_MASH_PROGRAM(); break;
    case MENU_LAUTERING_PROGRAM: page_MENU_LAUTERING_PROGRAM(); break;
    case MENU_HOPS_PROGRAM: page_MENU_HOPS_PROGRAM(); break;
    case MENU_WHIRLPOOL_PROGRAM: page_MENU_WHIRLPOOL_PROGRAM(); break;
  }
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
      } else if (cursorPos >= dispOffset + DISP_ITEM_ROWS) {
        dispOffset = cursorPos - DISP_ITEM_ROWS + 1; 
      }
      updateAllItems = true; // Force update of all items
      // Navigation happens after loop() already checked updateAllItems,
      // so clear the old framebuffer here before the page redraws.
      display1.clearBuffer();
      // printPointer();  // Only redraw when view actually changes
    }
    timeLastTouched = millis()/1000.0/60.0; // Update last touched time
    
    // Serial.print(", Direction: ");
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

        updateAllItems = true; //updateItemValue = true; changed
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

        updateAllItems = true; //updateItemValue = true; changed
        timeLastTouched = millis()/1000.0/60.0;
    }

    delayMicroseconds(5);
}
void incrementDecrementDouble(double *v, double amount, double min, double max)
{
  double direction = encoder.getRPM();

    if (direction != 0) {
      
      double target;
      double newValue;
      int threshold = 4;

      if(abs(direction) <= threshold) //slow rotation
      {
        target = direction * amount;
        newValue = *v + target;
      }
      else if(abs(direction) > threshold) //fast rotation
      {
        target = direction * amount;
        newValue = *v + target;
        newValue = roundTo(newValue, -log10(amount));
      }
      if (newValue >= min && newValue <= max) 
        *v = newValue;
      else if (newValue < min) 
        *v = min;
      else 
        *v = max;

      updateAllItems = true; //updateItemValue = true; changed
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
  if(!(updateAllItems || (updateItemValue && (cursorPos + 1) == yPos))){return false;}
  uint8_t yMaxOffset = 0;
  if(yPos > DISP_ITEM_ROWS) {yMaxOffset = yPos - DISP_ITEM_ROWS;}
  if(dispOffset <= (yPos) && dispOffset >= yMaxOffset){display1.setCursor(CHAR_X*xPos, CHAR_Y*(yPos - dispOffset)); return true;}
  return false;
}

bool menuItemPrintableDisp2(uint8_t xPos, uint8_t yPos){ 
  if(!(updateAllItems || (updateItemValue && (cursorPos + 1) == yPos))){return false;}
  uint8_t yMaxOffset = 0;
  if(yPos > DISP2_ITEM_ROWS) {yMaxOffset = yPos - DISP2_ITEM_ROWS;}
  if(0 <= (yPos) && 0 >= yMaxOffset){display2.setCursor(DISP2_CHAR_X*xPos, DISP2_CHAR_Y*(yPos)); return true;}
  return false;
}

//======================================================TOOLS_display========================================================
void printPointer(){
  //Serial.println("printPointer");
  for(uint8_t i=1; i<=DISP_ITEM_ROWS; i++) display1.drawStr(0, i*CHAR_Y, " ");
  display1.drawStr(0, (cursorPos - dispOffset + 1)*CHAR_Y, "*");
  updateAllItems = true;
  // display1.sendBuffer();
}
void FlashPointer(){
  timeLastTouched = millis()/1000.0/60.0;
  for(uint8_t i=1; i<=DISP_ITEM_ROWS; i++) display1.drawStr(0, i*CHAR_Y, " ");
  updateAllItems = true;

  delay(100);
  //Serial.println("FlashPointer");
  display1.drawStr(0, (cursorPos - dispOffset + 1)*CHAR_Y, "*");
  updateAllItems = true;
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
void printInt32_tAtWidth(int32_t value, uint8_t width, const char* c){
  display1.print(value);
  display1.print(c);
  printChars(width-getInt32_tCharCnt(value), ' ');
}
void printDoubleAtWidth(double value, uint8_t width, const char* c, uint8_t decimals){
  char buf[10];
  dtostrf(value, width-getDoubleCharCnt(value), decimals, buf); // 1 decimal
  display1.print(buf);
  display1.print(c);
}

void printStringAtWidth(const char* str, uint8_t width)
{
  uint8_t len = strlen(str);

  display1.print(str);

  if(len < width)
  {
    for(uint8_t i = 0; i < (width - len); i++)
      display1.print(' ');
  }
}

const char* brewingLocationToString(BrewingLocation location)
{
  switch(location)
  {
    case OutDoor: return "OUT";
    case InDoor:  return "IN";
    default:       return "null";
  }
}

const char* identificationTypeToString(IdentificationType type)
{
  switch(type)
  {
    case IDENT_STEP_RESPONSE: return "STEP";
    case IDENT_SINE:          return "SINE";
    case IDENT_MULTI_SINE:    return "MULTISINE";
    default:                  return "UNKNOWN";
  }
}

const char* systemModeToString(SystemMode mode)
{
  switch(mode)
  {
    case MODE_CONTROL_LQG:    return "LQG CONTROL";
    case MODE_CONTROL_PI:    return "PI CONTROL";
    case MODE_IDENTIFICATION: return "IDENT";
    case MODE_MANUAL:         return "MANUAL";
    default:                  return "UNKNOWN";
  }
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

void updateModelForCurrentTemp(LQGController& controller, double T, ModelID& activeModel)
{
    // --------- DETERMINE DESIRED MODEL ----------
    ModelID desiredModel = activeModel;  // stay by default
    const float H = 2.0;
    const float boilTransition = settings.boilTemp - settings.deadband1 - H; //100 - 8 - 1 = 91, 93.5 - 8 - 1 = 84.5

   switch (activeModel)
    {
        case MODEL_50:
            if (T > 56 + H)
                desiredModel = MODEL_65;
            break;

        case MODEL_65:
            if (T > 69 + H)
                desiredModel = MODEL_75;
            else if (T < 56 - H)
                desiredModel = MODEL_50;
            break;

        case MODEL_75:
            if (T > boilTransition + H && settings.targetTemp >= 93.5)
                desiredModel = MODEL_Boil;
            else if (T < 69 - H)
                desiredModel = MODEL_65;
            break;

        case MODEL_Boil:
            if (T < boilTransition - H || settings.targetTemp < 93.5)
                desiredModel = MODEL_75;
            break;

        default:
            break;
    }

    // --------- SWITCH ONLY ON TRANSITION ----------
    if (desiredModel != activeModel) {

    // =========================================================
    // 🔥 ENTERING BOIL
    // =========================================================
    if (desiredModel == MODEL_Boil)
    {
        double Q_current = controller.GetCurrentPower_W();

        boilCtrl.setInputs(T, *ambientTempPtr, settings.boilTemp); 
        boilCtrl.setInitialPower(Q_current);
    }

    // =========================================================
    // 🔧 NORMAL LQG MODELS (50/65/75)
    // =========================================================
    else
    {
        double u_prev;

        // 👉 if coming FROM boil, use boil output
        if (activeModel == MODEL_Boil)
        {
            u_prev = boilCtrl.GetCurrentPower_W();  
        }
        else
        {
            u_prev = controller.GetCurrentPower_W();
        }

        switch (desiredModel)
        {
            case MODEL_50: controller.loadModel(model50); break;
            case MODEL_65: controller.loadModel(model65); break;
            case MODEL_75: controller.loadModel(model75); break;
            default: break;
        }

        controller.setControlOutput(u_prev);
        controller.resetIntegrator();  // ← add this
    }

    activeModel = desiredModel;
    mashCtrlLQG.SetActiveModel(activeModel);
  }
}

void resetAllAlarmsOnBoot()
{
  client.publish("mashTun/lautering_alarm", "0", true);
  client.publish("mashTun/boil_alarm", "0", true);
  client.publish("mashTun/hops_alarm", "0", true);
  client.publish("mashTun/whirlpool_alarm", "0", true);
}

void updateSettings()
{
  /* Krav
  I: Elementet får aldrig vara på om det inte finns vatten i bryggverket
  II: Pumpen får aldrig vara igång om det inte finns vatten i bryggverket
  */ 
  constexpr float MARGIN = 0.03205f; // margin for temperature comparison (3.2%) aproximate to 2.5C

  if(!waterDetected){pumpWater = false;} else {pumpWater = true;}

  // --- Pending mash start: auto-trigger when setpoint reached ---
  if(pendingMashStart && current_mashTemp >= settings.mashTemps[0] * (1.0f - MARGIN))
  {
    pendingMashStart = false;
    settings.startMashProgram = true;
    settings.startLauteringProgram = false;
    settings.startHopProgram = false;
    settings.startWhirlpoolProgram = false;
    if(waterDetected) settings.pump = true;
    updateAllItems = true;
  }

  if(pendingLauterStart && current_mashTemp >= settings.lauteringTemp * (1.0f - MARGIN))
  {
    pendingLauterStart = false;
    settings.startMashProgram = false;
    settings.startLauteringProgram = true;
    settings.startHopProgram = false;
    settings.startWhirlpoolProgram = false;
    if(waterDetected) settings.pump = true;
    updateAllItems = true;
  }

  if(pendingHopStart && current_mashTemp >= settings.boilTemp * (1.0f - MARGIN))
  {
    pendingHopStart = false;
    settings.targetTemp = settings.boilTemp;   // <-- ADD THIS
    settings.startMashProgram = false;
    settings.startLauteringProgram = false;
    settings.startHopProgram = true;
    settings.startWhirlpoolProgram = false;
    settings.pump = false; // hops kör utan pump
    updateAllItems = true;
  }

  if(current_mashTemp >= 81 && !tooHotForPump) // i dont want the pump to go whne its over 80 degrees, it damages the pump 
  {
    tooHotForPump = true;
    updateAllItems = true;
  }
  else if(current_mashTemp <= 79 && tooHotForPump)
  {
    tooHotForPump = false;
    updateAllItems = true;
  }

  if(pendingWhirlpoolStart && current_mashTemp <= settings.whirlpoolTemp * (1.0f + MARGIN)  && current_mashTemp >= settings.whirlpoolTemp * (1.0f - MARGIN))
  {
    pendingWhirlpoolStart = false;
    settings.startMashProgram = false;
    settings.startLauteringProgram = false;
    settings.startHopProgram = false;
    settings.startWhirlpoolProgram = true;
    settings.pump = true;
    updateAllItems = true;
  }
  
  if(!settings.power){pumpWater = false;}

  // --- Pump ---
  // Serial.print("settings.pump: ");
  // Serial.print(settings.pump);
  // Serial.print("pumpWater: ");
  // Serial.println(pumpWater ? "YES" : "NO");
  
  digitalWrite(pumpPin, settings.pump && pumpWater && !tooHotForPump ? HIGH : LOW);

  if(settings.power)
  {
    digitalWrite(ledOnPin, HIGH);
    digitalWrite(ledWaterDetectedPin, !waterDetected);

    bool startMashProgram_ = settings.startMashProgram;
    bool startLauteringProgram_ = settings.startLauteringProgram;
    bool startHopProgram_ = settings.startHopProgram;
    bool startWhirlpoolProgram_ = settings.startWhirlpoolProgram;

    // Reset tS när ett program startar eller byts
    bool programChanged = (startMashProgram_ != prevStartMashProgram_) || 
                          (startLauteringProgram_ != prevStartLauteringProgram_) ||
                          (startHopProgram_ != prevStartHopProgram_) ||
                          (startWhirlpoolProgram_ != prevStartWhirlpoolProgram_);

    if(programChanged)
      updateAllItems = true; // Force update of all items when program starts/stops or changes

    // New mash program
    if (startMashProgram_ && !prevStartMashProgram_)
    {
      currentMashStep = 0;
      mashStepAtTemp = false;
      mashStepTimerStartMs = 0;
      mashCompletedTimeMs = 0;
      passedTimeS_mashProgram = 0;
      settings.targetTemp = settings.mashTemps[0];
    }

    if (programChanged || (!startMashProgram_ && !startLauteringProgram_ && !startHopProgram_ && !startWhirlpoolProgram_)) {
      tS = millis();
    }

    // =====================================================
    // MASH PROGRAM
    // =====================================================
    if (startMashProgram_)
    {
      settings.targetTemp = settings.mashTemps[currentMashStep];

      // Wait for current step temperature before starting its timer
      if (!mashStepAtTemp && current_mashTemp >= settings.mashTemps[currentMashStep] * (1.0f - MARGIN))
      {
        mashStepAtTemp = true;
        mashStepTimerStartMs = millis();
      }

      unsigned long stepElapsedMs = mashStepAtTemp ? millis() - mashStepTimerStartMs : 0;
      unsigned long stepDurationMs = (unsigned long)settings.mashTimes[currentMashStep] * 60000UL;

      // Current mash step finished
      if (mashStepAtTemp && stepElapsedMs >= stepDurationMs)
      {
        mashCompletedTimeMs += stepDurationMs;

        if (currentMashStep < 2)
        {
          currentMashStep++;
          settings.targetTemp = settings.mashTemps[currentMashStep];
          mashStepAtTemp = false;
          mashStepTimerStartMs = 0;
          stepElapsedMs = 0;
          updateAllItems = true;
        }
        else
        {
          // Final mash step finished
          mashStepAtTemp = false;
          mashStepTimerStartMs = 0;
          stepElapsedMs = 0;

          updateAllItems = true;
        }
      }

      // Cumulative mash time; heating between steps is not counted
      if (stepElapsedMs > stepDurationMs) stepElapsedMs = stepDurationMs;
          passedTimeS_mashProgram = (mashCompletedTimeMs + stepElapsedMs) / 1000UL;
    }
    else
    {
      passedTimeS_mashProgram = 0;
    }


    // =====================================================
    // OTHER PROGRAM TIMERS
    // =====================================================
    passedTimeS_lauteringProgram = startLauteringProgram_ ? (millis() - tS) * 0.001f : 0;
    passedTimeS_hopProgram       = startHopProgram_       ? (millis() - tS) * 0.001f : 0;
    passedTimeS_whirlpoolProgram = startWhirlpoolProgram_ ? (millis() - tS) * 0.001f : 0;


    // Store states for next loop
    prevStartMashProgram_      = startMashProgram_;
    prevStartLauteringProgram_ = startLauteringProgram_;
    prevStartHopProgram_       = startHopProgram_;
    prevStartWhirlpoolProgram_ = startWhirlpoolProgram_;

    // Time to lauter notification
    if(startMashProgram_ && passedTimeS_mashProgram / 60.0 >= settings.mashTimes[0] + settings.mashTimes[1] + settings.mashTimes[2])
    {
      settings.targetTemp = settings.lauteringTemp;
      settings.pump = true;

      if(!mashFinishedNotified)
      {
        mashFinishedNotified = true;
        lastMashReminderTime = millis();
        client.publish("mashTun/lautering_alarm", "1", true); // mäskningen är färdig, nu är det dags att laka
      }
      else if(millis() - lastMashReminderTime >= REMINDER_INTERVAL_MS)
      {
        lastMashReminderTime = millis();
        client.publish("mashTun/lautering_reminder", "1", false); //  mäskningen är färdig, för helvete
      }
    }
    else
    {
      if(mashFinishedNotified)
      {
        client.publish("mashTun/lautering_alarm", "0", true); // reset — condition no longer true
      }
      mashFinishedNotified = false;
    }

    // Time to boil notification
    if(startLauteringProgram_ && passedTimeS_lauteringProgram / 60.0 >= settings.lauteringTime)
    {
      settings.targetTemp = settings.boilTemp;
      settings.pump = false;

      if(!lauteringFinishedNotified)
      {
        lauteringFinishedNotified = true;
        lastLauteringReminderTime = millis();
        client.publish("mashTun/boil_alarm", "1", true); // lakningen är färdig, nu är det dags att koka
      }
      else if(millis() - lastLauteringReminderTime >= REMINDER_INTERVAL_MS)
      {
        lastLauteringReminderTime = millis();
        client.publish("mashTun/boil_reminder", "1", false); // lakningen är färdig, för helvete
      }
    }
    else
    {
      if(lauteringFinishedNotified)
      {
        client.publish("mashTun/boil_alarm", "0", true); // reset — condition no longer true
      }
      lauteringFinishedNotified = false;
    }

    float m = passedTimeS_hopProgram / 60.0f;
    float d = settings.alarmTime / 60.0f;

    settings.hopIndex = -1;

    // Time to hop notification
    if (startHopProgram_) {
        for (int i = 0; i < 3; i++) {
            if (settings.hopTimes[i] == 0) continue; // hop addition disabled, no alarm
            float t = settings.boilTime - settings.hopTimes[i];

            if (m >= t && m < t + d) {
                settings.hopIndex = i + 1;

                if (!hopFinishedNotified[i]) {
                    hopFinishedNotified[i] = true;
                    char payload[2];
                    snprintf(payload, sizeof(payload), "%d", settings.hopIndex);
                    client.publish("mashTun/hops_alarm", payload, true); // Nu är det dags att lägga i humlen
                }
                break;
            } else {
                  if (hopFinishedNotified[i]) 
                  {
                    client.publish("mashTun/hops_alarm", "0", true);
                  }
                  hopFinishedNotified[i] = false;
            }
        }
    } 
    else {
        bool anyWasSet = false;
        for (int i = 0; i < 3; i++) if (hopFinishedNotified[i]) anyWasSet = true;
        if (anyWasSet) client.publish("mashTun/hops_alarm", "0", true);
        memset(hopFinishedNotified, false, sizeof(hopFinishedNotified));
    }

    // Time to whirlpool notification
    if (startHopProgram_)
    {
      if(passedTimeS_hopProgram / 60.0 >= settings.boilTime)
      {
        settings.targetTemp = settings.whirlpoolTemp;
        settings.pump = true;
        if(settings.whirlpoolTime != 0)
        {
          if(!whirlpoolFinishedNotified)
          {
            whirlpoolFinishedNotified = true;
            lastWhirlpoolReminderTime = millis();
            client.publish("mashTun/whirlpool_alarm", "1", true); // Nu är kokningen färdig, nu är det dags att virvla
          }
          else if(millis() - lastWhirlpoolReminderTime >= REMINDER_INTERVAL_MS)
          {
            lastWhirlpoolReminderTime = millis();
            client.publish("mashTun/whirlpool_reminder", "1", false); // kokningen är färdig, för helvete
          }
        }
      } 
      else
      {
        settings.targetTemp = settings.boilTemp;
        settings.pump = false;
      }
    }

    else
    {
      if(whirlpoolFinishedNotified)
      {
        client.publish("mashTun/whirlpool_alarm", "0", true); // reset — condition no longer true
      }
      
      whirlpoolFinishedNotified = false;
    }
    
   

    if (startWhirlpoolProgram_)
    {
        if (passedTimeS_whirlpoolProgram / 60.0 >= settings.whirlpoolTime)
        {
          settings.targetTemp = 15.0;
          settings.pump = false;
        }
         
        else
        { 
          settings.targetTemp = settings.whirlpoolTemp;
          settings.pump = true;
        }
    }

    analogWrite(alarmPin, settings.hopIndex != -1 ? settings.alarmVolume / 100.0f * 4095.0f : 0);

    // PID_mashTemp.SetTunings(settings.Kp_mash, settings.Ki_mash, 0, settings.POnE_mash ? P_ON_E : P_ON_M);
    // PID_mashTemp.SetMode(AUTOMATIC);
    // PID_mashTemp.Compute();
    // PID_mashTemp.SetHeatLossCoefficient(settings.k_heatLoss);

    // --- Retune controller ONLY if changed ---
    if (settings.Kp_mash != lastKp || settings.Ki_mash != lastKi 
    || settings.k_heatLoss != lastKHeat || settings.deadband1 != lastDeadband1 || settings.deadband2 != lastDeadband2
    || settings.gamma != lastGamma)
    {
      mashCtrl_PI.SetTunings(settings.Kp_mash, settings.Ki_mash, settings.k_heatLoss);
      mashCtrl_PI.SetCaptureBands(settings.deadband1, settings.deadband2, settings.gamma);
      double Q0 = settings.k_heatLoss * (current_mashTemp - current_airTemp);
      mashCtrl_PI.Reset(Q0);

      lastKp = settings.Kp_mash;
      lastKi = settings.Ki_mash;
      lastKHeat = settings.k_heatLoss;
      lastDeadband1 = settings.deadband1;
      lastDeadband2 = settings.deadband2;
      lastGamma = settings.gamma;

      //LQG Controller retune
      mashCtrlLQG.SetCaptureBands(settings.deadband1, settings.deadband2, settings.gamma);
    }

    static BrewingLocation lastLoc;

    if (settings.brewingLocation != lastLoc)
    {
      updateAmbientSource(settings.brewingLocation);
      lastLoc = settings.brewingLocation;
    }

    handleMQTT();
  }
  else
  {
    digitalWrite(ledOnPin, LOW);
    digitalWrite(ledWaterDetectedPin, 0);
  }

  // updateAllItems = true;
}

void updateAmbientSource(BrewingLocation loc)
{
  double* newPtr;

  if (loc == OutDoor)
      newPtr = &outdoorTemp;
  else
      newPtr = &indoorTemp;

  mashCtrl_PI.SetAmbientTemp(newPtr);
  mashCtrlLQG.SetAmbientTemp(newPtr);
  ambientTempPtr = newPtr;
  updateDisp2();
}

void updateSensorValues() {

  unsigned long currentTime = millis();
  if (currentTime - lastReadTime >= readInterval)
  {
    lastReadTime = currentTime;

    // PT100 Temperature reading and conversion
    rawTemp = PT100.temperature(RNOMINAL, RREF);
    // Apply two-point calibration for PT100
    double calibratedTemp = a * rawTemp + b;
    filteredTempPT100 = Filter(calibratedTemp, filteredTempPT100, settings.filter_adc1, 100.0f);
    tempPT100 = roundTo(filteredTempPT100, 3); // Round to 3 decimal places

    waterDetected = !digitalRead(waterDetectedPin);

    current_oledTemp = dallasTemp.getTempCByIndex(0);
    dallasTemp.requestTemperatures();

  //  Serial.printf(
  //   "tempPT100: %.3f C, kLoss_W_per_degC: %.3f, Active Model: %s\n",
  //   tempPT100,
  //   (activeModel == MODEL_Boil) ? modelBoil.kLoss_W_per_degC
  //                              : mashCtrlLQG.model_->kLoss_W_per_degC,
  //   activeModel == MODEL_50  ? "50C"  :
  //   activeModel == MODEL_65  ? "65C"  :
  //   activeModel == MODEL_75  ? "75C"  :
  //   activeModel == MODEL_Boil ? "BOIL" :
  //                               "UNKNOWN"
  //   );

    // Serial.print("Voltage adc1: ");
    // Serial.print(adc1_raw_volt, 4);
    // Serial.print(" V");

    // Serial.print(", Gain range: ±");
    // Serial.print(adc.getFsRange(), 2);
    // Serial.print(" V");

    // Serial.print(", Water Detected: ");
    // Serial.print(waterDetected ? "YES" : "NO");
    
    // Serial.print(", pumpWater: ");
    // Serial.println(pumpWater ? "YES" : "NO");
    
    // Serial.print(", Filtered ADC1: ");
    // Serial.print(filteredAdc1_volt, 4); 

    // Serial.print(", current_mashTemp: ");
    // Serial.print(current_mashTemp, 1);
    // Serial.println(" °C, ");

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
    current_mashTemp = filteredTempPT100; //Temp_C;
  }

}


void updateDisp2()
{
  #define DUTY_TO_PERCENT 0.02442002442f
  display2.clearBuffer();
  display2.setCursor(DISP2_CHAR_X * 1,  DISP2_CHAR_Y * 1); display2.print(F("TargetTemp:"));
  display2.setCursor(DISP2_CHAR_X * 1,  DISP2_CHAR_Y * 2); display2.print(F("MashTemp:  "));
  display2.setCursor(DISP2_CHAR_X * 1,  DISP2_CHAR_Y * 3); display2.print(F("DC:        "));
  display2.setCursor(DISP2_CHAR_X * 1,  DISP2_CHAR_Y * 4); display2.print(F("AirTemp:   "));

  display2.setCursor(DISP2_CHAR_X * 13, DISP2_CHAR_Y * 1); printInt32_tAtWidthDisplay2(settings.targetTemp, 3, 'C');
  display2.setCursor(DISP2_CHAR_X * 13, DISP2_CHAR_Y * 2); printDoubleAtWidthDisplay2(current_mashTemp, 3, 'C');
  display2.setCursor(DISP2_CHAR_X * 6, DISP2_CHAR_Y * 3); printDoubleAtWidthDisplay2(DutyCycle * DUTY_TO_PERCENT, 3, '%');
  display2.setCursor(DISP2_CHAR_X * 13, DISP2_CHAR_Y * 3); printDoubleAtWidthDisplay2(DutyCycle * PWM_to_W, 3, 'W');
  display2.setCursor(DISP2_CHAR_X * 13, DISP2_CHAR_Y * 4); printDoubleAtWidthDisplay2(*ambientTempPtr, 3, 'C');
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
  client.setBufferSize(512);      // 🔴 KRITISK FIX (266 byte payload)

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
      subscribeMQTT();
      publishWifiMqttStatus();
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }

    // digitalWrite(ledMqttPin, client.connected() ? LOW : HIGH);
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

  // if (client.connected()) {
  //   digitalWrite(ledMqttPin, LOW);
  //   return;
  // }

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
      
      subscribeMQTT();
      publishWifiMqttStatus();
    }
  }

  // digitalWrite(ledMqttPin, HIGH);
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

void handleMQTT()
{
  if (TopicArrived)
  {
    TopicArrived = false;

    if (mqtttopic == "home/weather/outdoorTemp_plus1h")
    {
      outdoorTemp = atof(mqttpayload);
      // Serial.print("Received outdoorTemp: ");
      // Serial.println(outdoorTemp);
    }
    else if (mqtttopic == "home/datalogger/airTemp")
    {
      indoorTemp = atof(mqttpayload);

      // Serial.print("Received indoorTemp: ");
      // Serial.println(indoorTemp);
    }
  }
}

const char* modelIDToString(ModelID id)
{
    switch (id)
    {
        case MODEL_50:   return "MODEL_50";
        case MODEL_65:   return "MODEL_65";
        case MODEL_75:   return "MODEL_75";
        case MODEL_Boil: return "MODEL_Boil";
        default:         return "UNKNOWN";
    }
}

void publishMessage() //Home Assistant only
{
  StaticJsonDocument<1024> doc;

  // --- LQGI Model debug  ---
  // doc["activeModel"]   = modelIDToString(activeModel);
  if (activeModel == MODEL_Boil)
  {
    doc["Debug_log"] = boilCtrl.GetDebugString();
  }
  else
  {
    doc["Debug_log"] = mashCtrlLQG.GetDebugString();
  }

  // --- Core readings ---
  doc["mashTempPT100"] = roundTo(current_mashTemp, 3);  // 3 decimal
  doc["airTemp"]       = roundTo(*ambientTempPtr, 2);   // 2 decimal
  doc["timePassed"] = roundTo(
      (
        settings.startLauteringProgram ? passedTimeS_lauteringProgram :
        settings.startMashProgram ? passedTimeS_mashProgram :
        settings.startHopProgram  ? passedTimeS_hopProgram  :
                                  0.0f) / 60.0f,1);
  // --- Setpoint ---
  doc["targetTemp"]    = settings.targetTemp;

  // --- OLED temperature display ---
  doc["displayTemp"]   = roundTo(current_oledTemp, 1);

  // --- Power / control ---
  doc["dutyCycle"]     = roundTo(DutyCycle / 4095.0f * 100.0f, 3);
  doc["powerIn"]       = roundTo(DutyCycle * PWM_to_W, 1); // integer watts

  // --- Boolean states (REAL booleans) ---
  doc["pumpOn"]        = (settings.pump && pumpWater && !tooHotForPump);
  // doc["hopsAlarm"]     = hopsAlarm;
  doc["waterDetected"] = waterDetected;

  doc["hopIndex"] = settings.hopIndex; // e.g. "hop1": true

  // --- PI parameters ---
  doc["kp"]            = roundTo(settings.Kp_mash, 1);
  doc["ki"]            = roundTo(settings.Ki_mash, 3);
  doc["heatLoss"]      = roundTo(settings.k_heatLoss, 1); //change to: model->kLoss_W_per_degC

  // --- Filters (rounded for readability) ---
  doc["filterTemp"]    = roundTo(settings.filter_adc1, 2);
  doc["filterDC"]      = roundTo(settings.filterDC, 2);

  // --- Serialize and publish ---
  char buffer[1024];
  size_t n = serializeJson(doc, buffer);
  client.publish("mashTun/state", buffer, n);

  //InfluxDB
  if (!writeInflux(current_mashTemp, DutyCycle * PWM_to_W, *ambientTempPtr)) {
  Serial.println("Influx write failed");
  }
}

void subscribeMQTT() //Home Assistant only
{
  if (client.connected())
  {
    // client.subscribe("home/weather/outdoorTemp");
    client.subscribe("home/weather/outdoorTemp_plus1h");
    client.subscribe("home/datalogger/airTemp");
  }
}

bool writeInflux(float mashTemp, float powerW, double ambientTemp) //Only InfluxDB
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
    ",power="   + String(powerW, 1) +
    ",airtemp=" + String(ambientTemp, 2) +
    ",targettemp=" + String(settings.targetTemp, 1) +
    " " + String((uint64_t)time(nullptr) * 1000000000ULL) + "\n";

  int code = http.POST(line);
  // Serial.print("Influx HTTP code: ");
  // Serial.println(code);
  http.end();

  return (code == 204);
}

void setupTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  time_t now;
  while ((now = time(nullptr)) < 100000) {
    delay(100);
  }
}

void callback(char* topic, byte* payload, unsigned int length) //Homeassistant
{
  if ( !TopicArrived )
  {
    memset( mqttpayload, '\0', mqttpayloadSize ); // clear payload char buffer
    mqtttopic = ""; //clear topic string buffer
    mqtttopic = topic; //store new topic
    memcpy( mqttpayload, payload, length );
    mqttpayload[length] = '\0';                // <-- CRITICAL FIX
    TopicArrived = true;
  }
}

double Filter(double New, double Current, double alpha, double maxValue)
{
  double diff = fabs(New - Current);
  // Normalize difference by max value for relative comparison
  double relativeDiff = diff / maxValue;
  // Smaller alpha (more responsive) when difference is large
  double adjustedAlpha = alpha / (1.0 + relativeDiff * 0.15);
  // Serial.printf(", adjAlpha: %.4f |\n", adjustedAlpha);
  return (1.0 - adjustedAlpha) * New + adjustedAlpha * Current;
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float roundTo(float value, int decimals) {
  float multiplier = powf(10.0f, decimals);
  return roundf(value * multiplier) / multiplier;
}
