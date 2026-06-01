#pragma once
#include <Arduino.h>
#include <mash_config.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <PI_Controller.h>
#include <PressButton.h>
#include <RotaryEncoderAccel.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Privates.h>
#include <ArduinoJson.h>
#include <math.h>
#include <Adafruit_MAX31865.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HTTPClient.h>
#include <time.h>

// -----------------------------------------------------------------------
// IO-pins
#define dutyCycleOutPin     26
#define ledOnPin            14
#define ledWaterDetectedPin  4
#define ledWifiPin          17
#define outputA             33
#define outputB             32
#define confirmBtnPin       25
#define MAX31865_CS          5
#define MAX31865_SO         19
#define MAX31865_SCK        18
#define MAX31865_SI         23
#define pumpPin             27
#define alarmPin            13
#define ONE_WIRE_BUS        16
#define waterDetectedPin    34

// -----------------------------------------------------------------------
// OLED display
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64

// -----------------------------------------------------------------------
// Settings constants
#define PACING_MC       30
#define FLASH_RST_CNT    3
#define SETTINGS_CHKVAL  3647
#define SHIFT_UP         0

// PT100 calibration
#define RREF      435.8
#define RNOMINAL  100.0

// -----------------------------------------------------------------------
// Enums

enum BrewingLocation {
    OutDoor,
    InDoor
};

enum pageType {
    MENU_ROOT,
    MENU_MODE,
    MENU_CONTROL_MODE_PI,
    MENU_MISC,
    MENU_BREWING_LOCATION,
};

// -----------------------------------------------------------------------
// Settings struct
#pragma pack(1)
struct Mysettings {
    double  Kp_lauter         = 32.0;
    double  Ki_lauter         = 0.00;
    double  k_heatLoss        = 4.9;
    double  deadband1         = 4;
    double  deadband2         = 0.5;
    double  gamma             = 0.5;
    double  targetTemp        = 25;
    BrewingLocation brewingLocation = OutDoor;
    int16_t timeBeforeDisable = 2;
    boolean power             = true;
    int16_t maxMashTemp       = 100;
    double  filter_adc1       = 0.90f;
    double  filterDC          = 0.90f;
    int16_t alarmVolume       = 4;
    int16_t alarmTime         = 3;
    uint16_t settingsCheckValue = SETTINGS_CHKVAL;
};

// -----------------------------------------------------------------------
// Global variable declarations (defined in main.cpp)

// Display sizing
extern uint8_t DISP_ITEM_ROWS;
extern uint8_t DISP_CHAR_WIDTH;
extern uint8_t CHAR_X;
extern uint8_t CHAR_Y;
extern uint8_t DISP2_ITEM_ROWS;
extern uint8_t DISP2_CHAR_WIDTH;
extern uint8_t DISP2_CHAR_X;
extern uint8_t DISP2_CHAR_Y;

// Settings
extern Mysettings settings;
extern Mysettings oldSettings;

// MQTT / WiFi
extern Privates      privates;
extern WiFiClient    espClient;
extern PubSubClient  client;
extern char          messages[50];
extern volatile bool TopicArrived;
extern char          mqttpayload[];
extern String        mqtttopic;

// Encoder / button
extern RotaryEncoderAccel encoder;
extern PressButton        btnOk;

// Menu state
extern pageType  currPage;
extern boolean   updateAllItems;
extern boolean   updateItemValue;
extern uint8_t   itemCnt;
extern int8_t    cursorPos;
extern uint8_t   saveCursorPos;
extern uint8_t   dispOffset;
extern uint8_t   saveDispOffset;
extern bool      edditing;
extern bool      detectedRotation;
extern uint8_t   root_pntrPos;
extern uint8_t   root_dispOffset;
extern uint8_t   flashCntr;
extern boolean   flashIsOn;
extern bool      changeValues[20];
extern bool      initPage;

// Timing
extern unsigned long previousTime;
extern unsigned long currentTime;
extern double        tS;
extern unsigned long previousSensorRead;
extern unsigned long lastEditTime;
extern double        passedTime;
extern double        previousPassedTime1;
extern double        previousPassedTime2;

// Controller / temperatures
extern double Output_lauterTemp;
extern double current_lauterTemp;
extern double previous_lauterTemp;
extern double rawTemp;
extern double current_airTemp;
extern double outdoorTemp;
extern double indoorTemp;
extern double previous_indoorTemp;
extern double previous_airTemp;
extern double previous_outdoorTemp;
extern double* ambientTempPtr;
extern double DutyCycle;
extern double previousDutyCycle;
extern double lastKp, lastKi, lastKHeat, lastDeadband1, lastDeadband2, lastGamma;
extern double currentHeaterPower_W;

extern PI_Controller lauterCtrl_PI;

// PWM
extern int       freq;
extern const int DC_Channel;
extern const int resolution;

// OLED
extern U8G2_SH1106_128X64_NONAME_F_HW_I2C display1;
extern U8G2_SH1106_128X64_NONAME_F_HW_I2C display2;
extern u16_t timeLastTouched;
extern bool  displaySleeping;

// PT100
extern Adafruit_MAX31865  PT100;
extern double             tempPT100;
extern double             filteredTempPT100;
extern unsigned long      lastReadTime;
extern const unsigned long readInterval;
extern double const CAL_T1, CAL_T2, CAL_M1, CAL_M2;
extern double a, b;

// Dallas / OneWire
extern OneWire         oneWire;
extern DallasTemperature dallasTemp;

// Water indicator
extern bool waterDetected;

// Loop timing
extern uint32_t last, now_, loopTime, sumLoopTime, loopCount, avgLoopTime;

// -----------------------------------------------------------------------
// ISR
void IRAM_ATTR handleInterrupt();

// -----------------------------------------------------------------------
// Function declarations

// Arduino entry points
void setup();
void loop();

// Pages
void page_MenuRoot();
void page_MENU_CONTROL_MODE_PI();
void page_MENU_MISC();
void page_MENU_BREWING_LOCATION();

// Menu internals
void initMenuPage(String title, uint8_t itemCount);
void doPointerNavigation();
void incrementDecrementInt(int16_t* v, int16_t amount, int16_t min, int16_t max);
void incrementDecrementFloat(float* v, float amount, float min, float max);
void incrementDecrementDouble(double* v, double amount, double min, double max);
bool menuItemPrintable(uint8_t xPos, uint8_t yPos);
bool menuItemPrintableDisp2(uint8_t xPos, uint8_t yPos);
bool isFlashChanged();

// Print tools (display1)
void printPointer();
void FlashPointer();
void printOnOff(bool val);
void printChars(uint8_t cnt, char c);
void printInt32_tAtWidth(int32_t value, uint8_t width, const char* c);
void printDoubleAtWidth(double value, uint8_t width, const char* c, uint8_t decimals = 1);
void printStringAtWidth(const char* str, uint8_t width);
uint8_t getInt32_tCharCnt(int32_t value);
uint8_t getDoubleCharCnt(double value);

// Print tools (display2)
void printCharsDisplay2(uint8_t cnt, char c);
void printInt32_tAtWidthDisplay2(int32_t value, uint8_t width, char c);
void printDoubleAtWidthDisplay2(double value, uint8_t width, char c);

// Helpers
const char* brewingLocationToString(BrewingLocation location);
void updateAmbientSource(BrewingLocation loc);

// Settings
void sets_SetDeafault();
void sets_Load();
void sets_Save();
void updateSettings();

// Sensors / control
void updateSensorValues();
void updateDisp2();

// WiFi / MQTT / InfluxDB
void setupWiFi();
void setupMQTT();
void setupTime();
void reconnectWiFi();
void reconnectMQTT();
void subscribeMQTT();
void publishMessage();
void publishWifiMqttStatus();
void handleMQTT();
void callback(char* topic, byte* payload, unsigned int length);
bool writeInflux(float lauterTemp, float powerW, double ambientTemp);

// Math / filter
double Filter(double New, double Current, double alpha, double maxValue);
double map(double x, double in_min, double in_max, double out_min, double out_max);
float  roundTo(float value, int decimals);