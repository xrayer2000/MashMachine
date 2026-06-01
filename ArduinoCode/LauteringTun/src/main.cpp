#include "main.h"

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

enum pageType currPage = MENU_ROOT;
void page_MenuRoot();
void page_MENU_CONTROL_MODE_PI();
void page_MENU_MISC();
void page_MENU_BREWING_LOCATION();

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

Mysettings settings;
Mysettings oldSettings;
void sets_SetDeafault();
void sets_Load();
void sets_Save();
bool prevStartMashProgram_ = false;
bool prevStartHopProgram_ = false;
bool prevStartWhirlpoolProgram_ = false;
//-----------------------------------------------------------------------
//Time
unsigned long previousTime = 0; 
unsigned long currentTime;
double tS;
long passedTimeS_mashProgram = 0;
long passedTimeS_hopProgram = 0;
long passedTimeS_whirlpoolProgram = 0;
unsigned long previousSensorRead = 0;
const unsigned long saveInterval = 60000; // 60 000 ms = 60 sekunder
unsigned long lastEditTime = 0;
//-----------------------------------------------------------------------
//CONTROLLER
double Output_lauterTemp;
double current_lauterTemp;
double previous_lauterTemp;
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
// PID PID_lauterTemp(&current_lauterTemp, &Output_lauterTemp, &settings.targetTemp, &current_airTemp, settings.Kp_lauter, settings.Ki_lauter, 0, 0, DIRECT); //P_ON_M
PI_Controller lauterCtrl_PI(&current_lauterTemp, &Output_lauterTemp, &settings.targetTemp, ambientTempPtr); //PI controller with heat loss compensation
double lastKp = NAN;
double lastKi = NAN;
double lastKHeat = NAN;
double lastDeadband1 = NAN;
double lastDeadband2 = NAN;
double lastGamma = NAN;
double currentHeaterPower_W = 0.0;
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
  // Calibration coefficients (computed automatically)
  double a;
  double b;

//-----------------------------------------------------------------------
//Dallas Temperature - Air temperature
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature dallasTemp(&oneWire);
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
//Water indicator
  bool waterDetected = true;
//-----------------------------------------------------------------------
// Loop time measurement variables
uint32_t last  = 0;
uint32_t now  = 0;
uint32_t loopTime  = 0;
uint32_t sumLoopTime = 0;
uint32_t loopCount = 0;
uint32_t avgLoopTime = 0;



void setup() {//=================================================SETUP=======================================================
  Serial.begin(115200);

  // WiFi.disconnect(true);
  // WiFi.mode(WIFI_OFF);

  PT100.begin(MAX31865_3WIRE);
   // Calculate calibration constants
  a = (CAL_T2 - CAL_T1) / (CAL_M2 - CAL_M1);
  b = CAL_T1 - a * CAL_M1;

  currentTime = millis();

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

  // Serial.println("Scanning Wire...");
  // for (uint8_t addr = 1; addr < 127; addr++)
  // {
  //     Wire.beginTransmission(addr);
  //     uint8_t error = Wire.endTransmission();
  //     if (error == 0)
  //     {
  //         Serial.print("Found device at 0x");
  //         Serial.println(addr, HEX);
  //     }
  // }
  // Serial.println("Wire done");

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

  current_lauterTemp = tempPT100; //Temp_C;
  previous_lauterTemp = tempPT100; //Temp_C;

  updateAmbientSource(settings.brewingLocation);

  setupWiFi(); //Home assistant
  setupMQTT(); //Home assistant

  WiFi.onEvent([](WiFiEvent_t event) {
    if (event == SYSTEM_EVENT_STA_DISCONNECTED) {
      if (client.connected()) {
        client.publish("esp32/wifi", "offline", true);
      }
    }
  });

  setupTime(); //NTP time for InfluxDB

  //(PI controller)
  lauterCtrl_PI.SetSampleTime(1000);   // ms
  // --- Output range ---
  lauterCtrl_PI.SetHeaterPowerLimits(0.0, heaterRatedPower_W);
  // --- Tunings ---
  lauterCtrl_PI.SetTunings(settings.Kp_lauter, settings.Ki_lauter, settings.k_heatLoss);
  // --- Safety / behavior ---
  lauterCtrl_PI.SetCaptureBands(settings.deadband1, settings.deadband2, settings.gamma); // °C  // Switch from full power to regulation at ±5 °C
  lauterCtrl_PI.SetIntegralLimit(0.25 * heaterRatedPower_W);
  // Start with feedforward only
  double Q0 = settings.k_heatLoss * (current_lauterTemp - current_airTemp);
  lauterCtrl_PI.Reset(Q0);

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

    float Q_W = 0.0;

    lauterCtrl_PI.Compute();
    Q_W = lauterCtrl_PI.GetCurrentPower_W();

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

  switch (currPage)
  {
    case MENU_ROOT: page_MenuRoot(); break;
    case MENU_CONTROL_MODE_PI: page_MENU_CONTROL_MODE_PI(); break;
    case MENU_MISC: page_MENU_MISC(); break;
    case MENU_BREWING_LOCATION: page_MENU_BREWING_LOCATION(); break;
  }
  if (updateAllItems || updateItemValue)
  {
    display1.sendBuffer();                        //välldigt långsam
    updateAllItems = false;
    updateItemValue = false;
  }

  // printPointer();     // this one take long time
  if (current_lauterTemp != previous_lauterTemp || current_airTemp != previous_airTemp || DutyCycle != previousDutyCycle || settings.targetTemp != oldSettings.targetTemp)
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

    initMenuPage(F("MAIN MENU"), 2);
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
      case 0: currPage = MENU_CONTROL_MODE_PI;         changeValues[0] = false; edditing = false; return;
      case 1: currPage = MENU_MISC;                    changeValues[1] = false; edditing = false; return;
    }
  }
  else
    doPointerNavigation();

  if(!(updateAllItems || updateItemValue)) return;

  for(uint8_t i = 1; i <= 2; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("MODE               ")); break;
        case 2: display1.print(F("MISC               ")); break;
      }
    }
  }
}

void page_MENU_CONTROL_MODE_PI(){//=================================================CONTROL_MODE_PI============================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("CONTROL MODE (PI)"), 9);
    memset(changeValues, 0, sizeof(changeValues));
    initPage = false;
  }

  if(btnOk.Pressed())
  {
    FlashPointer();
    changeValues[cursorPos] = !changeValues[cursorPos];
    edditing = !edditing;
  }

       if(changeValues[0]){incrementDecrementDouble(&settings.targetTemp, 1.0, 15.0, settings.maxMashTemp); settings.targetTemp = round(settings.targetTemp);}
  else if(changeValues[1])incrementDecrementDouble(&settings.Kp_lauter, 0.5, 0.0, 500.0);
  else if(changeValues[2])incrementDecrementDouble(&settings.Ki_lauter, 0.01, 0.0, 10.0);
  else if(changeValues[3])incrementDecrementDouble(&settings.k_heatLoss, 0.1, 0.0, 25.0);
  else if(changeValues[4])incrementDecrementDouble(&settings.deadband1, 0.5, settings.deadband2, 20.0);
  else if(changeValues[5])incrementDecrementDouble(&settings.deadband2, 0.1, 0.0, settings.deadband1);
  else if(changeValues[6])incrementDecrementDouble(&settings.gamma, 0.01, 0.0, 1.0);
  else if(changeValues[8]) {currPage = MENU_ROOT; sets_Save(); changeValues[8] = false; initPage = true;}
  else 
    doPointerNavigation(); 

  if(!(updateAllItems || updateItemValue)) return;

  display1.drawVLine(9 * CHAR_X, 0, 64);

  for(uint8_t i = 1; i <= 9; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1: display1.print(F("Setpoint ")); break;
        case 2: display1.print(F("Kp       ")); break;
        case 3: display1.print(F("Ki       ")); break;
        case 4: display1.print(F("k_heat   ")); break;
        case 5: display1.print(F("Band 1   ")); break;
        case 6: display1.print(F("Band 2   ")); break;
        case 7: display1.print(F("Gamma    ")); break;
        case 8: display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 9: display1.print(F("Back     ")); break;
      }
    }
    if(menuItemPrintable(10, i))
    {
      switch(i)
      {
        case 1: printInt32_tAtWidth((uint32_t)settings.targetTemp, 3, "C"); break;
        case 2: printDoubleAtWidth(settings.Kp_lauter, 4, "W/C"); break;
        case 3: printDoubleAtWidth(settings.Ki_lauter, 4, "W/Cs", 2); break;
        case 4: printDoubleAtWidth(settings.k_heatLoss, 4, "W/C"); break;
        case 5: printDoubleAtWidth(settings.deadband1, 4, "C"); break;
        case 6: printDoubleAtWidth(settings.deadband2, 3, "C"); break;
        case 7: printDoubleAtWidth(settings.gamma, 4, " ", 2); break;
      }
    }
  }
}


void page_MENU_MISC(){//=================================================MISC==========================================================
  if(initPage)
  {
    cursorPos = 0;
    dispOffset = 0;
    initMenuPage(F("MISC"), 10);
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
 
  else if(changeValues[1])incrementDecrementDouble(&settings.filter_adc1, 0.01f, 0.0f, 0.99f);
  else if(changeValues[2])incrementDecrementDouble(&settings.filterDC, 0.01f, 0.0f, 0.99f);
  else if(changeValues[3])incrementDecrementInt(&settings.maxMashTemp, 1, 5, 110);
  else if(changeValues[4])incrementDecrementInt(&settings.timeBeforeDisable, 1, 1, 90);
  else if(changeValues[5])incrementDecrementInt(&settings.alarmVolume, 1, 0, 100);
  else if(changeValues[6])incrementDecrementInt(&settings.alarmTime, 1, 1, 30);
  else if(changeValues[7]){currPage = MENU_BREWING_LOCATION; changeValues[8] = false; initPage = true; return;}
  else if(changeValues[9]){currPage = MENU_ROOT; sets_Save(); initPage = true; changeValues[9] = false; return;}
  else 
    doPointerNavigation(); 

  if(!(updateAllItems | updateItemValue)) return;
   
  display1.drawVLine(11 * CHAR_X, 0, 64);

  for(uint8_t i = 1; i <= 10; i++)
  {
    if(menuItemPrintable(1, i))
    {
      switch(i)
      {
        case 1:  display1.print(F("Power        ")); break;  
        case 2:  display1.print(F("Filter Temp  ")); break;
        case 3:  display1.print(F("Filter DC    ")); break;
        case 4:  display1.print(F("Max Temp     ")); break;
        case 5:  display1.print(F("Disable disp ")); break;
        case 6:  display1.print(F("Alarm Volume ")); break;
        case 7:  display1.print(F("Alarm Time   ")); break;
        case 8:  display1.print(F("Location     ")); break;
        case 9:  display1.drawHLine(5, display1.getCursorY(), 120); break;
        case 10: display1.print(F("Back         ")); break;
      }
    }
    if(menuItemPrintable(12, i))
    {
      switch(i)
      {
        case 1: printOnOff(settings.power); break;
        case 2: printDoubleAtWidth(settings.filter_adc1, 3, " ", 2); break;
        case 3: printDoubleAtWidth(settings.filterDC, 3, " ", 2); break;
        case 4: printInt32_tAtWidth(settings.maxMashTemp, 3, "C"); break;
        case 5: printInt32_tAtWidth(settings.timeBeforeDisable, 3, "m"); break;
        case 6: printInt32_tAtWidth(settings.alarmVolume, 3, "%"); break;
        case 7: printInt32_tAtWidth(settings.alarmTime, 3, "s"); break;
        case 8: printStringAtWidth(brewingLocationToString(settings.brewingLocation), 3); break;
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
  /* Krav
  I: Elementet får aldrig vara på om det inte finns vatten i bryggverket
  */ 


  if(settings.power)
  {
    digitalWrite(ledOnPin, HIGH);
    digitalWrite(ledWaterDetectedPin, !waterDetected);


    float m = passedTimeS_hopProgram / 60.0f;
    float d = settings.alarmTime / 60.0f;


    //analogWrite(alarmPin, hopsAlarm ? settings.alarmVolume / 100.0f * 4095.0f : 0);

    // --- Retune controller ONLY if changed ---
    if (settings.Kp_lauter != lastKp || settings.Ki_lauter != lastKi 
    || settings.k_heatLoss != lastKHeat || settings.deadband1 != lastDeadband1 || settings.deadband2 != lastDeadband2
    || settings.gamma != lastGamma)
    {
      lauterCtrl_PI.SetTunings(settings.Kp_lauter, settings.Ki_lauter, settings.k_heatLoss);
      lauterCtrl_PI.SetCaptureBands(settings.deadband1, settings.deadband2, settings.gamma);
      double Q0 = settings.k_heatLoss * (current_lauterTemp - current_airTemp);
      lauterCtrl_PI.Reset(Q0);

      lastKp = settings.Kp_lauter;
      lastKi = settings.Ki_lauter;
      lastKHeat = settings.k_heatLoss;
      lastDeadband1 = settings.deadband1;
      lastDeadband2 = settings.deadband2;
      lastGamma = settings.gamma;

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

  lauterCtrl_PI.SetAmbientTemp(newPtr);
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

    // Dallas: read result from PREVIOUS request, then fire next one
    previous_airTemp = current_airTemp;
    current_airTemp = dallasTemp.getTempCByIndex(0);  // reads last conversion
    dallasTemp.requestTemperatures();                 // fires next (non-blocking)

    waterDetected = !digitalRead(waterDetectedPin);


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

    previous_lauterTemp = current_lauterTemp;
    current_lauterTemp = filteredTempPT100; //Temp_C;
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
  display2.setCursor(DISP2_CHAR_X * 13, DISP2_CHAR_Y * 2); printDoubleAtWidthDisplay2(current_lauterTemp, 3, 'C');
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

    if (mqtttopic == "home/weather/outdoorTemp")
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

void publishMessage() //Home Assistant only
{
  StaticJsonDocument<512> doc;

  // --- Core readings ---
  doc["lauteringTemp"] = roundTo(current_lauterTemp, 3);  // 3 decimal
  doc["airTemp"]       = roundTo(*ambientTempPtr, 2);   // 2 decimal

  // --- Setpoint ---
  doc["targetTemp"]    = settings.targetTemp;

  // --- Power / control ---
  doc["dutyCycle"]     = roundTo(DutyCycle / 4095.0f * 100.0f, 3);
  doc["powerIn"]       = roundTo(DutyCycle * PWM_to_W, 1); // integer watts

  // --- Boolean states (REAL booleans) ---
  doc["waterDetected"] = waterDetected;

  // --- PI parameters ---
  doc["kp"]            = roundTo(settings.Kp_lauter, 1);
  doc["ki"]            = roundTo(settings.Ki_lauter, 3);
  doc["heatLoss"]      = roundTo(settings.k_heatLoss, 1); //change to: model->kLoss_W_per_degC

  // --- Filters (rounded for readability) ---
  doc["filterTemp"]    = roundTo(settings.filter_adc1, 2);
  doc["filterDC"]      = roundTo(settings.filterDC, 2);

  // --- Serialize and publish ---
  char buffer[512];
  size_t n = serializeJson(doc, buffer);
  client.publish("lauteringTun/state", buffer, n);

  //InfluxDB
  if (!writeInflux(current_lauterTemp, DutyCycle * PWM_to_W, *ambientTempPtr)) {
  Serial.println("Influx write failed");
  }
}

void subscribeMQTT() //Home Assistant only
{
  if (client.connected())
  {
    client.subscribe("home/weather/outdoorTemp");
    client.subscribe("home/datalogger/airTemp");
  }
}

bool writeInflux(float lauterTemp, float powerW, double ambientTemp) //Only InfluxDB
{
  if (WiFi.status() != WL_CONNECTED)
    return false;

  HTTPClient http;
  http.begin(privates.influxURL);
  http.addHeader("Authorization", String("Token ") + privates.influxToken);
  http.addHeader("Content-Type", "text/plain");

  String line =
    "lauteringTun,source=esp32 "
    "lauterTemp=" + String(lauterTemp, 3) +
    ",power="   + String(powerW, 1) +
    ",airtemp=" + String(ambientTemp, 2) +
    ",targettemp=" + String(settings.targetTemp, 1) +
    " " + String((uint64_t)time(nullptr) * 1000000000ULL) + "\n";

  int code = http.POST(line);
  Serial.print("Influx HTTP code: ");
  Serial.println(code);
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