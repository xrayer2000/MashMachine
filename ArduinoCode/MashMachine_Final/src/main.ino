// #include <Arduino.h>
// #include <Wire.h> //temperatur
// #include <Adafruit_GFX.h> //oled
// #include <Adafruit_SSD1306.h> //oled
// #include <U8g2lib.h> //oled
// #include <Adafruit_I2CDevice.h>
// #include <OneWire.h> //temperatur
// #include <DallasTemperature.h> //temperatur
// #include <PID_v1.h> //PID
// #include <PressButton.h> //Interface
// #include <RotaryEncoder.h>; //Interface
// #include <EEPROM.h> //Save Settings
// #include <WiFi.h> //Home assistant
// #include <PubSubClient.h> //Home assistant
// #include "Privates.h" //Homeassistant

// Privates privates; //Homeassistant
// WiFiClient espClient; //Home assistant
// PubSubClient client(espClient); //Home assistant
// char messages[50]; //Home assistant

// #define SCREEN_WIDTH 128 // OLED display1 width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display1 height, in pixels

// #define oneWireBus1 4 //temperatur
// #define oneWireBus2 32 //temperatur thermostat
// #define raiseTemp_Pin 26
// #define BTN_OK 25 //ok
// #define ledOnPin 14
// #define alarmPin 33

// #define DISP_ITEM_ROWS 4
// #define DISP_CHAR_WIDTH 16
// #define PACING_MC 30 //25
// #define FLASH_RST_CNT 3 //30
// #define SETTINGS_CHKVAL 3647 
// #define CHAR_X SCREEN_WIDTH/DISP_CHAR_WIDTH // 240/16
// #define CHAR_Y SCREEN_HEIGHT/DISP_ITEM_ROWS // 240/8
// #define SHIFT_UP 0 //240/8

// //rotary encoder
// #define outputA 13
// #define outputB 16
// RotaryEncoder encoder(outputA,outputB,5,1,60000);

// //Buttons
// PressButton btnOk(BTN_OK);

// //Menu structure
// enum pageType{
//   MENU_ROOT,
//   MENU_TARGET_TEMP,
//   MENU_TARGET_TEMP_A,
//   MENU_TARGET_TEMP_B,
//   MENU_TEMPERATURES,
//   MENU_MISC,
//   MENU_PID,
//   MENU_TIME
// };

// enum pageType currPage = MENU_ROOT;
// void page_MenuRoot();
// void page_MENU_TARGET_TEMP();
// void page_MENU_TARGET_TEMP_A();
// void page_MENU_TARGET_TEMP_B();
// void page_TEMPERATURES();
// void page_MENU_MISC();
// void page_MENU_PID();
// void page_MENU_TIME();

// //Menu internals
// uint32_t loopStartMs;
// boolean updateAllItems;
// boolean updateItemValue;
// uint8_t itemCnt;
// uint8_t pntrPos;
// uint8_t dispOffset;
// uint8_t root_pntrPos = 1;
// uint8_t root_dispOffset = 0;
// uint8_t flashCntr;
// boolean flashIsOn;
// void initMenuPage(String title, uint8_t itemCount);
// void captureButtonDownStates();
// void incrementDecrementDouble(double *v, double amount, double min, double max);
// void doPointerNavigation();
// bool isFlashChanged();
// void pacingWait();
// bool menuItemPrintable(uint8_t xPos, uint8_t yPos);

// //Print tools
// void printPointer();
// void printOnOff(bool val);
// void printUint32_tAtWidth(uint32_t value, uint8_t width, char c, boolean isRight);
// void printDoubleAtWidth(double value, uint8_t width, char c, boolean isRight);

// //Settings
// #pragma pack(1) //memory alignment
// struct Mysettings{
//   double Kp_mash = 125.0;
//   double Ki_mash = 0.0;
//   double  Kd_mash = 0.0;  
//   double Kp_element = 125.0/32;
//   double Ki_element = 0.0;
//   double  Kd_element = 0.0;  

//   double targetTemp = 25;

//   boolean manualMode = 1;
//   double temp1 = 65;
//   double time1 = 45;
//   double temp2 = 70;
//   double time2 = 30;
//   double temp3 = 77;
//   double time3 = 15;

//   boolean power = true;
//   double RawLow = 0.31;
//   double RawHigh = 99.56;
//   double maxElementTemp = 30; //126
//   double marginalTemp = 10;
//   double dutycycleThreshold = 50;

//   uint16_t settingsCheckValue = SETTINGS_CHKVAL;
// };

// Mysettings settings;
// void sets_SetDefaults();
// void sets_Load();
// void sets_Save();

// double current_temp_DS18B20;
// double previous_temp_DS18B20;
// double current_thermostatTemp_DS18B20;
// double previous_thermostatTemp_DS18B20;

// double RawRange;
// double ReferenceHigh = 100.0;
// double ReferenceLow = 0.0;
// double ReferenceRange = ReferenceHigh - ReferenceLow;

// bool raiseTemp_status;
// double previousTargetTemp;
// unsigned long  previousTime = 0; 
// unsigned long currentTime;
// unsigned long loopTime;
// bool heat = false;

// //PID
// double error;
// double previousError;
// double dt, last_time;
// double integral, previous, pid = 0;
// double Output_mashTemp;
// double Output_elementTemp;
// double tempC;
// float DutyCycle = 0;
// float previousDutyCycle = 0;
// double targetElementTemp = settings.maxElementTemp - settings.marginalTemp;
// PID PID_mashTemp(&current_temp_DS18B20, &Output_mashTemp, &settings.targetTemp, settings.Kp_mash, settings.Ki_mash, settings.Kd_mash, DIRECT);
// PID PID_elementTemp(&current_thermostatTemp_DS18B20, &Output_elementTemp, &targetElementTemp, settings.Kp_element, settings.Ki_element, settings.Kd_element, DIRECT);

// double tS;
// long passedTimeS;
// long previousPassedTimeS;

// // setting PWM properties
// int freq = 5; //5
// const int ledChannel = 0;
// const int resolution = 12;

// //rotary encoder
// int counter = 1; 
// int aState;
// int bState;
// int aLastState;  
 
// // Declaration for an SSD1306 display1 connected to I2C (SDA, SCL pins)
// //Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// U8G2_SH1106_128X64_NONAME_F_HW_I2C display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// U8G2_SH1106_128X64_NONAME_F_HW_I2C display2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// OneWire oneWire1(oneWireBus1);
// DallasTemperature sensor1(&oneWire1);

// OneWire oneWire2(oneWireBus2);
// DallasTemperature sensor2(&oneWire2);


// void setup() {//=================================================SETUP=======================================================

//   Serial.begin(115200);
//   setupWiFi(); //Home assistant
//   client.setServer(privates.broker, 1883); //Home assistant

//   sensor1.begin();
//   sensor2.begin();
//   sensor1.setResolution(9);
//   sensor2.setResolution(9);

//   currentTime = millis();
//   loopTime = currentTime;

//   // configure LED PWM functionalitites
//   ledcSetup(ledChannel, freq, resolution);

//   // attach the channel to the GPIO to be controlled
//   ledcAttachPin(raiseTemp_Pin, ledChannel);

//   pinMode(oneWireBus1, INPUT_PULLUP);
//   pinMode(oneWireBus2, INPUT_PULLUP);
//   pinMode(ledOnPin, OUTPUT);
//   pinMode(alarmPin, OUTPUT);
//   pinMode(outputA,INPUT_PULLUP);
//   pinMode(outputB,INPUT_PULLUP);

//   Wire.setClock(3400000 );      
//   display1.setBusClock(3400000 ); 
//   display2.setBusClock(3400000 ); 

//   display1.setI2CAddress(0x78); 
//   display1.begin();  
//   display1.setFont(u8g2_font_6x10_mf);	// choose a suitable font
//   display1.clearBuffer();					// clear the internal memory
//   display1.setCursor(0, 0);

//   display2.setI2CAddress(0x7A);
//   display2.begin();  
//   display2.setFont(u8g2_font_6x10_mf);	// choose a suitable font  
//   display2.clearBuffer();					// clear the internal memory
//   display2.setCursor(0, 0);

//   EEPROM.begin(512);
//   sets_Load();
  
//   PID_mashTemp.SetTunings(settings.Kp_mash, settings.Ki_mash, settings.Kd_mash);
//   PID_mashTemp.SetOutputLimits(0,4095.0 * settings.dutycycleThreshold * 0.01);
//   PID_mashTemp.SetMode(AUTOMATIC);  
//   PID_mashTemp.Compute();

//   PID_elementTemp.SetTunings(settings.Kp_element, settings.Ki_element, settings.Kd_element);
//   PID_elementTemp.SetOutputLimits(0,settings.maxElementTemp - settings.marginalTemp);
//   PID_elementTemp.SetMode(AUTOMATIC);
//   PID_elementTemp.Compute();

// }

// void loop() { //=================================================LOOP=======================================================

//   currentTime = millis();
  
//   RawRange = settings.RawHigh - settings.RawLow;
//   switch (currPage)
//   {
//   case MENU_ROOT: page_MenuRoot(); break;
//   case MENU_TARGET_TEMP: page_MENU_TARGET_TEMP(); break;
//   case MENU_TARGET_TEMP_A: page_MENU_TARGET_TEMP_A(); break;
//   case MENU_TARGET_TEMP_B: page_MENU_TARGET_TEMP_B(); break;
//   case MENU_TEMPERATURES: page_TEMPERATURES(); break;
//   case MENU_MISC: page_MENU_MISC(); break;
//   case MENU_PID: page_MENU_PID(); break;
//   case MENU_TIME: page_MENU_TIME(); break;
//   }
// }

// void page_MenuRoot(){//=================================================ROOT_MENU============================================
//   pntrPos = root_pntrPos;
//   dispOffset = root_dispOffset;
//   //Serial.println("root");
//   initMenuPage(F("MAIN MENU"), 5);
  
//   double passedTime,previousPassedTime = 0;

//   while(true){
    
//     passedTime = millis() / (1000.0);

//    if(passedTime - previousPassedTime >= 0.8)
//       updateSettings();

//     if(updateAllItems)
//     {
//       previousPassedTime = passedTime;
//       passedTime = 0;
//       updateSensorValues(); 
//       updateDisp2();

//       display1.clearBuffer(); 
//       if(menuItemPrintable(1,1)){display1.print(F("TARGET_TEMP  "));} //kan inte ändra
//       if(menuItemPrintable(1,2)){display1.print(F("LIVE VARIBLES"));} //kan inte ändra
//       if(menuItemPrintable(1,3)){display1.print(F("MISC         "));} //kan ändra
//       if(menuItemPrintable(1,4)){display1.print(F("PID          "));} //settings, kan ändra
//       if(menuItemPrintable(1,5)){display1.print(F("MashProgram  "));} //settings, kan ändra  
//       printPointer();
//       display1.sendBuffer();
//     }

//     updateAllItems = false;
//     captureButtonDownStates();

//     if(btnOk.PressReleased())
//     {
//       FlashPointer();
//       root_pntrPos = pntrPos;
//       root_dispOffset = dispOffset;
//       //Serial.print("root_pntrPos-root_dispOffset: ");
//       //Serial.println(root_pntrPos-root_dispOffset);
//       switch (pntrPos)
//       {
//       case 1: currPage = MENU_TARGET_TEMP; return;
//       case 2: currPage = MENU_TEMPERATURES; return;
//       case 3: currPage = MENU_MISC; return;
//       case 4: currPage = MENU_PID; return;
//       case 5: currPage = MENU_TIME; return;
//       }
//     }
//     doPointerNavigation();
//     publishMessage();
//   }
//   }
// void page_MENU_TARGET_TEMP(){//=================================================TARGET_TEMP============================================
//   pntrPos = 1;
//   dispOffset = 0;
//   initMenuPage(F("TARGET_TEMP"), 4);
  
//   double passedTime,previousPassedTime = 0;
//   bool changeValue = false;
//   while(true){
//     passedTime = millis() / (1000.0);

//     if(passedTime - previousPassedTime >= 0.8)
//       updateSettings();

//     if(changeValue)
//       incrementDecrementDouble(&settings.targetTemp, 1, 15.0, 100.0);
//     else 
//      doPointerNavigation(); 

//     if(updateAllItems)
//     {
//       previousPassedTime = passedTime;
//       passedTime = 0;
//       updateSensorValues();
//       updateDisp2();
//       display1.clearBuffer();
//       if(menuItemPrintable(1,1)){display1.print(F("Target_Temp =     "));}
//       if(menuItemPrintable(1,2)){display1.print(F("TargetTemp_A      "));}
//       if(menuItemPrintable(1,3)){display1.print(F("TargetTemp_B      "));}
//       if(menuItemPrintable(1,4)){display1.print(F("Back              "));}
//       printPointer();
//     }

//     if(updateAllItems || updateItemValue)
//     {
//       if(menuItemPrintable(13,1)){printDoubleAtWidth(settings.targetTemp, 4, ' ', false);}
//       display1.sendBuffer();
//     }
//     updateAllItems = false;
//     updateItemValue = false;

//     captureButtonDownStates();

//     if(btnOk.PressReleased())
//     {
//       FlashPointer();
//       switch (pntrPos)
//       {
//         case 1: changeValue = !changeValue; break;
//         case 2: currPage = MENU_TARGET_TEMP_A; return;
//         case 3: currPage = MENU_TARGET_TEMP_B; return;
//         case 4: currPage = MENU_ROOT; sets_Save(); return;
//       }
//     }
//     publishMessage();
//   }
// }
// void page_MENU_TARGET_TEMP_A(){
//   initMenuPage(F("TargetTemp_A"), 2);
  
//   while(true){
//     updateSettings();
//     updateSensorValues();
//     if(updateAllItems){
//       updateDisp2();
//       display1.clearBuffer();
//       if(menuItemPrintable(1,1)){display1.print(F("TargetTemp A    "));}
//       if(menuItemPrintable(1,2)){display1.print(F("Back            "));}
//       printPointer();
//       display1.sendBuffer();
//     }
//     updateAllItems = false;
//     captureButtonDownStates();
//     if(btnOk.PressReleased())
//     {
//       switch (pntrPos)
//         case 2: currPage = MENU_TARGET_TEMP; sets_Save(); return;
//     }
//     doPointerNavigation();
//     publishMessage();
//   }
// }
// void page_MENU_TARGET_TEMP_B(){
//   initMenuPage(F("TargetTemp_B"), 2);
  
//   while(true){
//     updateSettings();
//     updateSensorValues();
//     if(updateAllItems){
//       updateDisp2();
//       display1.clearBuffer();
//       if(menuItemPrintable(1,1)){display1.print(F("TargetTemp B   "));}
//       if(menuItemPrintable(1,2)){display1.print(F("Back           "));}
//       printPointer();
//       display1.sendBuffer();
//     }
//     updateAllItems = false;
//     captureButtonDownStates();
//     if(btnOk.PressReleased())
//     {
//       switch (pntrPos)
//         case 2: currPage = MENU_TARGET_TEMP; sets_Save(); return;
//     }
//     doPointerNavigation();
//     publishMessage();
//   }
// }
// void page_TEMPERATURES(){//=================================================LIVE_VARIABLES============================================
//   pntrPos = 1;
//   dispOffset = 0;
//   initMenuPage(F("LIVE_VARIABLES"), 4);
//   double passedTime,previousPassedTime = 0;

//   while(true){
//     passedTime = millis() / (1000.0);

//     if(passedTime - previousPassedTime >= 0.8)
//       updateSettings();

//     if(updateAllItems)
//     {
//       previousPassedTime = passedTime;
//       passedTime = 0;
//       updateSensorValues();
//       updateDisp2();
//       display1.clearBuffer();
//       //display1.setTextColor(WHITE, BLACK);
//       if(menuItemPrintable(1,1)){display1.print(F("mashTemp =      "));}
//       if(menuItemPrintable(1,2)){display1.print(F("DC       =      "));}
//       if(menuItemPrintable(1,3)){display1.print(F("eleTemp  =      "));}
//       if(menuItemPrintable(1,4)){display1.print(F("Back            "));}
      
//       if(menuItemPrintable(11,1)){printDoubleAtWidth(current_temp_DS18B20, 4, 'C', false);}
//       if(menuItemPrintable(11,2)){printDoubleAtWidth(DutyCycle/4096.0 * 100.0, 4, '%', false);}
//       if(menuItemPrintable(11,3)){printDoubleAtWidth(current_thermostatTemp_DS18B20, 4, 'C', false);}
//       printPointer();
//       display1.sendBuffer();
//     }
//     updateAllItems = false;
//     updateItemValue = false;
//     captureButtonDownStates();
//     if(btnOk.PressReleased())
//     {
//       FlashPointer();
//       switch (pntrPos)
//       {
//         case 4: currPage = MENU_ROOT; sets_Save(); return;
//       }
//     }
//     doPointerNavigation();
//     publishMessage();
//   }
// }
// void page_MENU_MISC(){//=================================================MISC==========================================================
//   pntrPos = 1;
//   dispOffset = 0;
//   initMenuPage(F("MISC"), 7);
//   double passedTime,previousPassedTime = 0;
//   bool changeValues [10];

//   while(true){
    
//     passedTime = millis() / (1000.0);

//     if(passedTime - previousPassedTime >= 0.8)
//       updateSettings();
     
//     if(updateAllItems)
//     {
//       previousPassedTime = passedTime;
//       passedTime = 0;
//       updateSensorValues();
//       updateDisp2();
//       display1.clearBuffer();
//       if(menuItemPrintable(1,1)){display1.print(F("POWER        =     "));}
//       if(menuItemPrintable(1,2)){display1.print(F("RawLow       =     "));}
//       if(menuItemPrintable(1,3)){display1.print(F("RawHigh      =     "));}
//       if(menuItemPrintable(1,4)){display1.print(F("max_Ele_temp =     "));}
//       if(menuItemPrintable(1,5)){display1.print(F("marginalTemp =     "));}
//       if(menuItemPrintable(1,6)){display1.print(F("DC_TH        =     "));}
//       if(menuItemPrintable(1,7)){display1.print(F("Back               "));}
//     }
//     if(updateAllItems || updateItemValue)
//     {
//       if(menuItemPrintable(12,1)){printOnOff(settings.power);}
//       if(menuItemPrintable(12,2)){printDoubleAtWidth(settings.RawLow, 6, ' ', false);}
//       if(menuItemPrintable(12,3)){printDoubleAtWidth(settings.RawHigh, 6, ' ', false);}
//       if(menuItemPrintable(12,4)){printUint32_tAtWidth(settings.maxElementTemp, 6, 'C', false);}
//       if(menuItemPrintable(12,5)){printUint32_tAtWidth(settings.marginalTemp, 6, 'C', false);}
//       if(menuItemPrintable(12,6)){printUint32_tAtWidth(settings.dutycycleThreshold, 6, '%', false);}
//       printPointer();
//       display1.sendBuffer();
//     }

//     updateAllItems = false;
//     updateItemValue = false;
//     captureButtonDownStates();
//     if(btnOk.PressReleased())
//     {
//       FlashPointer();
//       switch (pntrPos)
//       {
//         case 1: changeValues [0] = !changeValues [0]; break; 
//         case 2: changeValues [1] = !changeValues [1]; break; 
//         case 3: changeValues [2] = !changeValues [2]; break;
//         case 4: changeValues [3] = !changeValues [3]; break; 
//         case 5: changeValues [4] = !changeValues [4]; break;
//         case 7: currPage = MENU_ROOT; sets_Save(); return;
//       }
//     }
//     double amountConstant = 0.01;
//     if(changeValues[0])
//     {
//       *&settings.power = !*&settings.power;
//       changeValues [0] = false;
//       updateItemValue = true; 
//     }
//     else if(changeValues[1])incrementDecrementDouble(&settings.RawLow, 0.05, 0.0, 5.0);
//     else if(changeValues[2])incrementDecrementDouble(&settings.RawHigh, 0.05, 95.0, 105.0);
//     else if(changeValues[3])incrementDecrementDouble(&settings.maxElementTemp, 1, 15, 130);
//     else if(changeValues[4])incrementDecrementDouble(&settings.marginalTemp, 1, 0, 20);
//     else 
//       doPointerNavigation(); 

//     PID_elementTemp.SetTunings(settings.Kp_element, settings.Ki_element, settings.Kd_element);
//     PID_elementTemp.SetOutputLimits(0,settings.maxElementTemp - settings.marginalTemp);
//     PID_elementTemp.SetMode(AUTOMATIC);
//     PID_elementTemp.Compute();
//     publishMessage();
//   }
// }
// void page_MENU_PID(){//=================================================PID===============================================================
//   pntrPos = 1;
//   dispOffset = 0;
//   initMenuPage(F("PID"), 7);
//   double passedTime,previousPassedTime = 0;
//   bool changeValues [10]; 

//   while(true){
//   passedTime = millis() / (1000.0);

//    if(passedTime - previousPassedTime >= 0.8)
//       updateSettings();

//     if(updateAllItems)
//     {
//       previousPassedTime = passedTime;
//       passedTime = 0;
//       updateSensorValues();
//       updateDisp2();
//       display1.clearBuffer();
//       if(menuItemPrintable(1,1)){display1.print(F("Kp_mash  =           "));}
//       if(menuItemPrintable(1,2)){display1.print(F("Ki_mash  =           "));}
//       if(menuItemPrintable(1,3)){display1.print(F("kd_mash  =           "));}
//       if(menuItemPrintable(1,4)){display1.print(F("Kp_ele   =           "));}
//       if(menuItemPrintable(1,5)){display1.print(F("Ki_ele   =           "));}
//       if(menuItemPrintable(1,6)){display1.print(F("kd_ele   =           "));}
//       if(menuItemPrintable(1,7)){display1.print(F("Back                 "));}
//     }

//     if(updateAllItems || updateItemValue)
//     {
//       if(menuItemPrintable(13,1)){printDoubleAtWidth(settings.Kp_mash, 4, ' ', false);}
//       if(menuItemPrintable(13,2)){printDoubleAtWidth(settings.Ki_mash, 4, ' ', false);}
//       if(menuItemPrintable(13,3)){printDoubleAtWidth(settings.Kd_mash, 4, ' ', false);}
//       if(menuItemPrintable(13,4)){printDoubleAtWidth(settings.Kp_element, 4, ' ', false);}
//       if(menuItemPrintable(13,5)){printDoubleAtWidth(settings.Ki_element, 4, ' ', false);}
//       if(menuItemPrintable(13,6)){printDoubleAtWidth(settings.Kd_element, 4, ' ', false);}
//       //display1.display();
//       printPointer();
//       display1.sendBuffer();
//     }

//     updateAllItems = false;
//     updateItemValue = false;

//     captureButtonDownStates();

//     if(btnOk.PressReleased())
//     {
//       FlashPointer();
//       switch (pntrPos)
//       {
//         case 1: changeValues [0] = !changeValues [0]; break;
//         case 2: changeValues [1] = !changeValues [1]; break;
//         case 3: changeValues [2] = !changeValues [2]; break;
//         case 4: changeValues [3] = !changeValues [3]; break;
//         case 5: changeValues [4] = !changeValues [4]; break;
//         case 6: changeValues [5] = !changeValues [5]; break;
//         case 7: currPage = MENU_ROOT; sets_Save(); return;
//       }
//     }
//          if(changeValues[0])incrementDecrementDouble(&settings.Kp_mash, 1, 0.0, 1000.0);
//     else if(changeValues[1])incrementDecrementDouble(&settings.Ki_mash, 0.1, 0.0, 200);
//     else if(changeValues[2])incrementDecrementDouble(&settings.Kd_mash, 0.05, 0.0, 100);
//     else if(changeValues[3])incrementDecrementDouble(&settings.Kp_element, 0.1, 0.0, 200.0);
//     else if(changeValues[4])incrementDecrementDouble(&settings.Ki_element, 0.05, 0.0, 200);
//     else if(changeValues[5])incrementDecrementDouble(&settings.Kd_element, 0.05, 0.0, 100);
//     else 
//       doPointerNavigation(); 

//     PID_mashTemp.SetTunings(settings.Kp_mash, settings.Ki_mash, settings.Kd_mash);
//     PID_mashTemp.SetOutputLimits(0,4095.0 * settings.dutycycleThreshold * 0.01);
//     PID_mashTemp.SetMode(AUTOMATIC);
//     PID_mashTemp.Compute();

//     PID_elementTemp.SetTunings(settings.Kp_element, settings.Ki_element, settings.Kd_element);
//     PID_elementTemp.SetOutputLimits(0,settings.maxElementTemp - settings.marginalTemp);
//     PID_elementTemp.SetMode(AUTOMATIC);
//     PID_elementTemp.Compute();
    
//     publishMessage(); //Home assistant
//   }
// }

// void page_MENU_TIME(){//=================================================MASH_PROGRAM====================================================
//   pntrPos = 1;
//   dispOffset = 0;
//   initMenuPage(F("MashProgram"), 9);
//   double passedTime,previousPassedTime = 0;
//   bool changeValues [10];

//   while(true){
//   passedTime = millis() / (1000.0);

//   if(passedTime - previousPassedTime >= 0.8)
//       updateSettings();

//   if(updateAllItems)
//   {
//     previousPassedTime = passedTime;
//     passedTime = 0;
//     updateSensorValues();
//     updateDisp2();
//     display1.clearBuffer();
//     if(menuItemPrintable(1,1)){display1.print(F("Manual =            "));}
//     if(menuItemPrintable(1,2)){display1.print(F("mash:               "));}
//     if(menuItemPrintable(1,3)){display1.print(F("temp_1 =            "));}
//     if(menuItemPrintable(1,4)){display1.print(F("time_1 =            "));}
//     if(menuItemPrintable(1,5)){display1.print(F("temp_2 =            "));}
//     if(menuItemPrintable(1,6)){display1.print(F("time_2 =            "));}
//     if(menuItemPrintable(1,7)){display1.print(F("temp_3 =            "));}
//     if(menuItemPrintable(1,8)){display1.print(F("time_3 =            "));}
//     if(menuItemPrintable(1,9)){display1.print(F("Back                "));}
//   }

//   if(updateAllItems || updateItemValue)
//   {
//     if(menuItemPrintable(11,1)){printOnOff(settings.manualMode);}
//     if(menuItemPrintable(7,2)){printDoubleAtWidth(passedTimeS/60.0, 3, 'm', false);}
//     if(menuItemPrintable(13,2)){printDoubleAtWidth(settings.targetTemp, 10, 'C', false);}
//     if(menuItemPrintable(11,3)){printDoubleAtWidth(settings.temp1, 10, 'C', false);}
//     if(menuItemPrintable(11,4)){printDoubleAtWidth(settings.time1, 10, 'm', false);}
//     if(menuItemPrintable(11,5)){printDoubleAtWidth(settings.temp2, 10, 'C', false);}
//     if(menuItemPrintable(11,6)){printDoubleAtWidth(settings.time2, 10, 'm', false);}
//     if(menuItemPrintable(11,7)){printDoubleAtWidth(settings.temp3, 10, 'C', false);}
//     if(menuItemPrintable(11,8)){printDoubleAtWidth(settings.time3, 10, 'm', false);}
//     printPointer();
//     display1.sendBuffer();
//   }

//   updateAllItems = false;
//   updateItemValue = false;

//   captureButtonDownStates();

//   if(btnOk.PressReleased())
//   {
//     FlashPointer();
//     switch (pntrPos)
//     {
//       case 1: changeValues [0] = !changeValues [0]; break;
//       case 3: changeValues [1] = !changeValues [1]; break;
//       case 4: changeValues [2] = !changeValues [2]; break;
//       case 5: changeValues [3] = !changeValues [3]; break;
//       case 6: changeValues [4] = !changeValues [4]; break;
//       case 7: changeValues [5] = !changeValues [5]; break;
//       case 8: changeValues [6] = !changeValues [6]; break;
//       case 9: currPage = MENU_ROOT; sets_Save(); return; 
//     }
//   }

//   if(changeValues[0])
//   {
//     *&settings.manualMode = !*&settings.manualMode;
//     changeValues [0] = false;
//     updateItemValue = true; 
//   } 
//   else if(changeValues[1])incrementDecrementDouble(&settings.temp1, 1.0, 15.0, 100.0);
//   else if(changeValues[2])incrementDecrementDouble(&settings.time1, 1.0, 0.0, 90.0);
//   else if(changeValues[3])incrementDecrementDouble(&settings.temp2, 1.0, 15.0, 100.0);
//   else if(changeValues[4])incrementDecrementDouble(&settings.time2, 1.0, 0.0, 90.0);
//   else if(changeValues[5])incrementDecrementDouble(&settings.temp3, 1.0, 15.0, 100.0);
//   else if(changeValues[6])incrementDecrementDouble(&settings.time3, 1.0, 0.0, 90.0);
//   else 
//     doPointerNavigation();
//   publishMessage();
//   }
// }

// //======================================================TOOLS - menu Internals==================================================
// void initMenuPage(String title, uint8_t itemCount){
//   //display1.clearDisplay();
//   display1.clearBuffer();
//   printPointer();
//   uint8_t fillCnt = (DISP_CHAR_WIDTH - title.length()) / 2;

//   btnOk.ClearWasDown();
 
//   itemCnt = itemCount;
//   flashCntr = 0;
//   flashIsOn = false;
//   updateAllItems = true;
//   loopStartMs = millis();
// }
// void captureButtonDownStates(){
//   btnOk.CaptureDownState();
// }

// void doPointerNavigation(){
//   currentTime = millis();
//   //Serial.print("pntrPos - dispOffset: ");
//   //Serial.println(pntrPos - dispOffset );
//   if (currentTime >= (loopTime + 1) ) {
//     aState = digitalRead(outputA); 
//    bState = digitalRead(outputB);
//   if (aState > aLastState)
//   {
//     if (bState != aState){  
//         if(pntrPos > 1)
//         {
//           flashIsOn = false; flashCntr = 0; 
//           if(pntrPos - dispOffset == 1){updateAllItems = true; dispOffset--;}
//           pntrPos--;
//           printPointer();
//           //Serial.print("Up: ");Serial.println(pntrPos);
//           //Serial.println("pntrPos - dispOffset ");
//           //Serial.println(pntrPos - dispOffset );
//         }
//         counter ++;
//       } 
//     else{
//         if(pntrPos < itemCnt)
//         {
//           flashIsOn = false; flashCntr = 0; 
//           if(pntrPos - dispOffset == DISP_ITEM_ROWS){updateAllItems = true; dispOffset++;}
//           pntrPos++;
//           printPointer();
//           //Serial.print("Down: ");Serial.println(pntrPos);
//           //Serial.println("pntrPos - dispOffset ");
//           //Serial.println(pntrPos - dispOffset );
//         }
//         counter --;
//     }
//    }
//    aLastState = aState; 
//    loopTime = currentTime;
//   }
  
// }

// void incrementDecrementDouble(double *v, double amount, double min, double max)
// {
//   int enc = encoder.readEncoder();
//   if(enc != 0) {
//     *v += (enc*amount);
//     *v = constrain(*v,min,max);
//     //Serial.println(enc*amount);
//     updateItemValue = true;
//   } 
//   delayMicroseconds(5);
// }

// bool isFlashChanged(){
//   if(flashCntr == 0){
//     flashIsOn = !flashIsOn;

//     flashCntr = FLASH_RST_CNT;

//     return true;
//   }
//   else{flashCntr--; return false;}
// }

// bool menuItemPrintable(uint8_t xPos, uint8_t yPos){
//   if(!(updateAllItems || (updateItemValue && pntrPos == yPos))){return false;}
//   uint8_t yMaxOffset = 0;
//   if(yPos > DISP_ITEM_ROWS) {yMaxOffset = yPos - DISP_ITEM_ROWS;}
//   if(dispOffset <= (yPos) && dispOffset >= yMaxOffset){display1.setCursor(CHAR_X*xPos, CHAR_Y*(yPos - dispOffset)); return true;}
//   return false;
// }

// bool menuItemPrintableDisp2(uint8_t xPos, uint8_t yPos){ 
//   if(!(updateAllItems || (updateItemValue && pntrPos == yPos))){return false;}
//   uint8_t yMaxOffset = 0;
//   if(yPos > DISP_ITEM_ROWS) {yMaxOffset = yPos - DISP_ITEM_ROWS;}
//   if(0 <= (yPos) && 0 >= yMaxOffset){display2.setCursor(CHAR_X*xPos, CHAR_Y*(yPos)); return true;}
//   return false;
// }

// //======================================================TOOLS_display========================================================
// void printPointer(){
//   //Serial.println("printPointer");
//   display1.drawStr(0, 1*CHAR_Y, " ");
//   display1.drawStr(0, 2*CHAR_Y, " ");
//   display1.drawStr(0, 3*CHAR_Y, " ");
//   display1.drawStr(0, 4*CHAR_Y, " ");
//   display1.drawStr(0, (pntrPos - dispOffset)*CHAR_Y, "*");
//   display1.sendBuffer();
// }
// void FlashPointer(){
//   display1.drawStr(0, 1*CHAR_Y, " ");
//   display1.drawStr(0, 2*CHAR_Y, " ");
//   display1.drawStr(0, 3*CHAR_Y, " ");
//   display1.drawStr(0, 4*CHAR_Y, " ");
//   display1.sendBuffer();

//   delay(50);
//   //Serial.println("FlashPointer");
//   display1.drawStr(0, (pntrPos - dispOffset)*CHAR_Y, "*");
//   display1.sendBuffer();
// }

// void printOnOff(bool val){
//   if(val){display1.print(F("ON    "));}
//   else   {display1.print(F("OFF   "));}
// }
// void printChars(uint8_t cnt, char c){
//   if(cnt > 0){
//     char cc[] = " "; cc[0] = c;
//     for(u_int8_t i = 1; i < cnt; i++){display1.print(cc);}
//   }
// }
// uint8_t getUint32_tCharCnt(uint32_t value)
// {
//   if(value == 0){return 1;}
//   uint32_t tensCalc = 10; int8_t cnt = 1;
//   while (tensCalc <= value && cnt < 20){tensCalc *= 10; cnt += 1;}
//   return cnt;
// }
// uint8_t getDoubleCharCnt(double value)
// {
//   if(value == 0){return 1;}
//   uint32_t tensCalc = 10; int8_t cnt = 1;
//   while (tensCalc <= value && cnt < 20){tensCalc *= 10; cnt += 1;}
//   return cnt;
// }
// void printUint32_tAtWidth(uint32_t value, uint8_t width, char c, boolean isRight){

//   uint8_t numChars = getUint32_tCharCnt(value);
//   if(isRight){printChars(width-numChars, ' ');}
//   display1.print(value);
//   display1.print(c);
//   if(!isRight){printChars(width-numChars, ' ');}
// }
// void printDoubleAtWidth(double value, uint8_t width, char c, boolean isRight){

//   uint8_t numChars = getDoubleCharCnt(value);
//   if(isRight){printChars(width-numChars, ' ');}
//   display1.print(value);
//   //display1.print(" ");
//   display1.print(c);
//   if(!isRight){printChars(width-numChars, ' ');}
// }
// //======================================================DISPLAY_2======================================================
// void printCharsDisplay2(uint8_t cnt, char c){
//   if(cnt > 0){
//     char cc[] = " "; cc[0] = c;
//     for(u_int8_t i = 1; i < cnt; i++){display2.print(cc);}
//   }
// }
// void printUint32_tAtWidthDisplay2(uint32_t value, uint8_t width, char c, boolean isRight){

//   uint8_t numChars = getUint32_tCharCnt(value);
//   if(isRight){printCharsDisplay2(width-numChars, ' ');}
//   display2.print(value);
//   display2.print(c);
//   if(!isRight){printCharsDisplay2(width-numChars, ' ');}
// }
// void printDoubleAtWidthDisplay2(double value, uint8_t width, char c, boolean isRight){

//   uint8_t numChars = getDoubleCharCnt(value);
//   if(isRight){printCharsDisplay2(width-numChars, ' ');}
//   display2.print(value);
//   //display1.print(" ");
//   display2.print(c);
//   if(!isRight){printCharsDisplay2(width-numChars, ' ');}
// }

// //======================================================TOOLS_settings======================================================
// void sets_SetDeafault()
// {
//   Mysettings tempSets;
//   memcpy(&settings, &tempSets, sizeof settings);
// }

// void sets_Load()
// {
//   EEPROM.get(0,settings);
//   if(settings.settingsCheckValue != SETTINGS_CHKVAL){sets_SetDeafault();}
// }
// void sets_Save()
// {
//   EEPROM.put(0, settings);
//   EEPROM.commit();
// }
// void updateSettings()
// {
//   if (current_thermostatTemp_DS18B20 > settings.maxElementTemp)
//   {
//     updateAllItems = true;
//     updateItemValue = true;
//     settings.power = false;
//     analogWrite(alarmPin, 100);
//   }
//   else analogWrite(alarmPin, LOW);

//   if(settings.power)
//   {
//     digitalWrite(ledOnPin, HIGH);
//     if (settings.manualMode == false)
//     {
//       previousPassedTimeS = passedTimeS;
//       passedTimeS = (millis() - tS) / (1000);

//       if (0 <= passedTimeS && passedTimeS < settings.time1 * 60) //60
//         settings.targetTemp = settings.temp1;
//       else if (settings.time1 * 60 <= passedTimeS && passedTimeS < (settings.time1 + settings.time2) * 60) //60
//         settings.targetTemp = settings.temp2;     
//       else if ((settings.time1 + settings.time2) * 60 <= passedTimeS && passedTimeS < (settings.time1 + settings.time2 + settings.time3) * 60) //60     
//         settings.targetTemp = settings.temp3;     
//       else
//         settings.targetTemp = 15;
      
//     }
//     else
//     {
//       tS = millis();
//       passedTimeS = 0;
//     }
//     //Serial.print("PID_mashTemp.Compute(): ");
//     //Serial.println(PID_mashTemp.Compute());
//     //Serial.println(settings.Kp_mash);
//     //Serial.println(PID_mashTemp.GetKp());
//     updateAllItems = PID_mashTemp.Compute() || PID_elementTemp.Compute();
//     previousDutyCycle = DutyCycle;
//     DutyCycle = Output_mashTemp; 
//     DutyCycle = constrain(DutyCycle, 0, 4096.0 * settings.dutycycleThreshold * 0.01);
//     ledcWrite(ledChannel, DutyCycle); 

    
//     settings.dutycycleThreshold = Output_elementTemp; 
//     //Serial.println(settings.dutycycleThreshold);
//     settings.dutycycleThreshold = constrain(settings.dutycycleThreshold, 0, 100); //100

//     targetElementTemp = settings.maxElementTemp - settings.marginalTemp;
//   }
//   else
//   {
//     digitalWrite(ledOnPin, LOW);
//     ledcWrite(ledChannel, 0);
//   }
  
// }
// void updateSensorValues()                        // långsamast i hela programet
// {
//     sensor1.requestTemperatures();
//     sensor2.requestTemperatures();

//     previous_temp_DS18B20 = current_temp_DS18B20;
//     previous_thermostatTemp_DS18B20 = current_thermostatTemp_DS18B20;

//     float RawValue1 = sensor1.getTempCByIndex(0);  // långsamast i hela programet
//     float RawValue2 = sensor2.getTempCByIndex(0);  // långsamast i hela programet

//     current_temp_DS18B20 = constrain((((RawValue1 - settings.RawLow) * ReferenceRange) / RawRange) + ReferenceLow, 0, 110);
//     current_thermostatTemp_DS18B20 = constrain((((RawValue2 - settings.RawLow) * ReferenceRange) / RawRange) + ReferenceLow, 0, 150);
// }
// void updateDisp2()
// {
//   display2.clearBuffer();
//   if(menuItemPrintableDisp2(1,1)){display2.print(F("mashTemp =      "));}
//   if(menuItemPrintableDisp2(1,2)){display2.print(F("DC       =      "));}
//   if(menuItemPrintableDisp2(1,3)){display2.print(F("eleTemp  =      "));}

//   if(menuItemPrintableDisp2(11,1)){printDoubleAtWidthDisplay2(current_temp_DS18B20, 4, 'C', false);}
//   if(menuItemPrintableDisp2(11,2)){printDoubleAtWidthDisplay2(DutyCycle/4096.0 * 100.0, 4, '%', false);}
//   if(menuItemPrintableDisp2(11,3)){printDoubleAtWidthDisplay2(current_thermostatTemp_DS18B20, 4, 'C', false);}
//   display2.sendBuffer();
// }

// double Pid(double error, float kp, float ki, float kd)
// {
//   double proportional = error;
//   integral += error * dt;
//   double derivative = (error - previous) / dt;
//   previous = error;
//   double output = (kp * proportional) + (ki * integral) + (kd * derivative);
//   return output;
// }

// void setupWiFi()
// {
//   Serial.print("\nConeccting to ");
//   Serial.print(privates.ssid);
//   WiFi.begin(privates.ssid, privates.pass);
//   while(WiFi.status() != WL_CONNECTED)
//   {
//     delay(100);
//     Serial.print(".");
//   }
//   Serial.print("\nConnected to ");
//   Serial.println(privates.ssid);
// }
// void reconnect()
// {
//   //Serial.print("\nConnecting to ");
//   //Serial.println(privates.broker);
//   if(client.connect("boll", privates.brokerUser, privates.brokerPass))
//   {
//     //Serial.print("\nConnected to ");
//     //Serial.println(privates.broker);
//   }
//   else
//   {
//     Serial.println("\nTrying connect again");
//   }
// }
// void publishMessage()
// {
//   if(!client.connected()){reconnect();}
//   client.loop();
  
//   if(currentTime - previousTime > 1000)
//   {
//     snprintf(messages, 10, "%ld", (int)current_temp_DS18B20);
//     client.publish(privates.outTopic, messages);
//     previousTime = millis();
//   }
// }