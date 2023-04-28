#include <Arduino.h>
#include "Sensors.h"

//Other Functions
void SelectShield(uint8_t);
//Other Functions

//Global variables
int last = millis(); //checks the time after each code run, for possible freezes
volatile uint8_t MCPShieldID,MCPShieldIDold;
volatile bool MCPShieldChange;
uint8_t NextionMessage;
//Global variables


void setup(){
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  
  Serial.begin(115200);//Serial for esp32
  Serial2.begin(9600);//Serial for NEXTION
  MCPinit();
}

void loop(){
  MCPloop();

  switch(MCPShieldID)
  {
    case 0:                   // no shield
                              MCPClearGPIOAndShields();
                              break;
    case POTmeterShieldID:    // Potentiometer
                              POTloop();
                              break;
    case BME280ShieldID:     // BME280
                              BME280loop();
                              break;
    case HW040ShieldID:      // HW040
                              HWloop();
                              break;
    case JOYShieldID:        // JOYSTICK
                              JOYloop();
                              break;
    case VL53L0XShieldID:  // VL53L0X
                              VL53L0Xloop();
                              break;
    case WS2812ShieldID:  // WS2812
                              WS2812loop();
                              break;      
    case HCSR04ShieldID:  // HCSR04
                              HCSR04loop();
                              break;
    case SG90ShieldID:  // SG90
                              SG90loop();
                              break;
    case MPU6050ShieldID:  // MPU6050
                              MPU6050loop();
                              break;                        
                              
  }


  // WDT
  if (millis() - last >= 2000) 
  {
    //Serial.println("Resetting WDT...");
    esp_task_wdt_reset();
    last = millis();
  }
}

void MCPinit(){
  byte error;
  //First step
  Wire.begin();

  //Second step
  pinMode(MCPpin, INPUT_PULLUP);

  //Third step, Shield ID setup
  MCPShieldIDold = 255;
  MCPShieldID = 0;
  MCPShieldChange = false;

  // ----- configure the GPINTENA -----
  Wire.beginTransmission(MCPI2CAddress);
  Wire.write(MCPGpintenaAddress);
  Wire.write(0xFF);
  error = Wire.endTransmission();
  if(error){
    return;
  }
  // 0 = success
  // 1 = error: buffer too short
  // 2 = error: address send, nack received
  // 3 = error: data send, nack received
  // 4 = error: other twi error
  // 5 = error: timeout

  // ------ configure the GPPUA -------
  Wire.beginTransmission(MCPI2CAddress);
  Wire.write(MCPGppuaAddress);
  Wire.write(0xFF);
  error = Wire.endTransmission();
  if(error){
    return;
  }
  MCPReadShieldID();
  Serial2.printf("page homepage");
  MCPNextionTXEnd();
  return;
}

void MCPloop(){
  if(!digitalRead(MCPpin)){
      MCPReadShieldID();
      if(MCPShieldID != MCPShieldIDold){
        MCPShieldIDold = MCPShieldID;
        MCPShieldChange = true;
      }
  }
  return;
}

//this function will read the current shield id many times, until it is certain that a shield is plugged in
void MCPReadShieldID(void){
  uint8_t LastRead , CurrentRead, ReadCounter, error;

  for(LastRead=255, ReadCounter=0; ReadCounter < 6; ReadCounter ++){
    Wire.beginTransmission(MCPI2CAddress);
    Wire.write(MCPGpioaAddress);
    error = Wire.endTransmission();

    if(error){
      MCPShieldID = 0;
      return;
    }

    Wire.requestFrom(MCPI2CAddress, 1);
    if(error){
      MCPShieldID = 0;
      return;
    }

    delay(50);

    if(Wire.available()){
      CurrentRead = (255 - (uint8_t)Wire.read());
      if(CurrentRead != LastRead){
        if(CurrentRead == 0){
          MCPShieldID = CurrentRead; //no shield detected, shieldID = 0
          return;
        }
        LastRead = CurrentRead;
        ReadCounter = 0;
      }
    }
    else{
      MCPShieldID = 0; //No shield detected
      return;
    }
  }

  MCPShieldID = LastRead;
  return;
}

void MCPNextionTXEnd(){
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  return;
}

void MCPClearGPIOAndShields(void)  // used to reset all GPIOs to INPUT state, used each time a no shield state is detected
{
  pinMode(32,INPUT);  //ADC1
  pinMode(33,INPUT);  //ADC1
  pinMode(25,INPUT);  //ADC1
  pinMode(26,INPUT);  //ADC2
  pinMode(5,INPUT);
  pinMode(18,INPUT);
  pinMode(19,INPUT);
  pinMode(23,INPUT);

  // DeInit BME280
  HWShieldDeinit();
  BME280ShieldDeInit();   //ID=14
  POTShieldDeinit();
  JOYShieldDeinit();
  VL53L0XShieldDeinit();
  WS2812ShieldDeinit();
  HCSR04ShieldDeinit();
  SG90ShieldDeinit();
  MPU6050ShieldDeinit();

  return;
}
//=======================================================================================
//=======================================================================================
//=======================================================================================


//=======================================================================================
//=======================================================================================
//=======================================================================================
void BME280loop(){
  if(!BME280Data.bShieldInit) BME280ShieldInit();
  if(!BME280Data.bSensorInit) BME280SensorInit();
  if(BME280Data.bSensorFound) BME280SensorRead();

  while(Serial2.available()){
    NextionMessage = Serial2.read();
  } //Used to read current page, read until buffer is empty
  if(NextionMessage == 2) BME280Data.ui8CurrentPage = NextionMessage;
  else if(NextionMessage == 3) BME280Data.ui8CurrentPage = NextionMessage;

  BME280GetData();
  return;
}

void BME280ShieldInit(){
  Serial2.print("page BMP0");
  MCPNextionTXEnd();
  delay(20);

  BME280Data.bShieldInit = true;
  return;
}

void BME280ShieldDeInit(void){
  if(!BME280Data.bShieldInit) return;

  BME280Data.bShieldInit = false;
  BME280SensorDeinit();
  
  Serial2.print("page homepage");
  MCPNextionTXEnd();
  delay(20);

  return;
}

void BME280SensorInit(void){
  if(BME280Data.bme.begin(BME280I2CAddress) && !BME280Data.bSensorInit)  // My BME sensor's address is 0x77
  {
    BME280Data.bme.begin(BME280I2CAddress);
    BME280Data.i16Altitude[200] = {0};
    BME280Data.ui16Humidity[200] = {0};
    BME280Data.ui16Pressure[200] = {0};
    BME280Data.ui16Temperature[200] = {0};
    BME280Data.uiArrayIndex = 1;

    BME280Data.bSensorFound = true;
    BME280Data.bSensorInit = true;
  }
  return;
}

void BME280SensorDeinit(void){
    Serial2.print("page BMP0");
    MCPNextionTXEnd();
    delay(20);

    BME280Data.bSensorFound = false;
    BME280Data.bSensorInit = false;

    BME280Data.i16Altitude[200] = {0};
    BME280Data.ui16Humidity[200] = {0};
    BME280Data.ui16Pressure[200] = {0};
    BME280Data.ui16Temperature[200] = {0};
    BME280Data.uiArrayIndex = 1;
  return;
}

void BME280SensorRead(){
  if(BME280Data.uiArrayIndex > 199){
    BME280Data.i16Altitude[200] = {0};
    BME280Data.ui16Humidity[200] = {0};
    BME280Data.ui16Pressure[200] = {0};
    BME280Data.ui16Temperature[200] = {0};
    BME280Data.uiArrayIndex = 1;
  }

  BME280Data.ui16Humidity[0] = (uint16_t)(BME280Data.bme.readHumidity()*10); //Measure
  BME280Data.ui16Temperature[0] = (uint16_t)(BME280Data.bme.readTemperature()*100);
  BME280Data.i16Altitude[0] = (uint16_t)(BME280Data.bme.readAltitude(BME280_SEA_LEVEL_PRESSURE_HPA));
  BME280Data.ui16Pressure[0] = (uint16_t)(BME280Data.bme.readPressure()/100);

  if( (BME280Data.ui16Temperature[0] > 15000) || (BME280Data.ui16Pressure[0] > 60000) || (BME280Data.i16Altitude[0] > 60000) )
  { // BME 280 Error
    BME280SensorDeinit();
    return;
  }

  BME280Data.ui16Humidity[BME280Data.uiArrayIndex] = BME280Data.ui16Humidity[0]; //Put the measurement into an array
  BME280Data.ui16Temperature[BME280Data.uiArrayIndex] = BME280Data.ui16Temperature[0]; 
  BME280Data.i16Altitude[BME280Data.uiArrayIndex] = BME280Data.i16Altitude[0];
  BME280Data.ui16Pressure[BME280Data.uiArrayIndex] = BME280Data.ui16Pressure[0];
  BME280Data.uiArrayIndex += 1;


  BME280UpdateDisplay();
  return;
}

void BME280UpdateDisplay(){

  if(BME280Data.ui8CurrentPage == 2){
    char chTemp[20];
    double dTemp;
    uint8_t ui8Temp;

    //update temperature
    dTemp = (double)BME280Data.ui16Temperature[0];
    dTemp /= 100.0;
    sprintf(chTemp,"%0.2lf C",dTemp);
    Serial2.printf("t1.txt=\"Temperature:%s\"",chTemp);
    MCPNextionTXEnd();
    delay(25);

    //update Humidity
    dTemp = (double)BME280Data.ui16Humidity[0];
    dTemp /= 10;
    sprintf(chTemp,"%0.1lf %%",dTemp);
    Serial2.printf("t2.txt=\"Humidity:%s\"",chTemp);
    MCPNextionTXEnd();
    delay(25);

    //update Pressure
    sprintf(chTemp,"%u hPa",BME280Data.ui16Pressure[0]);
    Serial2.printf("t4.txt=\"Pressure:%s\"",chTemp);
    MCPNextionTXEnd();
    delay(25);

    //update Altitude
    sprintf(chTemp,"%u m",BME280Data.i16Altitude[0]);
    Serial2.printf("t5.txt=\"Altitude:%s\"",chTemp);
    MCPNextionTXEnd();
    delay(25);
    return;
  }

  if(BME280Data.ui8CurrentPage == 3){
    // Update Waveform
    // Temperature
    char chTemp[20];
    double dTemp;
    uint16_t ui16Temp;
    //================================================
    ui16Temp = (BME280Data.ui16Temperature[0]/100);
    if(ui16Temp > 100) ui16Temp = 100;
    Serial2.printf("add 3,0,%u",ui16Temp);
    MCPNextionTXEnd();
    delay(10);

    // Humidity
    ui16Temp = (BME280Data.ui16Humidity[0]/10);
    if(ui16Temp > 100) ui16Temp = 100;
    Serial2.printf("add 3,1,%u",ui16Temp);
    MCPNextionTXEnd();
    delay(10);
  }
  return;
}

void BME280GetData(){
  char chTemp;
  char chTempArray[30];
  double dTemp;

  if(Serial.available()){
    chTemp = Serial.read();
    if(chTemp == 'T'){ //T for TEST
      Serial.print("DATA");
      for(int i = BME280_MAX_ARRAY; i > 0; i--){
        Serial.print("\n");
        dTemp = (double)BME280Data.ui16Humidity[i];
        dTemp /= 10;
        sprintf(chTempArray,"%0.1lf",dTemp);
        Serial.printf("%s\n",chTempArray);
        dTemp = (double)BME280Data.ui16Temperature[0]; //Writing data out to the Serial monitor
        dTemp /= 100.0;
        sprintf(chTempArray,"%0.2lf",dTemp);
        Serial.printf("%s\n",chTempArray);
        sprintf(chTempArray,"%u",BME280Data.i16Altitude[i]);
        Serial.printf("%s\n",chTempArray);
        sprintf(chTempArray,"%u",BME280Data.ui16Pressure[i]);
        Serial.printf("%s\n",chTempArray);
      }
      BME280Data.i16Altitude[200] = {0}; //Reset the measurements after taking them out 
      BME280Data.ui16Humidity[200] = {0};
      BME280Data.ui16Pressure[200] = {0};
      BME280Data.ui16Temperature[200] = {0};
      BME280Data.uiArrayIndex = 1; //Reset indexing of the array
    }
  }
  return;
}
//BME280 SENSOR==============================================================
//=======================================================================================
//=======================================================================================
//=======================================================================================


//=======================================================================================
//=======================================================================================
//POTMETER SENSOR==============================================================
void POTloop(void){
  if(!POTData.bPShieldInit) POTShieldInit();
  if(!POTData.bPSensorInit) POTSensorInit();
  if(POTData.bPSensorFound) POTSensorRead();
  POTGetData();
}

void POTShieldInit(){
  POTData.ui8CurrentPage = 1;
  Serial2.print("page POT0");
  MCPNextionTXEnd();
  delay(5);

  POTData.bPSensorFound = false;
  POTData.bPSensorInit = false;
  POTData.bPShieldInit = true;

  return;
}

void POTSensorInit(){
  pinMode(32, INPUT);
  POTData.bPSensorFound = true;
  POTData.bPSensorInit = true;
  POTData.ui16Voltage = 0;
  return;
}

void POTSensorRead(){
  POTData.ui16Voltage = analogRead(32);

  while(Serial2.available()){
    NextionMessage = Serial2.read();
  }
  if(NextionMessage == 4) POTData.ui8CurrentPage = NextionMessage;
  if(NextionMessage == 5) POTData.ui8CurrentPage = NextionMessage;

  POTUpdateDisplay();
  return;
}

void POTUpdateDisplay(){
  if(POTData.ui8CurrentPage == 5){
    uint16_t ui16Temp;

    // update Voltage
    ui16Temp = (uint16_t)map(POTData.ui16Voltage,0,4095,0,3300);
    Serial2.printf("t0.txt=\"Voltage:%03u[mV]\"",ui16Temp);
    MCPNextionTXEnd();
    delay(5);

    // update Waveform
    ui16Temp = (uint16_t)map(POTData.ui16Voltage,0,4095,0,100);
    Serial2.printf("add 5,0,%u",ui16Temp);
    MCPNextionTXEnd();
    delay(5);
  }
  return;
}

void POTShieldDeinit(){
  if(!POTData.bPShieldInit) return;

  POTData.bPShieldInit = false;
  POTSensorDeinit();
  
  Serial2.print("page homepage");
  MCPNextionTXEnd();
  delay(10);
  return;
}

void POTSensorDeinit(){
  POTData.ui16Voltage= 0;
  POTData.bPSensorFound = false;
  POTData.bPSensorInit = false;
  return;
}

void POTGetData(){

}
//=======================================================================================
//=======================================================================================
//=======================================================================================


//=======================================================================================
//=======================================================================================
//=======================================================================================
//HWSENSOR GLOBAL VARIABLE
volatile boolean vbTurnDetected;
volatile boolean vbup;
static long slvirtualPosition=0; 

void HWloop(){
  if(!HWData.bHWShieldInit) HWShieldInit();
  if(!HWData.bHWSensorInit) HWSensorInit();
  HWSensorRead();

  return;
}

void HWShieldInit(){
  if(HWData.bHWShieldInit) return;
  HWData.bHWShieldInit = true;
  Serial2.print("page HW0");
  MCPNextionTXEnd();
  delay(20);
  return;
}

//isr interrupt for button press
void HWSensorInit(){
  HWData.ui8CurrentPage = 6;
  HWData.i16Position = 0;
  HWData.ui8PressedButton = 0;
  HWData.ScurrentDirection = "No Rotation";
  
  HWData.bHWSensorInit = true;
  HWData.bHWSensorFound = true;

  pinMode(HWCLK, INPUT);
  pinMode(HWDT, INPUT);
  pinMode(HWSW , INPUT_PULLUP);
  attachInterrupt (HWCLK,POTisr,CHANGE);
  delay(50);
  //Serial2.begin(9600);

  return;
}

void HWShieldDeinit(){
  if(!HWData.bHWShieldInit) return;
  HWData.bHWShieldInit = false;
  detachInterrupt(HWCLK);
  HWSensorDeinit();
  
  Serial2.print("page homepage");
  MCPNextionTXEnd();
  return;
}

void HWSensorDeinit(){
  HWData.ui8CurrentPage = 0;
  HWData.i16Position = 0;
  HWData.ui8PressedButton = 0;
  HWData.ScurrentDirection = "";
  
  HWData.bHWSensorInit = false;
  HWData.bHWSensorFound = false;

  //HWSENSOR GLOBAL VARIABLE
}

void POTisr ()  {                    // Interrupt service routine is executed when any CHANGE transition is detected on CLK
    volatile boolean CLK = digitalRead(HWCLK);
    volatile boolean DT = digitalRead(HWDT);
    vbup=((!CLK && DT)||(CLK && !DT));
   
    vbTurnDetected = true;
}


void HWSensorRead(){

  if (vbTurnDetected)  {       // do this only if rotation was detected
    if (vbup){
      slvirtualPosition++;
      HWData.ScurrentDirection = "Rotation:CW";
    }
    else{
     slvirtualPosition--;
     HWData.ScurrentDirection = "Rotation:CCW";
    }
    vbTurnDetected = false;          // do NOT repeat IF loop until new rotation detected
  }
  
  HWData.i16Position = (int16_t)slvirtualPosition;
  HWData.ui8PressedButton = digitalRead(HWSW);

  if(Serial2.available()){
    NextionMessage = Serial2.read();
    if(NextionMessage == 7) HWData.ui8CurrentPage = NextionMessage;
  }

  HWUpdateDisplay();
  return;
}

void HWUpdateDisplay(){
  if(HWData.ui8CurrentPage != 7) return;

  Serial2.printf("n0.val=%d",HWData.i16Position);
  MCPNextionTXEnd();
  delay(5);

  Serial2.printf("t0.txt=\"%s\"", HWData.ScurrentDirection);
  MCPNextionTXEnd();
  delay(20);

  if(HWData.ui8PressedButton == 0){
    Serial2.printf("t1.txt=\"Button:Pressed\"");
    MCPNextionTXEnd();
    delay(20);
  }
  else{
    Serial2.printf("t1.txt=\"Button:Not pressed\"");
    MCPNextionTXEnd();
    delay(20);
  }
  return;
}
//=======================================================================================
//=======================================================================================
//=======================================================================================


//=======================================================================================
//=======================================================================================
//=======================================================================================
//Joystick
void JOYShieldInit(void)            // called if shield was inserted
{
  // -TODO => Set the proper page on the LCD Screen
  Serial2.print("page JOYSTICK0");
  MCPNextionTXEnd();
  delay(5);
  //-------------------------------------

  // 5pin  <=> VCC=3.3V, GND
  // ADC_X  = GPIO32=ADC1_4
  // ADC_Y  = GPIO33=ADC1_5
  // SWITCH = GPIO25
  pinMode(32,INPUT);
  pinMode(33,INPUT);
  pinMode(25,INPUT_PULLUP);

  JOYData.bNewDataIsAvailable = false;
  JOYData.ui8GPIOPinXNo = 32;
  JOYData.ui8GPIOPinYNo = 33;
  JOYData.ui8GPIOPinSWNo = 25;
  JOYData.ui16ADCValueX = 0;
  JOYData.ui16ADCValueY = 0;
  JOYData.bSwitchValue = false;
  JOYData.ui16ADCValueXOld = 5000;
  JOYData.ui16ADCValueYOld = 5000;
  JOYData.bSwitchValueOld = true;
  JOYData.ulLastReadTimeMS = millis();
  JOYData.ulLastReadTimeMSInterval = JOY_READ_INTERVAL_MS;

  JOYData.bShieldInit = true;
  NextionMessage = 8;
  return;
}

void JOYloop(void)            // called from the loop() as long as the shield is present
{
  unsigned long ulTemp, ulResult;
  ulTemp = millis();
  if(ulTemp > JOYData.ulLastReadTimeMS)       ulResult = ulTemp - JOYData.ulLastReadTimeMS;
  else                                          ulResult = JOYData.ulLastReadTimeMS - ulTemp;
  if(ulResult > JOYData.ulLastReadTimeMSInterval){
    JOYData.ulLastReadTimeMS = ulTemp;
    // check if the shield was init
    if(!JOYData.bShieldInit){
      JOYShieldInit();
      return;
    }
    // take another measurement
    JOYReadData();
  }
  return;
}

void JOYReadData(void)             // used to read a new distance from the TOF sensor
{
  // X Value
  JOYData.ui16ADCValueX = analogRead(JOYData.ui8GPIOPinXNo);

  // Y Value
  JOYData.ui16ADCValueY = analogRead(JOYData.ui8GPIOPinYNo);

  // Switch
  JOYData.bSwitchValue = !digitalRead(JOYData.ui8GPIOPinSWNo);

  // Update the LCD
  while(Serial2.available()){
    NextionMessage = Serial2.read();
  }
  if(NextionMessage == 8) JOYData.ui8CurrentPage = 8;
  if(NextionMessage == 9) JOYData.ui8CurrentPage = 9;

  if(JOYData.ui8CurrentPage == 9){
    JOYUpdateDisplay();
  }

  return;
}

void JOYUpdateDisplay(void)            // used to update the values on the LCD after a new measurement
{
  uint16_t ui16Temp;

  // update X
  if(JOYData.ui16ADCValueXOld != JOYData.ui16ADCValueX)
  {
    JOYData.ui16ADCValueXOld = JOYData.ui16ADCValueX;

    ui16Temp = (uint16_t)map(JOYData.ui16ADCValueX,0,4095,0,100);
    Serial2.printf("t0.txt=\"Y=%02u [%%]\"",ui16Temp); //X is the real value, Y is printed for debugging
    MCPNextionTXEnd();
    delay(10);
    Serial2.printf("h1.val=%u",ui16Temp);
    MCPNextionTXEnd();
    delay(10);
  }
  
  // update Y
  if(JOYData.ui16ADCValueYOld != JOYData.ui16ADCValueY)
  {
    JOYData.ui16ADCValueYOld = JOYData.ui16ADCValueY;

    ui16Temp = (uint16_t)map(JOYData.ui16ADCValueY,0,4095,0,100);
    Serial2.printf("t1.txt=\"X=%02u [%%]\"",ui16Temp); //Y Value but on nextion appears as X, depends on the joystick placement
    MCPNextionTXEnd();
    delay(10);
    Serial2.printf("h0.val=%u",ui16Temp);
    MCPNextionTXEnd();
    delay(10);
    
  }

  // update Switch
  if(JOYData.bSwitchValueOld != JOYData.bSwitchValue)
  {
    JOYData.bSwitchValueOld = JOYData.bSwitchValue;

    if(JOYData.bSwitchValue)  Serial2.print("t2.txt=\"Button: Pressed\"");
    else                        Serial2.print("t2.txt=\"Button: Not Pressed\"");
    MCPNextionTXEnd();
    delay(10);
  }
  return;
}

void JOYShieldDeinit(void)          // called in case shield was removed, called by the system function "MCPClearGPIOAndShields()"
{
  if(!JOYData.bShieldInit) return;   // already de-initialized
  JOYData.bShieldInit = false;

  pinMode(25,INPUT);

  JOYData.bNewDataIsAvailable = false;
  JOYData.ui16ADCValueX = 0;
  JOYData.ui16ADCValueY = 0;
  JOYData.bSwitchValue = false;
  JOYData.ulLastReadTimeMS = millis();
  JOYData.ulLastReadTimeMSInterval = JOY_READ_INTERVAL_MS;

  // LCD
  Serial2.print("page homepage");
  MCPNextionTXEnd();

  return;
}

//=======================================================================================
//=======================================================================================
//=======================================================================================


//=======================================================================================
//=======================================================================================
//=======================================================================================
//VL53L0X
void VL53L0XShieldInit(void)            // called if shield was inserted
{
  if(VL53L0XData.bShieldInit) return;
  // -TODO => Set the proper page on the LCD Screen
  Serial2.print("page VL53LOX0");
  MCPNextionTXEnd();
  delay(10);

  VL53L0XData.ui8DistanceHistoryIndex = 0;
  for(uint8_t i=0;i<VL53L0X_HISTORY_DEPTH;i++) VL53L0XData.ui16DistanceMMHistory[i] = i;  // populate with non-identical data

  VL53L0XData.bSensorFound = false;
  VL53L0XData.bShieldInit = true;

  return;
}

void VL53L0XSensorInit(void)           // called to initialize the VL53L0X sensor
{
  if(VL53L0XData.bSensorFound) return; // sensor already found
  
  if(!VL53L0XData.bSensorInit){
    VL53L0XData.ulLastReadTimeMS = millis();
    VL53L0XData.ulLastReadTimeMSInterval = VL53L0X_READ_INTERVAL_MS;
    VL53L0XData.ui16DistanceMM[200] = {0};
    VL53L0XData.bNewDataIsAvailable = false;
    VL53L0XData.ui8ErrorCode = 0;
    VL53L0XData.bSensorInit = true;
    VL53L0XData.uiArrayIndex = 1;
  }
  
  if(VL53L0XData.lox.begin())  // set if different than 41U
  { //sensor seems to be ok
    //--TODO => print the erro on the LCD Screen
    Serial2.print("t1.txt=\"Sensor: Detected\"");
    MCPNextionTXEnd();
    delay(10);
    Serial2.print("t1.pco=GREEN");
    MCPNextionTXEnd();
    delay(10);
    //------------------------------------------
    VL53L0XData.ui8ErrorCode = 0;
    VL53L0XData.bSensorFound = true;
    //VL53L0XData.lox.startRangeContinuous();

  }
  return;
}

void VL53L0XShieldDeinit(void)// called in case shield was removed, called by the system function "MCPClearGPIOAndShields()"
{
  if(!VL53L0XData.bShieldInit) return; // nothing to do
  VL53L0XData.bShieldInit = false;
  VL53L0XSensorDeinit();

  // LCD
  Serial2.print("page homepage");
  MCPNextionTXEnd();
  delay(10);
  return;
}

void VL53L0XSensorDeinit(void)         // called if a sensor removal was detected
{
  if(!VL53L0XData.bSensorFound) return; //the sensor is missing for already some time ...
  VL53L0XData.bSensorFound = false;
  VL53L0XData.bSensorInit = false;

  if(VL53L0XData.bSensorInit){
    VL53L0XData.bNewDataIsAvailable = false;
    VL53L0XData.ui16DistanceMM[200] = {0};
    VL53L0XData.ulLastReadTimeMS = millis();
    VL53L0XData.ulLastReadTimeMSInterval = VL53L0X_READ_INTERVAL_MS;
    VL53L0XData.bSensorInit = false;
    VL53L0XData.uiArrayIndex = 1;
  }

  //--TODO => print the erro on the LCD Screen
  Serial2.print("t1.txt=\"Sensor: Not Detected\"");
  MCPNextionTXEnd();
  delay(10);
  Serial2.print("t1.pco=RED");
  MCPNextionTXEnd();
  delay(10);

  return;
}

void VL53L0Xloop(void)                 // called from the loop() as long as the shield is present
{
  unsigned long ulTemp, ulResult;
  ulTemp = millis();
  if(ulTemp > VL53L0XData.ulLastReadTimeMS)   ulResult = ulTemp - VL53L0XData.ulLastReadTimeMS;
  else                                        ulResult = VL53L0XData.ulLastReadTimeMS - ulTemp;

  if(ulResult > VL53L0XData.ulLastReadTimeMSInterval)
  {
    VL53L0XData.ulLastReadTimeMS = ulTemp;

    // check if the shield was init
    if(!VL53L0XData.bShieldInit)
    {
      VL53L0XShieldInit();
      return;
    }

    // check if the sensor was found
    if(!VL53L0XData.bSensorFound)
    {
      VL53L0XSensorInit();
      return;
    }

    // take another measurement
    VL53L0XReadDistance();
  }
  return;
}


void VL53L0XReadDistance(void)         // used to read a new distance from the TOF sensor
{
  VL53L0X_RangingMeasurementData_t measure;
  
  VL53L0XData.lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  VL53L0XData.ui8ErrorCode = measure.RangeStatus;
  VL53L0XData.ui16DistanceMM[0] = measure.RangeMilliMeter;
  VL53L0XData.ui16DistanceMM[VL53L0XData.uiArrayIndex] = VL53L0XData.ui16DistanceMM[0];
  VL53L0XData.uiArrayIndex += 1;

  if(VL53L0XData.uiArrayIndex > 199){
    VL53L0XData.ui16DistanceMM[200] = {0};
    VL53L0XData.uiArrayIndex = 1;
  }

  VL53L0XData.bNewDataIsAvailable = true;

  if( (VL53L0XData.ui8ErrorCode > 4) || (VL53L0XData.ui16DistanceMM[0] > 32000) )
  { // sensor error
    VL53L0XSensorDeinit();
  }

  //detect if the sensor is "stucked" on the same value
  if(VL53L0XData.ui8DistanceHistoryIndex < (VL53L0X_HISTORY_DEPTH-1) )  VL53L0XData.ui8DistanceHistoryIndex ++;
  else                                                                  VL53L0XData.ui8DistanceHistoryIndex = 0;
  VL53L0XData.ui16DistanceMMHistory[VL53L0XData.ui8DistanceHistoryIndex] = VL53L0XData.ui16DistanceMM[0];
  uint8_t i,j;
  for(i=0, j=0; i<(VL53L0X_HISTORY_DEPTH-1); i++)
  {
    if(VL53L0XData.ui16DistanceMMHistory[i] != VL53L0XData.ui16DistanceMMHistory[i+1])  j=1;  // difference detected
  }
  if(j == 0)
  { // no difference was found => most likely the sensor is stucked => force a re-initialize
    VL53L0XSensorDeinit();  // for some reason it seems to stuck in the 0 val some time...
  }

  VL53L0XGetData();
  // LCD
  while(Serial2.available()){
    NextionMessage = Serial2.read();
  }
  if(NextionMessage == 10) VL53L0XData.ui8CurrentPage = 10;
  if(NextionMessage == 11) VL53L0XData.ui8CurrentPage = 11;

  if(VL53L0XData.ui8CurrentPage == 11) VL53L0XUpdateDisplay();
  return;
}

void VL53L0XUpdateDisplay(void)           // used to update the values on the LCD after a new measurement
{
  if(VL53L0XData.ui8ErrorCode == 4)       Serial2.print(F("t4.txt=\"OVF.\""));
  else if(VL53L0XData.ui8ErrorCode > 4)   Serial2.print(F("t4.txt=\"???\""));
  MCPNextionTXEnd();
  delay(10);

  // update sensor Waveform
  if(VL53L0XData.ui8ErrorCode < 4)
  {
    Serial2.printf("t4.txt=\"Distance=%4u mm\"",VL53L0XData.ui16DistanceMM[0]);
    MCPNextionTXEnd();
    delay(10);

    uint16_t ui16TempOut, ui16TempIn;

    ui16TempIn = VL53L0XData.ui16DistanceMM[0];
    if(ui16TempIn > 1300) ui16TempIn = 1300;

    ui16TempOut = (uint16_t)map(ui16TempIn,0,1300,0,100);
    Serial2.printf("add 4,0,%u",ui16TempOut);
    MCPNextionTXEnd();
    delay(10);
  }

  return;
}

void VL53L0XGetData(){
  char chTemp;
  char chTempArray[30];
  double dTemp;

  if(Serial.available()){
    chTemp = Serial.read();
    if(chTemp == 'T'){ //T for TEST
      Serial.print("\nDATA");
      for(int i = BME280_MAX_ARRAY; i > 0; i--){
        Serial.print("\n");
        Serial.printf("%4u",VL53L0XData.ui16DistanceMM[i]);
      }

      VL53L0XData.ui16DistanceMM[200] = {0};
      VL53L0XData.uiArrayIndex = 1;
    }
  }
  return;
}
//=======================================================================================
//=======================================================================================
//=======================================================================================



//=======================================================================================
//=======================================================================================
//=======================================================================================
//WS2812

void WS2812ShieldInit(void)            // called if shield was inserted
{
  if(WS2812Data.bShieldInit) return; // already initialized
  
  // -TODO => Set the proper page on the LCD Screen
  Serial2.print("page WS28120");
  MCPNextionTXEnd();
  delay(5);
  //-------------------------------------

  // Data From LCD
  WS2812Data.ui8LCDInterrogationIndex = 0;
  WS2812Data.bValueFromLCDExpected = false;
  // No of active LEDs
  WS2812Data.ui8NoOfActiveLEDsOld = 0;  // to force an update
  WS2812Data.ui8NoOfActiveLEDs = 8;     // default value
  // Start Values
  WS2812Data.ui8StartRGBOld[0] = 0;     // R
  WS2812Data.ui8StartRGBOld[1] = 0;     // G
  WS2812Data.ui8StartRGBOld[2] = 0;     // B
  WS2812Data.ui8StartRGB[0] = 128;      // R
  WS2812Data.ui8StartRGB[1] = 128;      // G
  WS2812Data.ui8StartRGB[2] = 128;      // B
  // End Values
  WS2812Data.ui8EndRGBOld[0] = 0;       // R
  WS2812Data.ui8EndRGBOld[1] = 0;       // G
  WS2812Data.ui8EndRGBOld[2] = 0;       // B
  WS2812Data.ui8EndRGB[0] = 128;        // R
  WS2812Data.ui8EndRGB[1] = 128;        // G
  WS2812Data.ui8EndRGB[2] = 128;        // B
  // Last update
  WS2812Data.ulLastReadTimeMS = millis();
  WS2812Data.ulLastReadTimeMSInterval = WS2812_UPDATE_INTERVAL_MS;
  WS2812Data.ulUpdateIfNoChangeMS = WS2812Data.ulLastReadTimeMS;

  WS2812strip.begin();
  WS2812strip.setBrightness(50);
  //WS2812strip.show(); // Initialize all pixels to 'off'

  WS2812Data.bShieldInit = true;

  #ifdef WS2812_USE_SERIAL_DEBUG
    Serial.println("WS2812 Shield detected! ID=5");
  #endif
  
  return;
}

void WS2812ShieldDeinit(void)          // called in case shield was removed, called by the system function "MCPClearGPIOAndShields()"
{
  if(!WS2812Data.bShieldInit) return; // already De-initialized
  WS2812Data.bShieldInit = false;
  
  WS2812Data.ulLastReadTimeMS = millis();
  WS2812Data.ulLastReadTimeMSInterval = WS2812_UPDATE_INTERVAL_MS;

  // LCD
  Serial2.print("page homepage");
  MCPNextionTXEnd();
  delay(10);
  return;
}

void WS2812loop(void)            // called from the loop() as long as the shield is present
{
  // handle sensor communication
  WS2812HandleSensor();

  // update the WS2812 content
  WS2812WriteData();

  return;
}

void WS2812HandleSensor(void)         // called from the sensor loop function to handle connection, initialization and data accumulation from the sensor
{
  unsigned long ulTemp, ulResult;
  ulTemp = millis();
  if(ulTemp > WS2812Data.ulLastReadTimeMS)        ulResult = ulTemp - WS2812Data.ulLastReadTimeMS;
  else                                              ulResult = WS2812Data.ulLastReadTimeMS - ulTemp;
  if(ulResult > WS2812Data.ulLastReadTimeMSInterval)
  {
    WS2812Data.ulLastReadTimeMS = ulTemp;

    // check if the shield was init
    if(!WS2812Data.bShieldInit)
    {
      WS2812ShieldInit();
      return;
    }

    // get data from the LCD (if needed)
    WS2812CollectDataFromLCD();
  }
  return;
}

void WS2812WriteData(void)             // used to read a new distance from the TOF sensor
{
  bool bNewData = false;

  if( WS2812Data.ui8NoOfActiveLEDsOld != WS2812Data.ui8NoOfActiveLEDs )   bNewData = true;
  else
  {
    for(uint8_t i=0; i<3; i++)
    {
      if(WS2812Data.ui8StartRGBOld[i] != WS2812Data.ui8StartRGB[i])
      {
        bNewData = true;
        break;
      }

      if(WS2812Data.ui8EndRGBOld[i] != WS2812Data.ui8EndRGB[i])
      {
        bNewData = true;
        break;
      }
    }
  }

  // update from time to time even if there is no change
  if( (abs((long)(WS2812Data.ulUpdateIfNoChangeMS - millis())) > WS2812_UPDATE_WITH_NO_CHANGE) )  bNewData = true;

  // there is change on the data => must update the WS2812
  if(bNewData)
  {
    WS2812Data.ulUpdateIfNoChangeMS = millis();

    WS2812Data.ui8NoOfActiveLEDsOld = WS2812Data.ui8NoOfActiveLEDs;
    for(uint8_t i=0; i<3; i++)
    {
      WS2812Data.ui8StartRGBOld[i] = WS2812Data.ui8StartRGB[i];
      WS2812Data.ui8EndRGBOld[i] = WS2812Data.ui8EndRGB[i];
    }

    // WS2812
    int iR, iG, iB;
    uint8_t ui8i;

    // empty strip
    for(ui8i=0; ui8i<20; ui8i++) WS2812strip.setPixelColor(ui8i, WS2812strip.Color(0,0,0));

    if(WS2812Data.ui8NoOfActiveLEDs > 2)
    { // more than 2 LEDs
      // calculate rates
      iR = (int)( ((WS2812Data.ui8EndRGB[0] - WS2812Data.ui8StartRGB[0])/(WS2812Data.ui8NoOfActiveLEDs - 1)) );
      iG = (int)( ((WS2812Data.ui8EndRGB[1] - WS2812Data.ui8StartRGB[1])/(WS2812Data.ui8NoOfActiveLEDs - 1)) );
      iB = (int)( ((WS2812Data.ui8EndRGB[2] - WS2812Data.ui8StartRGB[2])/(WS2812Data.ui8NoOfActiveLEDs - 1)) );

      #ifdef WS2812_USE_SERIAL_DEBUG
        Serial.printf("RGB=%d,%d,%d\t\t",iR,iG,iB);
      #endif

      uint8_t ui8R, ui8G, ui8B;
      ui8R = WS2812Data.ui8StartRGB[0];
      ui8G = WS2812Data.ui8StartRGB[1];
      ui8B = WS2812Data.ui8StartRGB[2];
      for(ui8i=0; ui8i<WS2812Data.ui8NoOfActiveLEDs; ui8i++)
      {
        WS2812strip.setPixelColor(ui8i, WS2812strip.Color(ui8R,ui8G,ui8B));  // one pixel (first to n-1)
        ui8R += iR;
        ui8G += iG;
        ui8B += iB;
      }
      //WS2812strip.setPixelColor((WS2812Data.ui8NoOfActiveLEDs - 1),WS2812strip.Color(WS2812Data.ui8EndRGB[0], WS2812Data.ui8EndRGB[1], WS2812Data.ui8EndRGB[2]));  // last pixel
    }
    else if(WS2812Data.ui8NoOfActiveLEDs == 2)
    { // 2 LEDs
      WS2812strip.setPixelColor(0, WS2812strip.Color(WS2812Data.ui8StartRGB[0], WS2812Data.ui8StartRGB[1], WS2812Data.ui8StartRGB[2]));
      WS2812strip.setPixelColor(1, WS2812strip.Color(WS2812Data.ui8EndRGB[0], WS2812Data.ui8EndRGB[1], WS2812Data.ui8EndRGB[2]));
    }
    else
    { // 1 LED
      WS2812strip.setPixelColor(0, WS2812strip.Color(WS2812Data.ui8StartRGB[0], WS2812Data.ui8StartRGB[1], WS2812Data.ui8StartRGB[2]));
    }
    WS2812strip.show();
  }

  return;
}

void WS2812CollectDataFromLCD(void)   // used to collect user inputs from the LCD
{
  uint8_t ui8Temp;

  if(Serial2.available())
  {
    byte chIn = Serial2.read();
    //Serial.printf("rx1=%d\r\n",chIn);

    if(chIn == 0x71)  // waiting for a "get" command reply
    { //expected, known message
      if(WS2812Data.bValueFromLCDExpected)
      { // value was expected => store-it
        WS2812Data.bValueFromLCDExpected = false;
        chIn = Serial2.read();
        //Serial.printf("rx2=%d\r\n",chIn);
        switch(WS2812Data.ui8LCDInterrogationIndex)
        {
          case 0:   // START RED    (h0.val)
                    ui8Temp = (uint8_t)chIn;
                    WS2812Data.ui8StartRGB[0] = (uint8_t)map(ui8Temp,0,100,0,255);
                    WS2812Data.ui8LCDInterrogationIndex ++;
                    break;
          case 1:   // START GREEN  (h1.val)
                    ui8Temp = (uint8_t)chIn;
                    WS2812Data.ui8StartRGB[1] = (uint8_t)map(ui8Temp,0,100,0,255);
                    WS2812Data.ui8LCDInterrogationIndex ++;
                    break;
          case 2:   // START BLUE   (h2.val)
                    ui8Temp = (uint8_t)chIn;
                    WS2812Data.ui8StartRGB[2] = (uint8_t)map(ui8Temp,0,100,0,255);
                    WS2812Data.ui8LCDInterrogationIndex ++;
                    break;
          case 3:   // END   RED    (h3.val)
                    ui8Temp = (uint8_t)chIn;
                    WS2812Data.ui8EndRGB[0]   = (uint8_t)map(ui8Temp,0,100,0,255);
                    WS2812Data.ui8LCDInterrogationIndex ++;
                    break;
          case 4:   // END   GREEN  (h4.val)
                    ui8Temp = (uint8_t)chIn;
                    WS2812Data.ui8EndRGB[1]   = (uint8_t)map(ui8Temp,0,100,0,255);
                    WS2812Data.ui8LCDInterrogationIndex ++;
                    break;
          case 5:   // END   BLUE   (h5.val)
                    ui8Temp = (uint8_t)chIn;
                    WS2812Data.ui8EndRGB[2]   = (uint8_t)map(ui8Temp,0,100,0,255);
                    WS2812Data.ui8LCDInterrogationIndex ++;
                    break;
          case 6:   // LEDs No Val. (n0.val)
                    WS2812Data.ui8NoOfActiveLEDs   = (uint8_t)chIn;
                    WS2812Data.ui8LCDInterrogationIndex = 0;
                    break;
          default:  // some error
                    WS2812Data.ui8LCDInterrogationIndex = 0;
                    break;
        }
      }
      while(Serial2.available()) chIn = Serial2.read(); // empty the RX buffer
    }
    else
    { // unknown message
      WS2812Data.bValueFromLCDExpected = false;
      Serial.print(">>>");
      while(Serial2.available())
      {
        Serial.write(Serial2.read());
      }
      Serial.println("<<<");
    }
  }

  // request the new field from the LCD
  WS2812UpdateDisplay();

  return;
}


void WS2812UpdateDisplay(void)            // used to update the values on the LCD after a new measurement
{
  // get statuses from the LCD
  if( (WS2812Data.ui8LCDInterrogationIndex < 6) )   Serial2.printf("get h%d.val",WS2812Data.ui8LCDInterrogationIndex);
  else                                              Serial2.print("get n0.val");
  MCPNextionTXEnd();
  WS2812Data.bValueFromLCDExpected = true;
  delay(10);
  
  return;
}
//=======================================================================================
//=======================================================================================
//=======================================================================================


//=======================================================================================
//=======================================================================================
//=======================================================================================
//HCSR04

void HCSR04ShieldInit(){
  if(HCSR04Data.bShieldInit) return;

  HCSR04Data.bShieldInit = true;

  HCSR04Data.i16Distance[0] = 0;
  HCSR04Data.ui64Duration[0] = 0;
  HCSR04Data.i16Distance[1] = 0;
  HCSR04Data.ui64Duration[1] = 0;
  HCSR04Data.fSpeed[0] = 0;
  HCSR04Data.fSpeed[1] = 0;
  HCSR04Data.fAcceleration = 0;

  HCSR04Data.fMeasurementDuration = 0;
  HCSR04Data.ui64UpdateTime = millis();

  Serial2.printf("page HCSR040");
  MCPNextionTXEnd();
  delay(10);
  HCSR04Data.ui8CurrentPage = 14;

  HCSR04SensorInit();
  return;
}

void HCSR04ShieldDeinit(){
  if(!HCSR04Data.bShieldInit) return;
  //Shield is not initialized
  HCSR04Data.bShieldInit = false;
  //Reset the measured variables
  HCSR04Data.i16Distance[0] = 0;
  HCSR04Data.ui64Duration[0] = 0;
  HCSR04Data.i16Distance[1] = 0;
  HCSR04Data.ui64Duration[1] = 0;
  //Reset the calculated variables
  HCSR04Data.fSpeed[0] = 0;
  HCSR04Data.fSpeed[1] = 0;
  HCSR04Data.fAcceleration = 0;
  HCSR04Data.fMeasurementDuration = 0;

  //Reset the measurement time
  HCSR04Data.ui64UpdateTime = 0;

  HCSR04SensorDeinit();

  Serial2.print("page homepage");
  MCPNextionTXEnd();
  delay(10);

  return;
}

void HCSR04SensorInit(){
  if(HCSR04Data.bSensorInit) return;

  pinMode(HCSR04TriggerPin , OUTPUT);
  pinMode(HCSR04EchoPin, INPUT);
  return;
}

void HCSR04SensorDeinit(){
  if(!HCSR04Data.bSensorInit) return;
  pinMode(HCSR04TriggerPin , INPUT); //Reset pin's to INPUT mode, without removing the Shield
  pinMode(HCSR04EchoPin, INPUT);
  return;
}

void HCSR04loop(){
  if(!HCSR04Data.bShieldInit) HCSR04ShieldInit();

  while(Serial2.available()){
    NextionMessage = Serial2.read();
  } //Used to read current page, read until buffer is empty

  if(NextionMessage == 13){
    HCSR04MeasureSpeed(); 
    HCSR04UpdateDisplay();
    NextionMessage = 15; //Set the current page as 15
  }

  HCSR04MeasureDistance();

  if(NextionMessage == 15)HCSR04UpdateDisplay();

  return;
}

void HCSR04UpdateDisplay(){
  char chTemp[20];

  Serial2.printf("t1.txt=\"Distance:%3u[cm]\"",HCSR04Data.i16Distance[0]);
  MCPNextionTXEnd();
  delay(10);

  sprintf(chTemp,"%2.2lf",HCSR04Data.fSpeed[0]); //convert to string the measurement

  Serial2.printf("t2.txt=\"Speed:%s[cm/s]\"",chTemp);
  MCPNextionTXEnd();
  delay(10);

  sprintf(chTemp,"%2.2lf",HCSR04Data.fAcceleration); //convert to string the measurement

  Serial2.printf("t4.txt=\"Acceleration:%s[cm/s^2]\"",chTemp);
  MCPNextionTXEnd();
  delay(10);

  return;
}

void HCSR04MeasureSpeed(){
  HCSR04Data.ui64Duration[0] = HCSR04EchoMeasure(); //Measure two distances in the given time frame, with delay()
  delay(500);
  HCSR04Data.ui64Duration[1] = HCSR04EchoMeasure();

  //The time between two measurements is 12 microseconds and the Duration[1] - Duration[0] in microseconds

  // Calculating the distance
  HCSR04Data.i16Distance[1] = HCSR04Data.ui64Duration[1] * HCSR04SoundSpeed / 2;
  HCSR04Data.i16Distance[0] = HCSR04Data.ui64Duration[0] * HCSR04SoundSpeed / 2; //given in cm

  //Calculating the speed
  HCSR04Data.fSpeed[0] = abs(HCSR04Data.i16Distance[1] - HCSR04Data.i16Distance[0])*2; //cm/seconds
  return;
}

void HCSR04MeasureDistance(){
  HCSR04Data.ui64Duration[0] = HCSR04EchoMeasure();
  HCSR04Data.i16Distance[0] = HCSR04Data.ui64Duration[0] * HCSR04SoundSpeed / 2; //given in cm
  return;
}

unsigned int HCSR04EchoMeasure(void){
  unsigned int durationTemp;
  //Get the start time of the measurement
  digitalWrite(HCSR04TriggerPin , LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(HCSR04TriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR04TriggerPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationTemp = pulseIn(HCSR04EchoPin, HIGH, 10000); //Duration of the wave as it travels
  //10000 Timeout time, in case of error
  return durationTemp;
}
//=======================================================================================
//=======================================================================================
//=======================================================================================


//=======================================================================================
//=======================================================================================
//=======================================================================================
//SG90

void SG90ShieldInit(void){
  if(SG90Data.bShieldInit) return;
  SG90Data.bShieldInit = true;
  SG90Data.bSensorInit = false;

  SG90SensorInit();

  Serial2.print("page SG900");
  MCPNextionTXEnd();
  delay(10);
  
  return;
}

void SG90SensorInit(void){
  if(SG90Data.bSensorInit) return;
  SG90Data.bSensorInit = true;
  SG90Data.iMotorAngle = 0;
  SG90Data.iMotorSpeed = 0;
  SG90Data.ui8CurrentPage = 16; //Nextion PAGE id, check the Nextion file for more information
  SG90Data.ui8CurrentOption = 0;

  SG90Data.servo.attach(SG90SignalPin);

  return;
}

void SG90ShieldDeinit(void){
  if(!SG90Data.bShieldInit) return;
  SG90Data.bShieldInit = false;
  SG90Data.ui8CurrentPage = 0;

  SG90SensorDeinit();

  Serial2.print("page homepage");
  MCPNextionTXEnd();
  delay(10);
}

void SG90SensorDeinit(void){
  if(!SG90Data.bSensorInit) return;
  SG90Data.bSensorInit = false;
  SG90Data.iMotorAngle = 0;
  SG90Data.iMotorSpeed = 0;
  SG90Data.ui8CurrentPage = 0;

  SG90Data.servo.detach();
}

void SG90loop(){
  if(!SG90Data.bShieldInit) SG90ShieldInit();
  if(!SG90Data.bSensorInit) SG90SensorInit();

  while(Serial2.available() && SG90Data.ui8CurrentPage != 17){ //While the Serial is available and the page is no 17 do the rest
    NextionMessage = Serial2.read();
    if(NextionMessage == 17){
      SG90Data.ui8CurrentPage = 17; //Save the current page in the Data
    }
    else if(NextionMessage == 16){
      SG90Data.ui8CurrentPage = 16;
    }
  }

  if(SG90Data.ui8CurrentPage == 17){ // If the current page is the number 17, then wait for the next option choosen by the user
    while(Serial2.available()){ 
      NextionMessage = Serial2.read();
      if(NextionMessage == 1) SG90Data.ui8CurrentOption = 1;
      if(NextionMessage == 2) SG90Data.ui8CurrentOption = 2;
    }
  }
  Serial.println(SG90Data.ui8CurrentOption);

  if(SG90Data.ui8CurrentOption == 1){
    SG90ChangeAngle();
  }
  if(SG90Data.ui8CurrentOption == 2){
    SG90RotateWithGivenSpeed();
  }
}

void SG90ChangeAngle(void){
  byte chIN;

  Serial2.print("get n0.val"); // Get the value of n0 parameter
  MCPNextionTXEnd();
  delay(10);

  chIN = Serial2.read(); // Read in the first byte of the "get" value

  if(chIN == 0x71){ // If 0x71 the first value, then ->
    chIN = Serial2.read(); // Read the next value, which will be the value of n0 in bytes
    // Change the position of the Servo Motor
    int pos = (int)chIN;
    SG90Data.iMotorAngle = pos;
    SG90Data.servo.write(pos);
  }

  return;
}

void SG90RotateWithGivenSpeed(void){
  byte chIN;

  Serial2.print("get n1.val"); // Get the value of n0 parameter
  MCPNextionTXEnd();
  delay(10);

  chIN = Serial2.read(); // Read in the first byte of the "get" value

  if(chIN == 0x71){ // If 0x71 the first value, then ->
    chIN = Serial2.read(); // Read the next value, which will be the value of n0 in bytes
    // Change the position of the Servo Motor
    SG90Data.iMotorSpeed = (int)chIN;
    chIN = Serial2.read();
    if((int)chIN != 0) SG90Data.iMotorSpeed += 16*16*(int)chIN;
  }

  for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
    esp_task_wdt_reset(); //Used to reset the watchdog timer
    SG90Data.servo.write(posDegrees);
    delay(SG90Data.iMotorSpeed);
  }

  for(int posDegrees = 180; posDegrees >= 0; posDegrees--) {
    esp_task_wdt_reset();
    SG90Data.servo.write(posDegrees);
    delay(SG90Data.iMotorSpeed);
  }

  //while(Serial2.available()) chIN = Serial2.read(); // empty the buffer

  return;
}

//=======================================================================================
//=======================================================================================
//=======================================================================================
void MPU6050ShieldInit(){
  if(MPU6050Data.bShieldInit) return;

  MPU6050Data.bShieldInit = true;
  MPU6050Data.ui8CurrentPage = 18;

  Serial2.print("page MPU60500");
  MCPNextionTXEnd();
  delay(10);
}

void MPU6050SensorInit(){
  if(MPU6050Data.bSensorInit) return;
  MPU6050Data.bSensorInit = true;

  MPU6050Data.MPU.begin();
  MPU6050Data.MPU.setAccelerometerRange(MPU6050_RANGE_8_G);
  MPU6050Data.MPU.setGyroRange(MPU6050_RANGE_500_DEG);
  MPU6050Data.MPU.setFilterBandwidth(MPU6050_BAND_21_HZ);

  return;
}


void MPU6050ShieldDeinit(){
  if(!MPU6050Data.bShieldInit) return;
  MPU6050Data.bShieldInit = false;
  MPU6050SensorDeinit();

  Serial2.print("page homepage");
  MCPNextionTXEnd();
  delay(10);
  return;
}

void MPU6050SensorDeinit(){
  if(!MPU6050Data.bSensorInit) return;
  MPU6050Data.bSensorInit = false;
  Serial2.print("page MPU60500");
  MCPNextionTXEnd();
  delay(10);

  MPU6050Data.faccx = 0;
  MPU6050Data.faccy = 0;
  MPU6050Data.faccz = 0;
  MPU6050Data.fgyrox = 0;
  MPU6050Data.fgyroy = 0;
  MPU6050Data.fgyroz = 0;
  return;
}

void MPU6050loop(){
  MPU6050ShieldInit();
  MPU6050SensorInit(); //Called, if Shield is still intact, but Sensor was removed
  while(Serial2.available()){
    NextionMessage = Serial2.read();
    NextionMessage = Serial2.read();
    if(NextionMessage == 18) MPU6050Data.ui8CurrentPage = 18;
    if(NextionMessage == 19) MPU6050Data.ui8CurrentPage = 19;
    if(NextionMessage == 20) MPU6050Data.ui8CurrentPage = 20;
  }

  if(MPU6050Data.bSensorInit)MPU6050Measure();
}

void MPU6050Measure(void){
  sensors_event_t a, g, temp;
  MPU6050Data.MPU.getEvent(&a, &g, &temp);

  MPU6050Data.faccx = a.acceleration.x;
  MPU6050Data.faccy = a.acceleration.y;
  MPU6050Data.faccz = a.acceleration.z;
  MPU6050Data.fgyrox = g.gyro.x;
  MPU6050Data.fgyroy = g.gyro.x;
  MPU6050Data.fgyroz = g.gyro.x;

  MPU6050UpdateDisplay();
}

void MPU6050UpdateDisplay(){
  char chTemp[20];
  uint16_t ui16Temp;
  int iTemp;

  if(MPU6050Data.ui8CurrentPage == 19){
    sprintf(chTemp,"%2.2lf", MPU6050Data.faccx); //convert to string the measurement
    Serial2.printf("t1.txt=\"a_x=%s [m/s^2]\"", chTemp);
    MCPNextionTXEnd();
    delay(10);

    sprintf(chTemp,"%2.2lf", MPU6050Data.faccy); //convert to string the measurement
    Serial2.printf("t2.txt=\"a_y=%s [m/s^2]\"", chTemp);
    MCPNextionTXEnd();
    delay(10);

    sprintf(chTemp,"%2.2lf", MPU6050Data.faccz); //convert to string the measurement
    Serial2.printf("t4.txt=\"a_z=%s [m/s^2]\"", chTemp);
    MCPNextionTXEnd();
    delay(10);

    sprintf(chTemp,"%2.2lf", MPU6050Data.fgyrox); //convert to string the measurement
    Serial2.printf("t5.txt=\"gyro_x=%s [deg/s^2]\"", chTemp);
    MCPNextionTXEnd();
    delay(10);


    sprintf(chTemp,"%2.2lf", MPU6050Data.fgyroy); //convert to string the measurement
    Serial2.printf("t6.txt=\"gyro_y=%s [deg/s^2]\"", chTemp);
    MCPNextionTXEnd();
    delay(10);

    sprintf(chTemp,"%2.2lf", MPU6050Data.fgyroz); //convert to string the measurement
    Serial2.printf("t7.txt=\"gyro_z=%s [deg/s^2]\"", chTemp);
    MCPNextionTXEnd();
    delay(10);
  }
  if(MPU6050Data.ui8CurrentPage == 20){

    iTemp = (int)MPU6050Data.faccx;
    if(iTemp < 0) iTemp *= -1;
    ui16Temp = (uint16_t)iTemp;
    Serial2.printf("add 3,0,%u",ui16Temp);
    MCPNextionTXEnd();
    delay(10);

    iTemp = (int)MPU6050Data.faccy;
    if(iTemp < 0) iTemp *= -1;
    ui16Temp = (uint16_t)iTemp;
    Serial2.printf("add 3,1,%u",ui16Temp);
    MCPNextionTXEnd();
    delay(10);

    iTemp = (int)MPU6050Data.faccz;
    if(iTemp < 0) iTemp *= -1;
    ui16Temp = (uint16_t)iTemp;
    Serial2.printf("add 3,2,%u",ui16Temp);
    MCPNextionTXEnd();
    delay(10);
  }

  return;
}