#ifndef _SENSOR_TESTER_
#define _SENSOR_TESTER_

//Necessary library includes, for Sensors and ESP32-Digital Pin Extender
#include <Wire.h>
#include <stdlib.h>
#include <esp_task_wdt.h>
#include <inttypes.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MPU6050.h>
#include <Servo.h>

//I2C PROTOCOL DEFINES
#define VL53L0XI2CAddress  0x29
#define MPU6050I2CAddress 0x68
#define BME280I2CAddress  0x77
//==========================================


//Defines for MCP
#define MCPI2CAddress 0x21
#define MCPpin  4
#define MCPGpintenaAddress  0x04
#define MCPGppuaAddress 0x0C
#define MCPGpioaAddress 0x12
//Defines for MCP

//Other defines
#define WDT_TIMEOUT 3 //Timeout time
//Other defines

//Shield ID defines
#define POTmeterShieldID 1 //Permanent ID's
#define JOYSTICKShieldID 3
#define BME280ShieldID 14
#define HW040ShieldID 2
#define JOYShieldID 3
#define VL53L0XShieldID 21
#define WS2812ShieldID 5
#define HCSR04ShieldID 20
#define SG90ShieldID 6
#define MPU6050ShieldID 12
//Shield ID defines

//Basic Functions for MCP and NEXTION==================================================
void MCPinit(void);
void MCPloop(void);
void MCPReadShieldID(void);
void MCPNextionTXEnd(void);
void MCPClearGPIOAndShields(void);

//=================================================================
//BME280

typedef struct{
    bool bShieldInit = false;
    bool bSensorInit = false;
    bool bSensorFound = false;
    uint8_t ui8CurrentPage;

    Adafruit_BME280 bme;
    unsigned int uiArrayIndex;
    uint16_t ui16Temperature[200];
    uint16_t ui16Humidity[200];
    uint16_t ui16Pressure[200];
    int16_t i16Altitude[200];
}BME280DataType;
BME280DataType BME280Data;      // sensor variable
#define BME280_READ_INTERVAL_MS      100
#define BME280_MAX_ARRAY 199
#define BME280_SEA_LEVEL_PRESSURE_HPA 1013.25

//----------------------------------------------------------------------------------------------
void BME280loop(void); // called from the loop() as long as the shield is present
//----------------------------------------------------------------------------------------------
void BME280ShieldInit(void); // called if the shield was inserted
//----------------------------------------------------------------------------------------------
void BME280ShieldDeInit(void); // called if the shield is missing
//----------------------------------------------------------------------------------------------
void BME280SensorRead(void); // called to handle the sensor measurements
//----------------------------------------------------------------------------------------------
void BME280SensorInit(void); // called if the sensor was inserted
//----------------------------------------------------------------------------------------------
void BME280SensorDeinit(void); // called if the sensor is missing 
//----------------------------------------------------------------------------------------------
void BME280UpdateDisplay(void); // called to update the Nextion display
//----------------------------------------------------------------------------------------------
void BME280GetData(void); // called to extract data from storage array
//----------------------------------------------------------------------------------------------



//=========================================================================================
//POTENTIALMETER
typedef struct{
    bool bPShieldInit = false;
    bool bPSensorInit = false;
    bool bPSensorFound = false;

    uint8_t ui8CurrentPage;
    uint8_t ui8LastPage;

    uint16_t ui16Voltage;
}POTDataType;
POTDataType POTData;

void POTloop(void); // called from the loop() as long as the shield is present
//----------------------------------------------------------------------------------------------
void POTShieldInit(void); // called if the shield was inserted
//----------------------------------------------------------------------------------------------
void POTSensorInit(void); // called if the sensor was inserted
//----------------------------------------------------------------------------------------------
void POTShieldDeinit(void); // called if the shield is missing
//----------------------------------------------------------------------------------------------
void POTSensorDeinit(void); // called if the sensor is missing
//----------------------------------------------------------------------------------------------
void POTSensorRead(void); // called to handle the measurements
//----------------------------------------------------------------------------------------------
void POTUpdateDisplay(void); // called to update the Nextion display
//----------------------------------------------------------------------------------------------
void POTGetData(void); // called to get the stored data
//----------------------------------------------------------------------------------------------


//=====================================================================================
//HW04
typedef struct{
    //Shield stuff
    bool bHWShieldInit = false; // Shield Initialization variable, TRUE if initialized, FALSE if else
    bool bHWSensorInit = false;  // Sensor Initialization variable, TRUE if initialized, FALSE if else
    bool bHWSensorFound = false; //IF Sensor is found, will become TRUE

    //nextion
    uint8_t ui8CurrentPage; //Holds Nextion display current page value

    //code only
    int16_t i16Position; //Holds current position of the sensor, how many times have it moved one way or another
    uint8_t ui8PressedButton; //Holds the value of 1 if the button is pressed, 0 if unpressed
    String ScurrentDirection = "No direction";
}HWDataType;
HWDataType HWData;
#define HWCLK 25
#define HWDT 33
#define HWSW 32

void HWloop(void); // called from the loop() as long as the shield is present
//----------------------------------------------------------------------------------------------
void HWShieldInit(void); // called if the shield was inserted
//----------------------------------------------------------------------------------------------
void HWSensorInit(void); // called if the sensor was inserted
//----------------------------------------------------------------------------------------------
void HWShieldDeinit(void); // called if the shield is missing
//----------------------------------------------------------------------------------------------
void HWSensorDeinit(void); // called if the sensor is missing
//----------------------------------------------------------------------------------------------
void HWSensorRead(void); // called to handle the sensor measurements
//----------------------------------------------------------------------------------------------
void HWUpdateDisplay(void); // called to update the Nextion display
//----------------------------------------------------------------------------------------------
void POTisr(void); // ISR function, works for interrupts in the sensor ports
//----------------------------------------------------------------------------------------------



//=====================================================================================
//JOYSTICK
typedef struct{
    bool bShieldInit = false;                   // set to true after the shield was initialized, set to false if the shield was removed

    unsigned long ulLastReadTimeMS;               // holds the last read moment when the sensor was read, time is from millis()
    unsigned long ulLastReadTimeMSInterval;       // holds the reading interval in ms units

    bool bNewDataIsAvailable = false;            // set to true each time a successful sensor reading is performed
    uint8_t ui8GPIOPinXNo;                  // the pin number where the sensor is connected
    uint8_t ui8GPIOPinYNo;                  // the pin number where the sensor is connected
    uint8_t ui8GPIOPinSWNo;                 // the pin number where the sensor is connected
    uint16_t ui16ADCValueX;                  // holds the ADC Value between 0 and 4095 since ADC is 12Bit
    uint16_t ui16ADCValueY;                  // holds the ADC Value between 0 and 4095 since ADC is 12Bit
    bool bSwitchValue;                   // if the switch is pressed => considered true, for not-pressed=>false
    uint16_t ui16ADCValueXOld;               // holds the ADC Value between 0 and 4095 since ADC is 12Bit
    uint16_t ui16ADCValueYOld;               // holds the ADC Value between 0 and 4095 since ADC is 12Bit
    bool bSwitchValueOld;                // if the switch is pressed => considered true, for not-pressed=>false
    uint8_t ui8CurrentPage;
}JOYDataType;

JOYDataType JOYData;      // sensor variable
#define JOY_READ_INTERVAL_MS 20

void JOYShieldInit(void);            // called if shield was inserted
//----------------------------------------------------------------------------------------------
void JOYShieldDeinit(void);          // called in case shield was removed, called by the system function "MCPClearGPIOAndShields()"
//----------------------------------------------------------------------------------------------
void JOYloop(void);            // called from the loop() as long as the shield is present
//----------------------------------------------------------------------------------------------
void JOYHandleSensor(void);         // called from the sensor loop function to handle connection, initialization and data accumulation from the sensor
//----------------------------------------------------------------------------------------------
void JOYReadData(void);             // used to read a new distance from the TOF sensor
//----------------------------------------------------------------------------------------------
void JOYUpdateDisplay(void); // used to update nextion display

//================================================================================================================
//----------------------------------------------------------------------------------------------
//VL53L0X
#define VL53L0X_HISTORY_DEPTH   5

typedef struct
{
    bool                    bShieldInit = false;            // set to true after the shield was initialized, set to false if the shield was removed
    bool                    bSensorFound = false;           // set to true if the sensor was detected on the shield, set to false if the sensor was removed from the shield
    bool                    bSensorInit = false;
    uint8_t                 ui8ErrorCode;                   // contains the last Error code received during I2C operations

    unsigned long           ulLastReadTimeMS;               // holds the last read moment when the sensor was read, time is from millis()
    unsigned long           ulLastReadTimeMSInterval;       // holds the reading interval in ms units
    unsigned int            uiArrayIndex;

    bool                    bNewDataIsAvailable;            // set to true each time a successful sensor reading is performed
    Adafruit_VL53L0X        lox = Adafruit_VL53L0X();       // VL53L0X sensor
    uint16_t                ui16DistanceMM[200];                 // holds the distance measured in millimeters
    uint16_t                ui16DistanceMMHistory[VL53L0X_HISTORY_DEPTH];       // used to detect if the sensor is jammed on the same value and to re-initialize the sensor
    uint8_t                 ui8DistanceHistoryIndex;        // used to index the above matrix
    uint8_t                 ui8CurrentPage;
}VL53L0XDataType;
VL53L0XDataType VL53L0XData;      // sensor variable
#define VL53L0X_READ_INTERVAL_MS      100
//----------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------
void VL53L0XShieldInit(void);            // called if shield was inserted
//----------------------------------------------------------------------------------------------
void VL53L0XSensorInit(void);           // called to initialize the VL53L0X sensor
//----------------------------------------------------------------------------------------------
void VL53L0XShieldDeinit(void);          // called in case shield was removed, called by the system function "MCPClearGPIOAndShields()"
//----------------------------------------------------------------------------------------------
void VL53L0XSensorDeinit(void);         // called if a sensor removal was detected
//----------------------------------------------------------------------------------------------
void VL53L0Xloop(void);                 // called from the loop() as long as the shield is present
//----------------------------------------------------------------------------------------------
void VL53L0XReadDistance(void);         // used to read a new distance from the TOF sensor
//----------------------------------------------------------------------------------------------
void VL53L0XUpdateDisplay(void);           // used to update the values on the LCD after a new measurement
//----------------------------------------------------------------------------------------------
void VL53L0XGetData(void);



//================================================================================================================
//----------------------------------------------------------------------------------------------
//LED STRIPE
typedef struct
{
    bool                    bShieldInit;                   // set to true after the shield was initialized, set to false if the shield was removed
    unsigned long           ulLastReadTimeMS;               // holds the last read moment when the sensor was read, time is from millis()
    unsigned long           ulLastReadTimeMSInterval;       // holds the reading interval in ms units
    unsigned long           ulUpdateIfNoChangeMS;           // if there is no change in the LCD settings from time to time must update in case the used 
                                                                //removes and re-inserts the WS2812 stripe.

    // values from the TFT/LCD setting
    uint8_t                 ui8LCDInterrogationIndex;       // used to interrogate the LCD fields
    uint8_t                 ui8CurrentPage;
    // -ui8LCDInterrogationIndex    == 0        => START RED    (h0.val)
    // -ui8LCDInterrogationIndex    == 1        => START GREEN  (h1.val)
    // -ui8LCDInterrogationIndex    == 2        => START BLUE   (h2.val)
    // -ui8LCDInterrogationIndex    == 3        => END   RED    (h3.val)
    // -ui8LCDInterrogationIndex    == 4        => END   GREEN  (h4.val)
    // -ui8LCDInterrogationIndex    == 5        => END   BLUE   (h5.val)
    // -ui8LCDInterrogationIndex    == 6        => LEDs No Val. (n0.val)
    bool                    bValueFromLCDExpected;          // set to true if a get command was sent to the LCD and a reply with 71...FF FF FF is expected
    uint8_t                 ui8NoOfActiveLEDs;              // from the 20 LEDs can be active fewer, the rest will be black;
    uint8_t                 ui8NoOfActiveLEDsOld;           // from the 20 LEDs can be active fewer, the rest will be black;
    uint8_t                 ui8StartRGB[3];                 // ui8StartRGB[0]=R,        ui8StartRGB[1]=G,       ui8StartRGB[2]=B
    uint8_t                 ui8StartRGBOld[3];              // ui8StartRGBOld[0]=R,     ui8StartRGBOld[1]=G,    ui8StartRGBOld[2]=B
    uint8_t                 ui8EndRGB[3];                   // ui8EndRGB[0]=R,          ui8EndRGB[1]=G,         ui8EndRGB[2]=B
    uint8_t                 ui8EndRGBOld[3];                // ui8EndRGBOld[0]=R,       ui8EndRGBOld[1]=G,      ui8EndRGBOld[2]=B
}WS2812DataType;
WS2812DataType WS2812Data={0};      // sensor variable
#define WS2812PIN 32
Adafruit_NeoPixel WS2812strip = Adafruit_NeoPixel(20, WS2812PIN, NEO_GRB + NEO_KHZ800);
#define WS2812_UPDATE_INTERVAL_MS       50       /* how often the LCD is interrogated */
#define WS2812_UPDATE_WITH_NO_CHANGE    800     /* in case there is no change on the LCD but the user removes and re-inserts the WS2812 stripe => must update from time to time */
//----------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------
void WS2812ShieldInit(void);            // called if shield was inserted
//----------------------------------------------------------------------------------------------
void WS2812ShieldDeinit(void);          // called in case shield was removed, called by the system function "MCPClearGPIOAndShields()"
//----------------------------------------------------------------------------------------------
void WS2812loop(void);            // called from the loop() as long as the shield is present
//----------------------------------------------------------------------------------------------
void WS2812HandleSensor(void);         // called from the sensor loop function to handle connection, initialization and data accumulation from the sensor
//----------------------------------------------------------------------------------------------
void WS2812WriteData(void);             // used to read a new distance from the TOF sensor
//----------------------------------------------------------------------------------------------
void WS2812CollectDataFromLCD(void);   // used to collect user inputs from the LCD
//----------------------------------------------------------------------------------------------
void WS2812UpdateDisplay(void);            // used to update the values on the LCD after a new measurement


//================================================================================================================
//----------------------------------------------------------------------------------------------
//HCSR04
#define HCSR04TriggerPin 5
#define HCSR04EchoPin 18
#define HCSR04SoundSpeed 0.034 //given in [cm/ms]

typedef struct{

    bool bShieldInit = false; //Holds whether the Shield was initialized or not
    bool bSensorInit = false; //If Shield is initialized, will check if Sensor is initialized next
    uint8_t  ui8CurrentPage; //Holds the current page on Nextion

    uint64_t ui64Duration[2]; //Holds the duration of the sound wave to travel to a object and back
    int16_t i16Distance[2]; //Will hold the distance between the sensor and the object
    float fSpeed[2]; //Will hold the relative speed of the sensor object 
    float fAcceleration; //Will hold the relative acceleration of the sensor object

    float fMeasurementDuration; //Will hold the time that passed between two measurements
    uint64_t ui64UpdateTime; //for speed and acceleration
}HCSR04DataType;
HCSR04DataType HCSR04Data;

//----------------------------------------------------------------------------------------------
void HCSR04hieldInit(void);            // Called if shield was inserted
//----------------------------------------------------------------------------------------------
void HCSR04ShieldDeinit(void);          // Called in case shield was removed, called by the system function "MCPClearGPIOAndShields()"
//----------------------------------------------------------------------------------------------
void HCSR04SensorInit(void);            // Called if sensor is inserted/present
//----------------------------------------------------------------------------------------------
void HCSR04SensorDeinit(void);          // Called if sensor is removed
//----------------------------------------------------------------------------------------------
void HCSR04loop(void);            // Called from the loop() as long as the shield is present
//----------------------------------------------------------------------------------------------
void HCSR04UpdateDisplay(void);   // Used to update the values on the Nextion Display after a new measurement
//----------------------------------------------------------------------------------------------
void HCSR04MeasureSpeed(void); // Function to measure speed in a given time frame when button is pressed
//----------------------------------------------------------------------------------------------
void HCSR04MeasureDistance(void); // Looped function to measure distance between sensor and object
//----------------------------------------------------------------------------------------------
unsigned int HCSR04EchoMeasure(void); //Used to measure the duration of the bounce from object to sensor

//================================================================================================================
//----------------------------------------------------------------------------------------------
//STEPPERMOTOR
#define SG90SignalPin 32

typedef struct{
    bool bShieldInit = false; //In the beginning, shield is not initialized
    bool bSensorInit = false;
    uint8_t ui8CurrentPage; // Keeps the choosen Page on the Nextion Display
    uint8_t ui8CurrentOption; // Keeps the choosen option, change angle or rotate at constant speed

    int iMotorAngle;
    int iMotorSpeed;
    Servo servo;

}SG90DataType;
SG90DataType SG90Data;

//----------------------------------------------------------------------------------------------
void SG90ShieldInit(void); // Called if shield was inserted
//----------------------------------------------------------------------------------------------
void SG90SensorInit(void); // Called if sensor is found
//----------------------------------------------------------------------------------------------
void SG90ShieldDeinit(void); // Called if shield is missing
//----------------------------------------------------------------------------------------------
void SG90SensorDeinit(void); // Called if sensor is missing
//----------------------------------------------------------------------------------------------
void SG90loop(void); // Sensor loop for measurements/changes in the sensor
//----------------------------------------------------------------------------------------------
void SG90UpdateDisplay(void); // Update the Nextion Display with the correct values 
//----------------------------------------------------------------------------------------------
void SG90ChangeAngle(void); // Called when user wants to change the motor angle
//----------------------------------------------------------------------------------------------
void SG90RotateWithGivenSpeed(void); // Called when the user wants the motor to rotate with given speed
//----------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
typedef struct{
    bool bShieldInit; //used to check if the shield was initialized
    bool bSensorInit; //used to check if the sensor was initialized
    uint8_t ui8CurrentPage; //Tracks the current page on Nextion Display

    Adafruit_MPU6050 MPU; //Adafruit_MPU6050 type class, holds the key to measure and use Adafruit functions

    float faccx; //holds the measurement of acceleration in x direction
    float faccy; //holds the measurement of acceleration in y direction
    float faccz; //holds the measurement of acceleration in z direction
    float fgyrox; //holds the measurement of gyoscope in x direction
    float fgyroy; //holds the measurement of gyroscope in y direction
    float fgyroz; //holds the measurement of gyroscope in z direction

}MPU6050DataType;
MPU6050DataType MPU6050Data;

//----------------------------------------------------------------------------------------------
void MPU6050ShieldInit(void); // Called if shield was inserted
//----------------------------------------------------------------------------------------------
void MPU6050SensorInit(void); // Called if sensor is found
//----------------------------------------------------------------------------------------------
void MPU6050ShieldDeinit(void); // Called if shield is missing
//----------------------------------------------------------------------------------------------
void MPU6050SensorDeinit(void); // Called if sensor is missing
//----------------------------------------------------------------------------------------------
void MPU6050loop(void); // Sensor loop for measurements/changes in the sensor
//----------------------------------------------------------------------------------------------
void MPU6050UpdateDisplay(void); // Update the Nextion Display with the correct values 
//----------------------------------------------------------------------------------------------
void MPU6050Measure(void); // Called to handle the masurements
//----------------------------------------------------------------------------------------------

#endif
