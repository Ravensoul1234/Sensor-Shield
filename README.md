# Sensor-Shield
## Main code of a sensor Shield.

This project is still a work in progress, but here's a short description of it:

Our main objective is to build a Shield, for the ESP32 microcontroller, that can detect more than 200 sensors using an MCP23071 digital port expander.
To ensure interactivity, the Shield is also equipped with a Nextion HMI Display, where measurements can be seen and interacted with.


**Sensors.h ->**

    ---contains all of the sensors structs
  
    ---contains all of the sensors functions
  
    ---contains some global variables for the code to run
  
    ---contains Shield ID's, for recognizing them
  
    ---contains #include libraries

**main.cpp ->** 

    ---contains the defined functions main code
  
    ---handles the setup and loop functions of an Arduino based code
  
    ---initializes the Watchdog-timer
  
    ---initializes Serial communications for the sensors and a Nextion display
  
  **integrity.dat ->**
  
    ---contains the lib-deps for PlatformIO and their versions
  
