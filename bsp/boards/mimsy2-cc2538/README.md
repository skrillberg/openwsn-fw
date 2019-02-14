# Mimsy 2 BSP and Drivers
This directory contains the board support package (BSP) and drivers for Mimsy 2. The BSP includes firmware that enables the use of inchworm motors, the Mimsy 2's onboard MPU9250 IMU, Mimsy 2's UART-Based serial port, and Mimsy 2's LEDs. 

## accel_mimsy.c
This file contains methods for interfacing with Mimsy 2's onboard MPU9250 IMU. It can currently initialize the accelerometer, gather accel and gyro data into an IMUData struct, and initialize the DMP onboard sensor fusion processor on the IMU. 

### Dependencies
* flash_mimsy.h: requires the IMUData struct from flash_mimsy
* mpu9250/MPU9250_RegisterMap.h
* i2c_mimsy.h: for i2c reads and writes
* gptimer.h: for timer access for timestamping
* hw_gptimer.h: for timer access
* hw_memmap.h: 
* inv_mpu.h:invensense mpu drivers
* inv_mpu_dmp_motion_driver.h:
* invensense.h: random utilities used by the invensense code
* invensense_adv.h: header for MPL binary library
* eMPL_outputs.h: includes data output utilities used by invensense's MPL
* mltypes.h: more random invensense stuff
* mpu.h: more invensense libraries
* log.h: more invensense utilities

### Functions 
* void mimsyIMUInit()

  Initalizes the imu on mimsy. This function enables all of the sensors.


* void mimsySetAccelFsr(int fsr)

  Sets the accellerometers full scale range in g's. Valid values for fsr are 2,4,8,and 16


* void mimsySetGyroFsr(int fsr)

  Sets the gyro's full scale range. Valid values are 250,500,1000, and 2000


* void mimsyIMURead6Dof( IMUData *data)

  Read accel xyz and gyro xyz data from imu. The input to this function is a pointer to a IMUData struct.


* void mimsyDmpBegin()

  Initializes the dmp 
  
## flash_mimsy.c
This file contains functions for using Mimsy's flash memory. Right now it supports saving and reading IMUData structs 
### Dependencies
* gptimer.h: for timer access
* sys_ctrl.h: for interrupt control
* hw_gptimer.h: for timer access
* interrupt.h

### Data Structures
* IMUData
  
  Union that contains three different types, **struct** *fields*, **struct** *signedfields*, and **uint32_t** *bits*. This data structure stores IMU data read from the MPU9250 and allows for easy interfacing with the flash write and read functions. 
  * *fields*: a struct that contains the raw unsigned representation of the IMU data
    * *timestamp*: contains the timestamp that denotes the time the data was taken at
  * *signedfields*: a struct that contains the true signed representation of the IMU data. This provides for easy conversion from the raw data to the true signed data
  
  * *bits*: This type representation of the union is used by the flash utilities to write and read the IMUData structures to and from flash
  
* IMUDataCard
  
  A struct used to keep track of which page IMU data is stored in in flash
  * *page*: Contains the page number where a range of IMUData points is stored. 
  * *startTime*: Contains the timestamp of the first datapoint (data[0]) stored in the page
  * *endTime*: the timestamp of the last datapoint (data[127]) stored in the page
    

### Functions
* flashWriteIMU(IMUData data[], uint32_t size, uint32_t startPage, IMUDataCard (pointer) card)

  Writes a list of IMUData structures to flash memory. This function writes up to one page worth of structs to flash.  128 IMUData structs fit in one page. Writing to a page will erase all of that page's contents, even if you are writing to only a portion of the page. Valid page range is 0-255.
  
* flashReadIMU(IMUDataCard card, IMUData *dataArray, uint32_t size)

  Reads all of the IMUData structs saved in the flash page specified by the IMUDataCard input parameter. 


## i2c_mimsy.c
This file contains functions for using i2c to write to and read from the accelerometer.

## inchworm.c
This file contains functions and utilities required for driving electrostaic inchworm motors with Mimsy 2. Driving inchworm motors requires a high voltage switching circuit or a PIPS board 

### Data Structures

* **struct** *InchwormMotor*

  A struct that is used to identify which GPIOs are used for driving the inchworm motors. Create one of these for each inchworm motor used
    * GPIObase1: the gpio base for pin 1 of the inchworm motor i.e. GPIO_BASE_A
    * GPIOpin1: the gpio pin for pin 1 of the inchworm i.e. GPIO_PIN_1
    * GPIObase2: the gpio base for pin 2 of the inchworm motor i.e. GPIO_BASE_B
    * GPIOpin2: the gpio pin for pin 2 of the inchworm i.e. GPIO_PIN_2
    * motorID: a unique ID for this motor object
* **struct** *InchwormSetup*

  A struct that is used to define the setup parameters for all of the inchworm motors being driven by the board. 
    * iwMotors: a pointer to the list of all the inchworm motor structs instantiated for use.
    * numOfMotors: the number of motors being driven by the Mimsy board
    * motorFrequency: the frequency that all inchworm motors will be driven at in Hz. All inchworm motors on the board must share the same frequency to to limited hardware timer resources available.
    * dutyCycle: duty cycle of pwm signals used to drive inchworm motors. Ideally between 60 and 90. Increasing duty cycle increases the amount of overlap time where both inchworm palls are engaged. 
    * motorID: not used
    * timer: determines which timer module is used by the inchworm module to drive the inchworm signals. This cannot be 2 because timer 2 is used by the openWsn board time. 
    * phaseTimer: determines which timer module should be used by the inchworm module to generate the phase offset of the two inchworm drive signals. This cannot be 2 because of previously stated reasons
    
### Functions

* inchwormInit(**struct** InchwormSetup)
  
  Initializes the timers and GPIOs and pin mappings required to generate inchworm drive signals. See above documentation for *InchwormSetup* and *InchwormMotor* structs for more information on inchworm configuration
  
* inchwormRelease(**InchwormMotor** motor)

  Releases the palls on the specified inchworm motor which allows free movement of the inchworm shuttle

* inchwormHold(**InchwormMotor** motor)

  Engages the palls on the specified inchworm motor and holds the shuttle in place.
  
* inchwormFreerun(**InchwormMotor** motor)

  Runs the specified inchworm motor indefinitely

* inchwormDriveToPosition(**InchwormMotor** motor, **uint32_t** steps)

  Runs the specified inchworm motor for the number of steps specified by *steps*

## uart_mimsy.c

Contains functions for using the mimsy uart serial port. At some point this could be used for communicating for a variety of devices. 

### Defines

* EXAMPLE_PIN_UART_RXD: Specifies the gpio pin number of the UART rx pin i.e. GPIO_PIN_2
* EXAMPLE_PIN_UART_TXD: Specifies the gpio pin number of the UART tx pin i.e. GPIO_PIN_1
* EXAMPLE_GPIO_BASE: specifies the gpio base number of the UART tx and rx pins (both pins must share the same GPIO base) i.e. GPIO_D_BASE

### Functions

* uartMimsyInit()

  Function that initializes the Mimsy UART serial port

* mimsyPrintf(**const char** (pointer) pcString, ...)

  Function that writes a string to serial port. Behaves like the standard *printf* command used in C.
  
## led.c 

This file contains functions for using the red and green LEDs on Mimsy 2


