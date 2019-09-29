## MotorLib Arduino
#### Based on Arduino FreeRTOS
Motor Library for Arduino. Is builded on top of Arduino FreeRTOS Library. Features include PID controllers with Autotuning capability. RTOS Tasks take care of printing required information to serial, Calculating velocity and running the Controller Loop. 
``` config.h ``` file can be used to set up the required parameters for the Library. 

The library is written using VSCode with PlatformIO plugin, for running the example file you can put the MotorLib folder and the ```main.cpp``` file inside the ```src``` folder of your PlatformIO workspace.

The Library is tested in Arduino Uno with one Motor. Code still under development.
**Feel free to test and contribute :)**
