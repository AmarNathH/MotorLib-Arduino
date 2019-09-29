#include <Arduino.h>
#include <MotorLib/MotorLib.h>

// Declare motor objects globally
MotorLib::Motor A(11, 6, 2, 7);

void setup()
{
  Serial.begin(9600);
  A.setMode(MotorLib::Motor::CLOSED_LOOP);

  // Initialize interrupts
  INIT_INTERRUPT(A);

  // Tune only one Motor at time
  // MotorLib::autoTune(&A);

  A.setPID(0.1466,0.9166,0);
  A.setpoint(1000000);

  // Initialize after declaring all the motor variables
  MotorLib::initialize();

  vTaskStartScheduler();
}

void loop()
{
  // put your main code here, to run repeatedly:
}