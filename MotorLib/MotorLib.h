#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

#ifndef MOTOR_LIB_H
#define MOTOR_LIB_H

#define INIT_INTERRUPT(name)      \
    auto updateCount##name = [] { \
        name.encoderCallback();   \
    };                            \
    attachInterrupt(digitalPinToInterrupt(name.pin_encA), updateCount##name, RISING)

namespace MotorLib
{

class Motor
{
    int pin_pwm;
    int pin_dir;

    long int encoderCount = 0;
    long int prevEncoderCount = 0;
    long int encoderVel = 0;

    // PID terms
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Controller_Input = 0;
    double Controller_Setpoint = 0;
    double Controller_Output = 0;
    double Error = 0;
    double prev_Error = 0;
    double diffError = 0;
    double sumError = 0;

    int current_pwm = 0;

    int pwm_u_limit = 0;
    int pwm_l_limit = 0;

    int limitPwm(int pwm);

public:
    typedef enum
    {
        OPEN_LOOP = 0,
        CLOSED_LOOP,
        AUTOTUNE
    } motorMode;

    motorMode Mode;

    int pin_encA;
    int pin_encB;

    Motor();
    Motor(int pwm, int dir, int encA, int encB);
    Motor(int pwm, int dir, int encA, int encB, motorMode Mode);

    void setPinPWM(int pin);
    void setPinDir(int pin);
    void setPinEncA(int pin);
    void setPinEncB(int pin);
    void setMode(motorMode mode);

    void encoderCallback(void);
    long int getEncoderCount(void);
    void calculateVelocity(void);
    long int getVelocity(void);
    int getPwm(void);
    void writeMotorPwm(int pwm);
    void setPwmLimits(int L_limit, int U_limit);

    void setPID(double p, double i, double d);
    void setpoint(long int setpoint);
    void runPosControlLoop(void);
    
    void printData(void);
};

struct motorStack
{
    Motor *motor;
    motorStack *next_motor;
};

typedef struct motorStack motorStack;

void initialize();
void autoTune(MotorLib::Motor *autotune_motor);

} // namespace MotorLib

#endif // MOTOR_LIB_H