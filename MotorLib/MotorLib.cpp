#include "MotorLib.h"
#include "config.h"

MotorLib::motorStack *top_motorStack; // to store the motor class object at the top of the stack

MotorLib::Motor::Motor()
{
    //set pins
    setPinPWM(0);
    setPinDir(0);
    setPinEncA(0);
    setPinEncB(0);
    setMode(MotorLib::Motor::OPEN_LOOP);

    //push class object to stack
    MotorLib::motorStack *temp_var;
    temp_var = new MotorLib::motorStack;
    temp_var->motor = this;
    temp_var->next_motor = top_motorStack;
    top_motorStack = temp_var;

    //Default settings
    setPID(0, 0, 0);
    setPwmLimits(-255, 255);
}

MotorLib::Motor::Motor(int pwm, int dir, int encA, int encB)
{
    //set pins
    setPinPWM(pwm);
    setPinDir(dir);
    setPinEncA(encA);
    setPinEncB(encB);
    setMode(MotorLib::Motor::OPEN_LOOP);

    //push class object to stack
    MotorLib::motorStack *temp_var;
    temp_var = new MotorLib::motorStack;
    temp_var->motor = this;
    temp_var->next_motor = top_motorStack;
    top_motorStack = temp_var;

    //Default settings
    setPID(0, 0, 0);
    setPwmLimits(-255, 255);
}

MotorLib::Motor::Motor(int pwm, int dir, int encA, int encB, MotorLib::Motor::motorMode mode)
{
    //set pins
    setPinPWM(pwm);
    setPinDir(dir);
    setPinEncA(encA);
    setPinEncB(encB);
    setMode(mode);

    //push class object to stack
    MotorLib::motorStack *temp_var;
    temp_var = new MotorLib::motorStack;
    temp_var->motor = this;
    temp_var->next_motor = top_motorStack;
    top_motorStack = temp_var;

    //Default settings
    setPID(0, 0, 0);
    setPwmLimits(-255, 255);
}

//Encoder callback function to update the encoder counts
//TODO: Find way to check how the motor moves.
void MotorLib::Motor::encoderCallback(void)
{
    if (digitalRead(pin_encB) == 0)
    {
        encoderCount++;
    }
    else
    {
        encoderCount--;
    }
}

void MotorLib::Motor::calculateVelocity(void)
{
    encoderVel = (encoderCount - prevEncoderCount) / VELOCITY_LOOP_DTIME;
    prevEncoderCount = encoderCount;
}

int MotorLib::Motor::limitPwm(int pwm)
{
    if (pwm > pwm_u_limit)
    {
        return pwm_u_limit;
    }
    else if (pwm < pwm_l_limit)
    {
        return pwm_l_limit;
    }
    return pwm;
}

void MotorLib::Motor::setPinPWM(int pin)
{
    pin_pwm = pin;
    pinMode(pin_pwm, OUTPUT);
}

void MotorLib::Motor::setPinDir(int pin)
{
    pin_dir = pin;
    pinMode(pin_dir, OUTPUT);
}

void MotorLib::Motor::setPinEncA(int pin)
{
    pin_encA = pin;
    pinMode(pin_encA, INPUT);
}

void MotorLib::Motor::setPinEncB(int pin)
{
    pin_encB = pin;
    pinMode(pin_encB, INPUT);
}

void MotorLib::Motor::setMode(MotorLib::Motor::motorMode mode)
{
    Mode = mode;
}

long int MotorLib::Motor::getEncoderCount(void)
{
    return encoderCount;
}

long int MotorLib::Motor::getVelocity(void)
{
    return encoderVel;
}

int MotorLib::Motor::getPwm(void)
{
    return current_pwm;
}

void MotorLib::Motor::writeMotorPwm(int pwm)
{
    if (pwm <= 0)
    {
        pwm = limitPwm(pwm);
        current_pwm = pwm;
        pwm = -1 * pwm;
        digitalWrite(pin_dir, HIGH);
        analogWrite(pin_pwm, pwm);
    }
    else
    {
        pwm = limitPwm(pwm);
        current_pwm = pwm;
        digitalWrite(pin_dir, LOW);
        analogWrite(pin_pwm, pwm);
    }
}

void MotorLib::Motor::setPwmLimits(int L_limit, int U_limit)
{
    pwm_l_limit = L_limit;
    pwm_u_limit = U_limit;
}

void MotorLib::Motor::setPID(double p, double i, double d)
{
    Kp = p;
    Ki = i;
    Kd = d;
}

void MotorLib::Motor::setpoint(long int setpoint)
{
    Controller_Setpoint = setpoint;
}

void MotorLib::Motor::runPosControlLoop()
{
    if (Mode == CLOSED_LOOP)
    {
        Controller_Input = encoderCount;
        Error = Controller_Setpoint - Controller_Input;
        diffError = (Error - prev_Error) * 1000 / CONTROLLER_LOOP_DTIME;
        sumError += Error * (CONTROLLER_LOOP_DTIME / 1000);
        prev_Error = Error;
        Controller_Output = Kp * Error + Ki * sumError + Kd * diffError;
        writeMotorPwm(Controller_Output);
    }
}

void MotorLib::Motor::printData()
{
    if (Mode != AUTOTUNE)
    {
        Serial.print(xTaskGetTickCount() * portTICK_PERIOD_MS);
        Serial.print(",");
        Serial.print(current_pwm);
        Serial.print(",");
        Serial.print(encoderCount);
        Serial.print(",");
        Serial.println(encoderVel);
    }
}

// TODO: Now we have to make sure that the motor variables are declared before the FreeRTOS scheduler starts;
// Allow for dynamic addition of Motor variables;
void TaskPosController(void *parameters)
{
    (void)parameters;

    MotorLib::motorStack *traverser_motorStack; // variable to go through the stack
    traverser_motorStack = top_motorStack;

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = (CONTROLLER_LOOP_DTIME / portTICK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        while (traverser_motorStack != NULL)
        {
            traverser_motorStack->motor->runPosControlLoop();
            traverser_motorStack = traverser_motorStack->next_motor;
        }
        traverser_motorStack = top_motorStack;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskCalculateVelocity(void *parameters)
{
    (void)parameters;
    MotorLib::motorStack *traverser_motorStack; // variable to go through the stack
    traverser_motorStack = top_motorStack;

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = (VELOCITY_LOOP_DTIME / portTICK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        while (traverser_motorStack != NULL)
        {
            traverser_motorStack->motor->calculateVelocity();
            traverser_motorStack = traverser_motorStack->next_motor;
        }
        traverser_motorStack = top_motorStack;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskPrintData(void *parameters)
{
    (void)parameters;

    MotorLib::motorStack *traverser_motorStack; // variable to go through the stack
    traverser_motorStack = top_motorStack;

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = (PRINTDATA_LOOP_DTIME / portTICK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        while (traverser_motorStack != NULL)
        {
            traverser_motorStack->motor->printData();
            traverser_motorStack = traverser_motorStack->next_motor;
        }
        traverser_motorStack = top_motorStack;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// TODO: As of now only one motor is supported at the time of Autotune; not limited to one programmatically
void TaskAutotune(void *parameters)
{
    MotorLib::Motor *autotune_motor = (MotorLib::Motor *)parameters;

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = (CONTROLLER_LOOP_DTIME / portTICK_PERIOD_MS);

    long int Output = 0;
    long int max_Output = 0;
    double dTime = 0;
    double prev_time = 0;
    int prevPwm = 0;
    long int min_Output = AUTOTUNE_SETPOINT;

    bool isAboveSetpoint = false;
    int setpointCrosses = 0;
    xLastWakeTime = xTaskGetTickCount();

    while (setpointCrosses != AUTOTUNE_NUMBER_OF_LOOPS)
    {
        Output = autotune_motor->getEncoderCount();
        prevPwm = autotune_motor->getPwm();
        if (isAboveSetpoint)
        {
            autotune_motor->writeMotorPwm(-1 * AUTOTUNE_RELAY_INPUT_AMPLITUDE);
        }
        else
            autotune_motor->writeMotorPwm(AUTOTUNE_RELAY_INPUT_AMPLITUDE);

        if (Output > max_Output)
            max_Output = Output;

        // Check for miniumum output once we cross the setpoint only or else lower values during initial starting time will become the min_Output value.
        if ((max_Output > AUTOTUNE_SETPOINT) && (Output < min_Output))
            min_Output = Output;

        if (Output > AUTOTUNE_SETPOINT)
            isAboveSetpoint = true;
        else
            isAboveSetpoint = false;

        if (((prevPwm < 0) && (autotune_motor->getPwm() > 0)) || ((prevPwm > 0) && (autotune_motor->getPwm() < 0)))
        {
            dTime = (xTaskGetTickCount() * portTICK_PERIOD_MS) - prev_time;
            prev_time = (xTaskGetTickCount() * portTICK_PERIOD_MS);
            setpointCrosses++;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    autotune_motor->writeMotorPwm(0);
    dTime = 2 * dTime / 1000;

    double Ku = (4 * 2 * AUTOTUNE_RELAY_INPUT_AMPLITUDE) / (PI * (max_Output - min_Output));

    // Print value to serial
    // Serial.print(dTime, DEC);
    // Serial.print(",");
    // Serial.print(Ku, DEC);
    // Serial.print(",");
    // Serial.print(min_Output);
    // Serial.print(",");
    // Serial.print(max_Output);
    // Serial.print(" ; ");
    Serial.print("Kp:");
    Serial.print(0.45 * Ku, DEC);
    Serial.print(",Ki:");
    Serial.print(0.54 * Ku / dTime, DEC);
    Serial.print(",Kd:");
    Serial.println(0);

    // autotune_motor->setPID(0.45 * Ku, 0.54 * Ku / dTime, 0); // PI Loop
    // autotune_motor->setMode(MotorLib::Motor::CLOSED_LOOP);

    vTaskDelete(NULL);
}

void MotorLib::initialize()
{
    xTaskCreate(TaskCalculateVelocity, (portCHAR *)"TaskCalculateVelocity", TASK_CALCULATEVELOCITY_STACKSIZE, NULL, 3, NULL);
    xTaskCreate(TaskPosController, (portCHAR *)"TaskPosController", TASK_CONTROLLER_STACKSIZE, NULL, 3, NULL);
    xTaskCreate(TaskPrintData, (portCHAR *)"TaskPrintData", TASK_PRINTDATA_STACKSIZE, NULL, 1, NULL);
}

void MotorLib::autoTune(MotorLib::Motor *autotune_motor)
{
    if (autotune_motor->Mode == MotorLib::Motor::AUTOTUNE)
        xTaskCreate(TaskAutotune, (portCHAR *)"TaskAutotune", TASK_AUTOTUNE_STACKSIZE, autotune_motor, 3, NULL);
}
