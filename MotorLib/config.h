/* Parameters required for MotorLib */

/* Task Parameters */
#define TASK_CONTROLLER_STACKSIZE 256
#define TASK_PRINTDATA_STACKSIZE 256
#define TASK_CALCULATEVELOCITY_STACKSIZE 64
#define TASK_AUTOTUNE_STACKSIZE 256

/* Loop Parameters */
#define CONTROLLER_LOOP_DTIME 16
#define PRINTDATA_LOOP_DTIME 32
#define VELOCITY_LOOP_DTIME 16

/* Autotune Paramters */
#define AUTOTUNE_RELAY_INPUT_AMPLITUDE 150  // Relay signal input amplitude
#define AUTOTUNE_SETPOINT 5000             // Setpoint for relay autotuning
#define AUTOTUNE_NUMBER_OF_LOOPS 20         // Limit on number of setpoint crosses