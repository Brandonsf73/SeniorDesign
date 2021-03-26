// Macros
#define lim_min(a,b) (((a) < (b) || (b) < 0) ? (a) : (b))
#define lim_max(a,b) (((a) < (b)) ? (b) : (a))

// Bluetooth Pins
#define BLUETOOTH_TX        11
#define BLUETOOTH_RX        10

// Analog Pins
#define JOYSTICK_XPOS       A0
#define JOYSTICK_YPOS       A1

// PWM pins
#define MOTOR1_PWM_CW       2
#define MOTOR1_PWM_CCW      3
#define MOTOR2_PWM_CW       4
#define MOTOR2_PWM_CCW      5
#define STEPPER_PWM_CW      6
#define STEPPER_PWM_CCW     7

// Sensor pins
#define SENSOR0_ECHO        30
#define SENSOR0_TRIG        31
#define SENSOR1_ECHO        32
#define SENSOR1_TRIG        33
#define SENSOR2_ECHO        34
#define SENSOR2_TRIG        35
#define SENSOR3_ECHO        36
#define SENSOR3_TRIG        37
#define SENSOR4_ECHO        38
#define SENSOR4_TRIG        39
#define SENSOR5_ECHO        40
#define SENSOR5_TRIG        41

// Push Button Pins
#define BUTTON A0

// Joystick Threshold values
// These are the thresholds the joystick has to be moved beyond
#define JOYSTICK_HIGH_THRES 550
#define JOYSTICK_LOW_THRES  480
#define JOYSTICK_TURN_LEFT 250
#define JOYSTICK_TURN_RIGHT 850

// How long we should not read joystick after we got a bluetooth command
#define BLUETOOTH_HOLD_TIME 50

// Bluetooth commands that we can recieve
typedef enum {
    Forward = 'F',
    Backward = 'B',
    Left = 'L',
    Right = 'R',
    Stop = 'S',
    Start = 'N',
} bluetoothCmd;

typedef enum {
    LeftCmd,
    RightCmd,
} turnCmd;

// Vairables for the car
int Accleration = 2;
int Deccleration = 2*Accleration;
int BreakingPower = 2*Deccleration;
int Distance = 35;
int MaxSpeed = 255;