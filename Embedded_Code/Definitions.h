// Macros
#define lim_min(a,b) (((a) < (b) || (b) < 0) ? (a) : (b))
#define lim_max(a,b) (((a) < (b)) ? (b) : (a))

// Bluetooth Pins
#define BLUETOOTH_TX        11
#define BLUETOOTH_RX        10

// Analog Pins
#define JOYSTICK_XPOS       A0
#define JOYSTICK_YPOS       A1
#define BUTTON              A0

// PWM pins
#define MOTOR1_PWM_CW       2
#define MOTOR1_PWM_CCW      3
#define MOTOR2_PWM_CW       4
#define MOTOR2_PWM_CCW      5
#define STEPPER_DIR         7
#define STEPPER_PUL         6

// Sensor pins
#define SENSOR0_ECHO        30
#define SENSOR0_TRIG        31
#define SENSOR1_ECHO        32
#define SENSOR1_TRIG        33
#define SENSOR2_ECHO        34
#define SENSOR2_TRIG        35
#define SENSOR3_ECHO        36  //back
#define SENSOR3_TRIG        37  //back
#define SENSOR4_ECHO        38  //left - top
#define SENSOR4_TRIG        39  //left - top
#define SENSOR5_ECHO        40  //right - top
#define SENSOR5_TRIG        41  //right - top

// Push Button Pins
#define BUTTON A0

// Joystick Threshold values
// These are the thresholds the joystick has to be moved beyond
#define JOYSTICK_HIGH_THRES 700
#define JOYSTICK_LOW_THRES  300
#define JOYSTICK_TURN_LEFT  120
#define JOYSTICK_TURN_RIGHT 1000

// Turning limit for the wheels, because the motor will sheer them off
#define TURN_LIMIT          40

//stopping distance
#define DISTANCE            60

//car length
#define CAR_LENGTH          150

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
    LeftCmd=0,
    RightCmd=1,
} turnCmd;

// Vairables for the car
int Accleration = 2;
int Deccleration = 2*Accleration;
int BreakingPower = 2*Deccleration;
int Distance = 35;
int MaxSpeed = 255;
