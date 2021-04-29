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
#define SENSOR0_ECHO        32
#define SENSOR0_TRIG        33
#define SENSOR1_ECHO        52
#define SENSOR1_TRIG        53
#define SENSOR2_ECHO        34
#define SENSOR2_TRIG        35
#define SENSOR3_ECHO        36 //back
#define SENSOR3_TRIG        37 //back
#define SENSOR4_ECHO        38 //left - top
#define SENSOR4_TRIG        39 //left - top
#define SENSOR5_ECHO        40 //right - top
#define SENSOR5_TRIG        41 //right - top

// Push Button Pins
#define BUTTON A0

// Joystick Threshold values
// These are the thresholds the joystick has to be moved beyond
#define JOYSTICK_HIGH_THRES 650
#define JOYSTICK_LOW_THRES  400
#define JOYSTICK_TURN_LEFT  425
#define JOYSTICK_TURN_RIGHT 600

// Turning limit for the wheels, because the motor will sheer them off
#define TURN_LIMIT          40

//stopping distance
#define DISTANCE            150
#define BACK_DIST           200
#define CAR_LENGTH          70

#define SENSOR_HOLD         5

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
int Accleration = 4;
int turningAccel = 2*Accleration;
int Deccleration = 8*Accleration;
int BreakingPower = Deccleration;
int MaxSpeed = 80;
int revSpeed = 75;
int MaxTurningSpeed = 100;
