#include <SoftwareSerial.h>
#include "Definitions.h"

// Rx, Tx pings for Bluetooth
SoftwareSerial bluetoothModule(BLUETOOTH_TX, BLUETOOTH_RX);

// Should the car be enabled or disabled based on Bluetooth
bool car_enabled = true;

// Speed values
signed int ReveseSpeed = 0;
signed int ForwardSpeed = 0;

// Bluetooth inputs will come suddenly and infrequently, and we don't want to suddently jolt the car
// Instead of sending data constantly we will get a single command and repeat it so many times
int BL_ForwCount = 0;
int BL_RevCount = 0;

//Define functions
bool SensorsDetectWall();
bool BluetoothControls();
void ReadJoystick();
void SetMotorForwardSpeed();
void SetMotorReveseSpeed();
void SetMotorIdle();
bool SetMotorTurning();

//boolean variables for distance sensors
bool LeftSensor = false;
bool CenterSensor = false;
bool RightSensor = false;
bool BackSensor = false;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    while(!Serial) { ; } // For using USB Serial

    // Setup the motor pins for output only
    pinMode(MOTOR1_PWM_CW, OUTPUT);
    pinMode(MOTOR1_PWM_CCW, OUTPUT);
    pinMode(MOTOR2_PWM_CW, OUTPUT);
    pinMode(MOTOR2_PWM_CCW, OUTPUT);
    pinMode(STEPPER_PWM_CW, OUTPUT);
    pinMode(STEPPER_PWM_CCW, OUTPUT);
   
    //Setup the Sensor echo pins as input and the triggers as output
    pinMode(SENSOR0_ECHO, INPUT);
    pinMode(SENSOR0_TRIG, OUTPUT);
    pinMode(SENSOR1_ECHO, INPUT);
    pinMode(SENSOR1_TRIG, OUTPUT);
    pinMode(SENSOR2_ECHO, INPUT);
    pinMode(SENSOR2_TRIG, OUTPUT);
    pinMode(SENSOR3_ECHO, INPUT);
    pinMode(SENSOR3_TRIG, OUTPUT);
    pinMode(SENSOR4_ECHO, INPUT);
    pinMode(SENSOR4_TRIG, OUTPUT);
    pinMode(SENSOR5_ECHO, INPUT);
    pinMode(SENSOR5_TRIG, OUTPUT);
    
    //Setup the button;
    pinMode(BUTTON, OUTPUT); // Generally, in push-button we take INPUT as a parameter but here we take OUTPUT because ANALOG PIN 

    digitalWrite(BUTTON, HIGH); // Make button condition HIGH


  
    Serial.println("App Started");
  
    //When it comes to the bluetooth this is going to be the biggest hickup
    //Depending on the module that is used this can be any baud rate
    //This was the baud rate for my Module, but their's might be 9600
    bluetoothModule.begin(115200);
}

void loop() {
    //Check sensors, if we are approaching a wall stop
    LeftSensor = SensorsDetectWall(SENSOR0_TRIG, SENSOR0_ECHO);
    CenterSensor = SensorsDetectWall(SENSOR1_TRIG, SENSOR1_ECHO);
    RightSensor = SensorsDetectWall(SENSOR2_TRIG, SENSOR2_ECHO);
    BackSensor = SensorsDetectWall(SENSOR3_TRIG, SENSOR3_ECHO);
   
    if(LeftSensor)
    {
      SetMotorIdle();
      return;
    }
    if(CenterSensor)
    {
      SetMotorIdle();
      return;
    }
    if(RightSensor)
    {
      SetMotorIdle();
      return;
    }

    // Attempt to read in values from bluetooth
    BluetoothControls();

    if(BL_ForwCount > 0)
    {
        Serial.println("Bluetooth: ForwardSpeed");
        BL_ForwCount--;   
        SetMotorForwardSpeed();
    }
    else if (BL_RevCount > 0)
    {
        Serial.println("Bluetooth: ReveseSpeed");
        BL_RevCount--;
        SetMotorReveseSpeed();
    }
    // Read from joystick if nothing else has stopped us
    else
    {
        Serial.println("Reading Joysticks");
        ReadJoystick();
        Serial.println("Reading Button");
        ReadButton();
    }
    // Let everything breath for a moment
    delay(50);
}


// Stop the motors if they start to detect a wall
bool SensorsDetectWall()
{
  float duration, sensorDistance;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  sensorDistance = (duration * .0343)/2;
  Serial.print("Distance: ");
  Serial.println(sensorDistance);
  if (sensorDistance < Distance)
  {
    return true;
  }
  return false;
}

// Read in the bluetooth commands and set the bluetooth values as needed
bool BluetoothControls()
{
    if(bluetoothModule.available())
    {
        // Read in the command from bluetooth and process the command
        bluetoothCmd data = (bluetoothCmd)bluetoothModule.read();
        switch(data){
            // For both ForwardSpeed and ReveseSpeed we are pulsing the car, basi
            case Forward:
                BL_ForwCount = 50;
                BL_RevCount = 0;
                break;
            case Backward:
                BL_RevCount = 50;
                BL_ForwCount = 0;
                break;
            case Left:
                SetMotorTurning();
                break;
            case Right:
                SetMotorTurning();
                break;
            case Stop:
                car_enabled = false;
                BL_ForwCount = 0;
                BL_RevCount = 0;
                break;
            case Start:
                car_enabled = true;
                BL_ForwCount = 0;
                BL_RevCount = 0;
                break;
            default:
                // Do nothing, garbage data
                break;
        }
        return true;
    }
    return false; 
}

// Read in the joystick values and set the motor valuse as needed
void ReadJoystick()
{
    int Joystick_xPos = analogRead(JOYSTICK_XPOS);
    int Joystick_yPos = analogRead(JOYSTICK_YPOS);

    // ForwardSpeed & Reverse
    if(Joystick_yPos > JOYSTICK_HIGH_THRES)
        SetMotorForwardSpeed();
    else if(Joystick_yPos < JOYSTICK_LOW_THRES)
        SetMotorReveseSpeed();
    else
        SetMotorIdle();

    // Turning
    // TODO
}

//read button
void ReadButton()
{
  if(digitalRead(BUTTON) == LOW)  // If button pressed
  {
    SetMotorForwardSpeed();
    Serial.println("Moving Forward");
  }
  else
  {
      SetMotorIdle();
      Serial.println("Stopping");
}

// Increase the ForwardSpeed speed of both of the motors and decrease their ReveseSpeed speed
void SetMotorForwardSpeed()
{
    ForwardSpeed = lim_min(255, ForwardSpeed+Accleration);
    ReveseSpeed = lim_min(0, ReveseSpeed-Deccleration);

    analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
    analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
    analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
    analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);
}

// Increase the ReveseSpeed speed of both of the motors and decrease their ForwardSpeed speed
void SetMotorReveseSpeed()
{
    ReveseSpeed = lim_min(255, ReveseSpeed+Accleration);
    ForwardSpeed = lim_min(0, ForwardSpeed-Deccleration);
    
    analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
    analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
    analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
    analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);
}

// Slowly break the motors, this prevents sudden breaking
void SetMotorIdle()
{
    ForwardSpeed = lim_max(0, (ForwardSpeed-BreakingPower));
    ReveseSpeed = lim_max(0, (ReveseSpeed-BreakingPower));

    analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
    analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);   
    analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
    analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
}

// Turn steering motor either right or left
bool SetMotorTurning()
{
    // TODO, still testing Stepper Motor
    return false;
}
