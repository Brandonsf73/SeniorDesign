#include <SoftwareSerial.h>
#include "Definitions.h"

// Rx, Tx pings for Bluetooth
SoftwareSerial bluetoothModule(BLUETOOTH_TX, BLUETOOTH_RX);

// Should the car be enabled or disabled based on Bluetooth
bool car_enabled = true;

// Speed values
signed int Motor1_ForwardSpeed = 0;
signed int Motor2_ForwardSpeed = 0;
signed int Motor1_ReverseSpeed = 0;
signed int Motor2_ReverseSpeed = 0;

// Turning values
int TurningTalue = 0;

//Define functions
bool SensorsDetectWall(int trigPin, int echoPin);
bool BluetoothControls();
void ReadJoystick();
void SetMotorForwardSpeed(bool forward);
void SetMotorReverseSpeed(bool reverse);
void SetMotorIdle();
bool SetMotorTurning();
void SetMotorForwardIdle();
void SetMotorReverseIdle();
bool SensorsTurn(int trigPin, int echoPin);

//boolean variables for distance sensors
bool LeftSensor = false;
bool CenterSensor = false;
bool RightSensor = false;
bool BackSensor = false;

//variable to check if the kid moved the joystick 0 -no, 1-left, 2-right
int did_child_turn = 0;

bool forwardAllow = true;
bool reverseAllow = true;
bool leftAllow = true;
bool rightAllow = true;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    while(!Serial) { ; } // For using USB Serial

    // Setup the motor pins for output only
    pinMode(MOTOR1_PWM_CW, OUTPUT);
    pinMode(MOTOR1_PWM_CCW, OUTPUT);
    pinMode(MOTOR2_PWM_CW, OUTPUT);
    pinMode(MOTOR2_PWM_CCW, OUTPUT);
    pinMode(STEPPER_DIR, OUTPUT);
    pinMode(STEPPER_PUL, OUTPUT);
   
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
    //pinMode(BUTTON, OUTPUT); // Generally, in push-button we take INPUT as a parameter but here we take OUTPUT because ANALOG PIN 
    //digitalWrite(BUTTON, HIGH); // Make button condition HIGH
  
    Serial.println("App Started");
  
    //When it comes to the bluetooth this is going to be the biggest hickup
    //Depending on the module that is used this can be any baud rate
    //This was the baud rate for my Module, but their's might be 9600
    bluetoothModule.begin(9600);
}

void loop() {
    // If the car is disabled, check for input from bluetooth and loop again
    if(!car_enabled)
    {
        Serial.println("Car disabled");
        SetMotorIdle();
        BluetoothControls();
        delayMicroseconds(100);
        return;
    }
    //Check sensors, if we are approaching a wall stop
    LeftSensor = SensorsDetectWall(SENSOR0_TRIG, SENSOR0_ECHO);
    CenterSensor = SensorsDetectWall(SENSOR1_TRIG, SENSOR1_ECHO);
    RightSensor = SensorsDetectWall(SENSOR2_TRIG, SENSOR2_ECHO);
    BackSensor = SensorsDetectWall(SENSOR3_TRIG, SENSOR3_ECHO);
    bool RightSensorTurn = SensorsTurn(SENSOR2_TRIG, SENSOR2_ECHO);
    bool LeftSensorTurn = SensorsTurn(SENSOR0_TRIG, SENSOR0_ECHO);

    leftAllow = true;
    rightAllow = true;
    forwardAllow = true;
    reverseAllow = true;
    if(CenterSensor) //obstacle in front of car
    {
      forwardAllow = false;
      leftAllow = false;
      rightAllow = false;
      bluetoothCmd parentData = (bluetoothCmd)bluetoothModule.read();
      did_child_turn = ReadJoystickTurn();
      
      if(!LeftSensor) //left side of car is free
      {
        Serial.println("entering first if statment");
        if ((parentData == Left) || (did_child_turn == 1)) //check if parent or child moved car left
        {
          forwardAllow = true;
          leftAllow = true;
          Serial.println("Let's move left");
        }
      }

       if(!RightSensor) //right side of car is free
      {
        if ((parentData == Right) || (did_child_turn == 2)) //check if parent or child moved car right
        forwardAllow = true;
        rightAllow = true;
        Serial.println("Let's move right");
      }

      if((!leftAllow) && (!rightAllow))
      {
          SetMotorForwardIdle();
      }
    } //end sensor if

    if(BackSensor)
    {
      reverseAllow = false;
      SetMotorReverseIdle();
    }

    // Attempt to read in values from bluetooth
    // If we get nothing from bluetooth use the joystick/button as input
    if( !BluetoothControls() )
    {
        Serial.println("Reading Analog Input");
        ReadJoystick();
        //ReadButton();
    }
    
    // Let everything breath for a moment
    delayMicroseconds(50);
}


// Stop the motors if they start to detect a wall
bool SensorsDetectWall(int trigPin, int echoPin)
{
    float duration, sensorDistance;
    
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH,20000);
    sensorDistance = (duration * .0343)/2;
    Serial.print("Distance: ");
    Serial.println(sensorDistance);
    if (sensorDistance < DISTANCE)
    {
      if(sensorDistance == 0){
        return false;
      }
      else{
        return true;
      }
    }
    return false;
}


//if the car is stopped, and we need to turn left or right check if les than 150
//if greater than 150, return true (meaning the car can make the zero radius turn)
bool SensorsTurn(int trigPin, int echoPin)
{
    float duration, sensorDistance;
    
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH,20000);
    sensorDistance = (duration * .0343)/2;
    Serial.print("Distance: ");
    Serial.println(sensorDistance);
    if (sensorDistance < CAR_LENGTH)
    {
      if(sensorDistance == 0){
        return true;
      }
      else{
        return false;
      }
    }
    return true;
}
//148 cm

// Read in the bluetooth commands and set the bluetooth values as needed
bool BluetoothControls()
{
    if(bluetoothModule.available())
    {
        // Read in the command from bluetooth and process the command
        bluetoothCmd data = (bluetoothCmd)bluetoothModule.read();
        Serial.print("Bluetooth data:");
        Serial.println((char)data);
        switch(data)
        {
            // For both ForwardSpeed and ReverseSpeed we are pulsing the car
            // This is in case data is not constantly coming in
            case Forward:
                SetMotorForwardSpeed(forwardAllow);
                return true;
            case Backward:
                SetMotorReverseSpeed(reverseAllow);
                return true;
            case Left:
                SetMotorTurning(LeftCmd);
                return true;
            case Right:
                SetMotorTurning(RightCmd);
                return true;
            case Stop:
                car_enabled = false;
                return true;
            case Start:
                car_enabled = true;
                return true;
            default:
                // Do nothing, garbage data
                return false;
        }
    }
    return false; 
}

// Read in the joystick values and set the motor valuse as needed
void ReadJoystick()
{
    int Joystick_xPos = analogRead(JOYSTICK_XPOS);
    int Joystick_yPos = analogRead(JOYSTICK_YPOS);
    bool changedForward = false;
    bool changedReverse = false;
    Serial.print("x pos: ");
    Serial.println(Joystick_xPos);
    Serial.print("y pos: ");
    Serial.println(Joystick_yPos);

    // ForwardSpeed & Reverse
    if(Joystick_yPos > JOYSTICK_HIGH_THRES)
        SetMotorForwardSpeed(forwardAllow);
    else if(Joystick_yPos < JOYSTICK_LOW_THRES)
        SetMotorReverseSpeed(reverseAllow);
    else
        changedForward = true;

    // Turning 
    if(Joystick_xPos < JOYSTICK_TURN_LEFT)
    {
        SetMotorTurning(LeftCmd);
    }
    else if(Joystick_xPos > JOYSTICK_TURN_RIGHT)
    {
        SetMotorTurning(RightCmd);
    }
    else
      changedReverse = true;

    if(changedForward && changedReverse)
    {
      SetMotorIdle();;
    }
}

// Read in the joystick values to see if we turned right or left
int ReadJoystickTurn()
{
    int Joystick_xPos = analogRead(JOYSTICK_XPOS);
    int Joystick_yPos = analogRead(JOYSTICK_YPOS);

    // Turning 
    if(Joystick_xPos < JOYSTICK_TURN_LEFT)
    {
      return 1;
    }
    else if(Joystick_xPos > JOYSTICK_TURN_RIGHT)
    {
      return 2;
    }
    else
    {
      return 0;
    }
}

//read button
void ReadButton()
{
    if(digitalRead(BUTTON) == LOW)  // If button pressed
    {
      SetMotorForwardSpeed(forwardAllow);
    }
    else
    {
        SetMotorIdle();
    }
}

// Increase the ForwardSpeed speed of both of the motors and decrease their ReverseSpeed speed
void SetMotorForwardSpeed(bool forward)
{
    if(!forward)
  {
    Serial.println("can't move forward");
  }

  else{
    Serial.println("Forwarding");
    Motor1_ForwardSpeed = lim_min(MaxSpeed, Motor1_ForwardSpeed+Accleration);
    Motor2_ForwardSpeed = lim_min(MaxSpeed, Motor2_ForwardSpeed+Accleration);
    Motor1_ReverseSpeed = lim_max(0, Motor1_ReverseSpeed-Deccleration);
    Motor2_ReverseSpeed = lim_max(0, Motor2_ReverseSpeed-Deccleration);

    analogWrite(MOTOR1_PWM_CW, Motor1_ForwardSpeed);
    analogWrite(MOTOR1_PWM_CCW, Motor1_ReverseSpeed);
    analogWrite(MOTOR2_PWM_CW, Motor2_ReverseSpeed);
    analogWrite(MOTOR2_PWM_CCW, Motor2_ForwardSpeed);
  }
}

// Increase the ReverseSpeed speed of both of the motors and decrease their ForwardSpeed speed
void SetMotorReverseSpeed(bool reverse)
{
    if(!reverse)
  {
    return;
  }
  else{
    Serial.println("Reversing");
    Motor1_ReverseSpeed = lim_min(MaxSpeed, Motor1_ReverseSpeed+Accleration);
    Motor2_ReverseSpeed = lim_min(MaxSpeed, Motor2_ReverseSpeed+Accleration);
    Motor1_ForwardSpeed = lim_max(0, Motor1_ForwardSpeed-Deccleration);
    Motor2_ForwardSpeed = lim_max(0, Motor2_ForwardSpeed-Deccleration);
    
    analogWrite(MOTOR1_PWM_CW, Motor1_ForwardSpeed);
    analogWrite(MOTOR1_PWM_CCW, Motor1_ReverseSpeed);
    analogWrite(MOTOR2_PWM_CW, Motor2_ReverseSpeed);
    analogWrite(MOTOR2_PWM_CCW, Motor2_ForwardSpeed);
  }
}

// Slowly break the motors, this prevents sudden breaking
void SetMotorIdle()
{
    Serial.println("Motor idling");
    Motor1_ForwardSpeed = lim_max(0, (Motor1_ForwardSpeed-Deccleration));
    Motor2_ForwardSpeed = lim_max(0, (Motor2_ForwardSpeed-Deccleration));
    Motor1_ReverseSpeed = lim_max(0, (Motor1_ReverseSpeed-Deccleration));
    Motor2_ReverseSpeed = lim_max(0, (Motor2_ReverseSpeed-Deccleration));

    analogWrite(MOTOR1_PWM_CW, Motor1_ForwardSpeed);
    analogWrite(MOTOR2_PWM_CCW, Motor2_ForwardSpeed);   
    analogWrite(MOTOR2_PWM_CW, Motor2_ReverseSpeed);
    analogWrite(MOTOR1_PWM_CCW, Motor1_ReverseSpeed);
}

// Slowly break the motor, this prevents sudden breaking
void SetMotorForwardIdle()
{
    // Serial.println("Motor idling");
    Motor1_ForwardSpeed = lim_max(0, (Motor1_ForwardSpeed-Deccleration));
    Motor2_ForwardSpeed = lim_max(0, (Motor2_ForwardSpeed-Deccleration));
    //ReveseSpeed = lim_max(0, (ReveseSpeed-Deccleration));

    analogWrite(MOTOR1_PWM_CW, Motor1_ForwardSpeed);
    analogWrite(MOTOR2_PWM_CCW, Motor2_ForwardSpeed);   
    //  analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
    //  analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
}

// Slowly break the motor, this prevents sudden breaking
void SetMotorReverseIdle()
{
    // Serial.println("Motor idling");
    //ForwardSpeed = lim_max(0, (ForwardSpeed-Deccleration));
    Motor1_ReverseSpeed = lim_max(0, (Motor1_ReverseSpeed-Deccleration));
    Motor2_ReverseSpeed = lim_max(0, (Motor2_ReverseSpeed-Deccleration));

   // analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
   // analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);   
    analogWrite(MOTOR2_PWM_CW, Motor2_ReverseSpeed);
    analogWrite(MOTOR1_PWM_CCW, Motor1_ReverseSpeed);
}

// Turn steering motor either right or left
bool SetMotorTurning(turnCmd turn)
{
    int count = 10;
    switch(turn)
    {
      case LeftCmd:
        if(!leftAllow)
        {
          return false;
        }
          Serial.println("Motor Turning LEFT");
          // Rotate the stepper motor once Left
          Motor1_ForwardSpeed = lim_min(MaxSpeed, Motor1_ForwardSpeed+Accleration);
          Motor1_ReverseSpeed = lim_max(0, Motor1_ReverseSpeed-Deccleration);
          Motor2_ReverseSpeed = lim_min(MaxSpeed, Motor2_ReverseSpeed+Accleration);
          Motor2_ForwardSpeed = lim_max(0, Motor2_ForwardSpeed-Deccleration);
          Serial.print("Motor1 Reverse: ");
          Serial.println(Motor1_ForwardSpeed);
          Serial.print("Motor2 Forward: ");
          Serial.println(Motor2_ReverseSpeed);
          
          analogWrite(MOTOR1_PWM_CW, Motor1_ForwardSpeed);
          analogWrite(MOTOR1_PWM_CCW, Motor1_ReverseSpeed);
          analogWrite(MOTOR2_PWM_CW, Motor2_ReverseSpeed);
          analogWrite(MOTOR2_PWM_CCW, Motor2_ForwardSpeed);
          return true;
      case RightCmd:
        if(!rightAllow)
        {
          return false;
        }
          Serial.println("Motor Turning RIGHT");
          // Rotate the stepper motor once Right
          Motor1_ReverseSpeed = lim_min(MaxSpeed, Motor1_ReverseSpeed+Accleration);
          Motor1_ForwardSpeed = lim_max(0, Motor1_ForwardSpeed-Deccleration);
          Motor2_ForwardSpeed = lim_min(MaxSpeed, Motor2_ForwardSpeed+Accleration);
          Motor2_ReverseSpeed = lim_max(0, Motor2_ReverseSpeed-Deccleration);
          Serial.print("Motor1 Reverse: ");
          Serial.println(Motor1_ReverseSpeed);
          Serial.print("Motor2 Forward: ");
          Serial.println(Motor2_ForwardSpeed);
      
          analogWrite(MOTOR1_PWM_CW, Motor1_ForwardSpeed);
          analogWrite(MOTOR1_PWM_CCW, Motor1_ReverseSpeed);
          analogWrite(MOTOR2_PWM_CW, Motor2_ReverseSpeed);
          analogWrite(MOTOR2_PWM_CCW, Motor2_ForwardSpeed);
          return true;
      default:
          // Garbage data, do nothing
          Serial.print("Garbage: ");
          Serial.println(turn);
          return false;
    }
}
