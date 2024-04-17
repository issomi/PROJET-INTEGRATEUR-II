#include <SoftwareSerial.h> // SoftwareSerial for serial communication with HM10 bluetooth module.
#include <ArduinoBlue.h> // ArduinoBlue bluetooth library
#include <Wire.h> 
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> // Library for servo motor
#include <Stepper.h> // Library for stepper motor

// shield -----------------------------------------------------------------------------------------------------
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *Rmotor = AFMS.getMotor(3);
Adafruit_DCMotor *Lmotor = AFMS.getMotor(4);
Adafruit_DCMotor *Rmagnet = AFMS.getMotor(1);
Adafruit_DCMotor *Lmagnet = AFMS.getMotor(2);

//bluetooth -----------------------------------------------------------------------------------------------------
const int speed = 110;
const int BLUETOOTH_TX = 53;
const int BLUETOOTH_RX = 52;
SoftwareSerial softSerial(BLUETOOTH_TX, BLUETOOTH_RX); // Object for serial communication to HM 10 bluetooth module using digital pins.
ArduinoBlue phone(softSerial); 

//Servo declare -----------------------------------------------------------------------------------------------------
int servoPin = 9;
int angle, button;
String  str;
Servo Servo;

//Stepper declare -----------------------------------------------------------------------------------------------------
// Number of steps per output rotation
const int stepsPerRevolution = 200;

// Create Instance of Stepper library
Stepper myStepper(stepsPerRevolution, 22, 24, 26, 28);

// Driving function -----------------------------------------------------------------------------------------------------



void motorBrake() {
Rmotor -> run(RELEASE);
Lmotor -> run(RELEASE);
  // Do not write the motor speeds on this function. It simply configures the motor controller.
}
void motorSetForward() {
Rmotor -> run(FORWARD);
Lmotor -> run(FORWARD);
  // Do not write the motor speeds on this function. It simply configures the motor controller.
}
void motorSetBackward() {
Rmotor -> run(BACKWARD);
Lmotor -> run(BACKWARD);
  // Do not write the motor speeds on this function. It simply configures the motor controller.
}

void turn360Droite(){
Rmotor -> run(BACKWARD);
Lmotor -> run(FORWARD);
}

void turn360Gauche(){
Rmotor -> run(FORWARD);
Lmotor -> run(BACKWARD);
}

void driveControl() {
  // THROTTLE AND STEERING CONTROL
  // throttle values after subtracting 49:
  //     50 = max forward throttle
  //     0 = no throttle
  //     -49 = max reverse throttle
  // steering values after subtracting 49:
  //     50 = max right
  //     0 = straight
  //     -49 = max left
  int throttle = phone.getThrottle() - 49;
  int steering = phone.getSteering() - 49;
  if (throttle == 0 && steering == 0) {
    // If the throttle is zero, don't move.
    motorBrake();
    return;
  }
  // Determine forwards or backwards.
  if (throttle > 0) {
    // Forward
    motorSetForward();
  }
  else if (throttle < 0) {
    // Backward
    motorSetBackward();
  } else if (steering > 0) {
    turn360Droite();
   
  } else if (steering < 0) {
    turn360Gauche();
  }
}


// Magnet function -----------------------------------------------------------------------------------------------------

void magnetOn(){
  Rmagnet -> run(FORWARD);
  Lmagnet -> run(FORWARD);
}
void magnetOff(){
  Rmagnet -> run(RELEASE);
  Lmagnet -> run(RELEASE);
}


//setup -----------------------------------------------------------------------------------------------------
void setup() {
 AFMS.begin();
 

 Rmagnet -> setSpeed(255);
 Lmagnet -> setSpeed(255);
 Rmotor -> setSpeed(100);
 Lmotor -> setSpeed (100);


 Rmagnet -> run(RELEASE);
 Lmagnet -> run(RELEASE);

 delay(500);
  // Start communication with HM10 bluetooth module.
  softSerial.begin(9600);
  Serial.begin(9600);
  myStepper.setSpeed(60);
  Servo.attach(servoPin);
  Serial.print("setup complete");

  Servo.write(0);
  delay(1000);
  Servo.detach();

}



//Loop -----------------------------------------------------------------------------------------------------
int rotations = 0;
void loop() { 
  // put your main code here, to run repeatedly:

// ############################### driving stuff  ##############################################


driveControl();
// ############################### get button and slider info  ##############################################




int sliderId = phone.getSliderId();
  int sliderVal = phone.getSliderVal();
  int bouton = phone.getButton();
// store button info in a seperate value
  if (bouton != -1){
    button = bouton;
  }



// ############################### slider to servo info  ##############################################
  str = phone.getText();
  if(sliderId == 2){
    Servo.attach(servoPin);
    Servo.write(map(sliderVal, 0,100, 0 , 180));
    delay(500);
    Servo.detach();
   
  }




// ############################### buttons to stepper info  ##############################################
  if (button == 5 && rotations == 0){
    rotations = 10 ;
    button = -1;
    Servo.detach();

  }
   if (button == 6 && rotations == 0){
     rotations = -10;
     button = -1;
     Servo.detach();
 }
    if (button == 7  && rotations != 0){
    rotations = 0;
    myStepper.step(0);
    button = -1;
    Servo.detach();
    
  }
 

  if (rotations > 0){
        myStepper.step(200);
        rotations--;
        delay(50);
  }
  
 if (rotations < 0){
  myStepper.step(-200);
  rotations ++;
  delay(50);

 }
    if (button == 7  && rotations != 0){
    rotations = 0;
    myStepper.step(0);
    button = -1;
    Servo.detach();
  }
 
//############################### magnet control ##############################################
if (button == 0){
  magnetOn();
  button = -1;
}
if (button == 1){
  magnetOff();
  button = -1;
}

//##################################Servo detach####################################
if (button == 9){
  Servo.detach();
}
}
