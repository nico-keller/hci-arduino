#include <Servo.h>
#include <Stepper.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Configuration
const int CLOCK_DELAY = 25;
const int MOTOR_STEP_DELAY = 50;

const int STEPPER_SPEED = 5;
const int STEP_PER_REVOLUTION = 1000;  // 180°
const int STEPPER_ABSOLUTE_FORWARD_LIMIT = 101;
const int STEPPER_ABSOLUTE_BACKWARD_LIMIT = -25;
const int STEPPER_FORWARD_ADJUSTMENT_LIMIT = 25;

const int SERVO_ABSOLUTE_FORWARD_LIMIT = 70;
const int SERVO_ABSOLUTE_BACKWARD_LIMIT = -70;
const int SERVO_ZERO_OFFSET = 63;

const int MAX_HUMAN_DISTANCE = 120;

const int joyStickUpper = 4;
const int joyStickLower = 2;

const int LIGHT_LEVEL_THRESHOLD = 500;


int delayTime = 1;
int relay = 3;
int SENDEN1 = 7, ECHO1 = 6;
int SENDEN2 = 5, ECHO2 = 4;
Servo servo_1;
int servo_pin = 8;
int potPin = A1; 
int joystick_X = A3, joystick_Y = A2, joystick_button = A4;
#define pirPin 13
#define photoResistorPin A0 // KY-018 photoresistor pin
int mode = 0; //0:off, 1:auto, 2:manual, 3:demo

Stepper myStepper(STEP_PER_REVOLUTION, 9, 11, 10, 12);

class StepperController{
  int currentPosition;
  public:
    StepperController() {
      currentPosition = 0;
    }
    int getStepperPosition(){
      return currentPosition;
    }
    void stepForward(){
      if(currentPosition>=STEPPER_ABSOLUTE_FORWARD_LIMIT){
        Serial.println("Stepper reached STEPPER_FORWARD_LIMIT");
        return;
      }
      myStepper.step(-STEPPER_SPEED);
      currentPosition++;
      delay(MOTOR_STEP_DELAY);
    }
    void stepBackward(){
      if(currentPosition<=STEPPER_ABSOLUTE_BACKWARD_LIMIT){
        Serial.println("Stepper reached STEPPER_BACkWARD_LIMIT");
        return;
      }
      myStepper.step(STEPPER_SPEED);
      currentPosition--;
      delay(MOTOR_STEP_DELAY);
    }
    bool hasExceededManualBounds(){
      return currentPosition>STEPPER_FORWARD_ADJUSTMENT_LIMIT;
    }
    bool moveTo(int desiredPosition){
      if(desiredPosition>=STEPPER_ABSOLUTE_FORWARD_LIMIT || desiredPosition<=STEPPER_ABSOLUTE_BACKWARD_LIMIT){
        Serial.println("Invalid input.");
        return false;
      }
      while(currentPosition != desiredPosition){
        if(currentPosition>desiredPosition){
          stepBackward();
        }
        else{
          stepForward();
        }
      }
      return true;
    }
    
};

class ServoController{
  int currentPosition;
  public:
    ServoController() {
      currentPosition = 0;
      servo_1.write(currentPosition + SERVO_ZERO_OFFSET);
    }
    int getServoPosition(){
      return currentPosition;
    }
    void stepForward(){
      if(currentPosition>=SERVO_ABSOLUTE_FORWARD_LIMIT){
        Serial.println("Servo reached SERVO_ABSOLUTE_FORWARD_LIMIT");
        return;
      }
      currentPosition+=1;
      servo_1.write(currentPosition + SERVO_ZERO_OFFSET);
      delay(MOTOR_STEP_DELAY);
    }
    void stepBackward(){
      if(currentPosition<=SERVO_ABSOLUTE_BACKWARD_LIMIT){
        Serial.println("Servo reached SERVO_ABSOLUTE_BACkWARD_LIMIT");
        return;
      }
      currentPosition-=1;
      servo_1.write(currentPosition + SERVO_ZERO_OFFSET);
      delay(MOTOR_STEP_DELAY);
    }
    bool moveTo(int desiredPosition){
      if(desiredPosition>=SERVO_ABSOLUTE_FORWARD_LIMIT || desiredPosition<=SERVO_ABSOLUTE_BACKWARD_LIMIT){
        Serial.println("Invalid input.");
        return false;
      }
      while(currentPosition != desiredPosition){
        if(currentPosition>desiredPosition){
          stepBackward();
        }
        else{
          stepForward();
        }
      }
      return true;
    }
    
};

void fatalError() {
    Serial.println("Fatal error occurred! Halting execution...");
    while (true) {
        // Optionally, flash an LED to signal the error
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }
}

StepperController stepperController;
ServoController servoController;

class MovementController{
  public: 
  bool isResting;
  MovementController(){
    isResting = false;
  }
  int getTilt(){
    return stepperController.getStepperPosition();
  }
  int getRotation(){
    return servoController.getServoPosition();
  }
  void move(int tilt, int rotate){
    if(tilt>STEPPER_FORWARD_ADJUSTMENT_LIMIT || isResting){
      return;
    }
    stepperController.moveTo(tilt);
    servoController.moveTo(rotate);
  }
  void rest(){
    if(isResting){
      return;
    }
    digitalWrite(relay, LOW);
    if(!servoController.moveTo(0)){
      Serial.println("An unaccounted error has occured. Shutting down.");
      fatalError();
    }
    if(!stepperController.moveTo(100)){
      Serial.println("An unaccounted error has occured. Shutting down.");
      fatalError();
    }
    isResting = true;
  }
  void wake(){
    if(!isResting){
      return;
    }
    if(!stepperController.moveTo(0)){
      Serial.println("An unaccounted error has occured. Shutting down.");
      fatalError();
    }
    isResting = false;
    digitalWrite(relay, HIGH);
  }

};

MovementController movementController;

long measureDistance(int sendenPin, int echoPin) {
  digitalWrite(sendenPin, LOW);
  delay(5);
  digitalWrite(sendenPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sendenPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  return (duration / 2) * 0.03432;
}

class MotionDirectionDetection {
  bool rightDetectedOnLastFrame;
  bool leftDetectedOnLastFrame;

  public:
  bool right;
  MotionDirectionDetection() {
    right = false;
    rightDetectedOnLastFrame = false;
    leftDetectedOnLastFrame = false;
  }

  void detect() {
    if (rightDetectedOnLastFrame && !leftDetectedOnLastFrame) {
      right = true;
    } 
    
    if (!rightDetectedOnLastFrame && leftDetectedOnLastFrame) {
      right = false;
    }

    rightDetectedOnLastFrame = measureDistance(SENDEN1, ECHO1) < MAX_HUMAN_DISTANCE;
    leftDetectedOnLastFrame = measureDistance(SENDEN2, ECHO2) < MAX_HUMAN_DISTANCE;
  }
};

MotionDirectionDetection motionDirectionDetection;


class HumanTracker{
  bool directionRight;
  bool detectedOnLastStep;
  public:
  int timeSinceDetection;
  HumanTracker(){
    directionRight = false;
    detectedOnLastStep = false;
    timeSinceDetection = 0;
  }
  void sweepStep(){
    if(movementController.isResting){
      return;
    }
    motionDirectionDetection.detect();
    if(humanPresent()){
      timeSinceDetection = 0;
      if(detectedOnLastStep){
        centreMassAdjustment();
      }
      detectedOnLastStep = true;
      return;
    }
    if(detectedOnLastStep){
      directionRight = motionDirectionDetection.right;
    }
    if(movementController.getRotation()+1>=SERVO_ABSOLUTE_FORWARD_LIMIT){
      directionRight = false;
    }
    if(movementController.getRotation()-1<=SERVO_ABSOLUTE_BACKWARD_LIMIT){
      directionRight = true;
    }
    if(directionRight){
      movementController.move(movementController.getTilt(),movementController.getRotation()+1);
    }
    else{
      movementController.move(movementController.getTilt(),movementController.getRotation()-1);
    }
    tiltTowardsZero();
    detectedOnLastStep = false;
    timeSinceDetection += 1;
  }
  bool humanPresent(){
    return measureDistance(SENDEN1, ECHO1) < MAX_HUMAN_DISTANCE && measureDistance(SENDEN2, ECHO2) < MAX_HUMAN_DISTANCE;
  }
  void centreMassAdjustment(){
    while(humanPresent()){
      movementController.move(movementController.getTilt()-1,movementController.getRotation());
    }
    while(!humanPresent()){
      if(movementController.getTilt()<=0){
        break;
      }
      movementController.move(movementController.getTilt()+1,movementController.getRotation());
    }
  }
  void tiltTowardsZero(){
    if(movementController.getTilt()+1<0){
      movementController.move(movementController.getTilt()+1,movementController.getRotation());
    }
  }
};

HumanTracker humanTracker;

void joyStickStep() {
  float temp_x = analogRead(joystick_X) * (5.0 / 1023.0);
  float temp_y = analogRead(joystick_Y) * (5.0 / 1023.0);

  if(temp_x<joyStickLower){
    movementController.move(movementController.getTilt(),movementController.getRotation()+1);
  }
  if(temp_x>joyStickUpper){
    movementController.move(movementController.getTilt(),movementController.getRotation()-1);
  }
  if(temp_y<joyStickLower){
    movementController.move(movementController.getTilt()+1,movementController.getRotation());
  }
  if(temp_y>joyStickUpper){
    movementController.move(movementController.getTilt()-1,movementController.getRotation());
  }
}

void autoCalibrate(){
  while(measureDistance(SENDEN2, ECHO2)>3){
    myStepper.step(-STEPPER_SPEED);
    delay(MOTOR_STEP_DELAY);
  }
  for(int i = 0; i<91; i++){
    myStepper.step(STEPPER_SPEED);
    delay(MOTOR_STEP_DELAY);
  }
}

bool detectMotion() {
  bool motionDetected = digitalRead(pirPin) == HIGH;
  if (motionDetected) {
    Serial.println("Motion detected! Scanning for human presence...");
  }
  return motionDetected;
}

bool isDark(){
  int lightLevel = analogRead(photoResistorPin);
  return (lightLevel>LIGHT_LEVEL_THRESHOLD);
}

void setup() {
  Serial.begin(9600);

  myStepper.setSpeed(5);
  servo_1.attach(servo_pin);

  pinMode(SENDEN1, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(SENDEN2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(pirPin, INPUT);
  pinMode(joystick_X, INPUT);
  pinMode(joystick_Y, INPUT);
  pinMode(joystick_button, INPUT);
  pinMode(photoResistorPin, INPUT); // Initialize photoresistor pin
  digitalWrite(joystick_button, HIGH);

  delay(2000);
  autoCalibrate();
  digitalWrite(relay, HIGH);
  
}


bool buttonPressed = false;

void loop() {
  
  //Mode switching
  if (digitalRead(joystick_button) == LOW && !buttonPressed) {
    mode=(mode+1)%4;
    buttonPressed = true;
    humanTracker.timeSinceDetection = 0;
  }
  if (digitalRead(joystick_button) == HIGH){
    buttonPressed = false;
  }
  Serial.println(mode);
  //Off mode
  if(mode==0){
    digitalWrite(relay, LOW);
    movementController.wake();
    movementController.move(0,0);
  }

  //Operation with optional sensors active
  if(mode==1){
    if(detectMotion()){
        humanTracker.timeSinceDetection=0;
      }
    if(humanTracker.timeSinceDetection>300 || isDark()){
      movementController.rest();
    }
    else{
      movementController.wake();
      humanTracker.sweepStep();
    }
  }

  //Manual adjustment mode
  if(mode==2){
    digitalWrite(relay, HIGH);
    joyStickStep();
  }

  //Operation without optional sensors active
  if(mode==3){
    digitalWrite(relay, HIGH);
    humanTracker.sweepStep();
  }
}