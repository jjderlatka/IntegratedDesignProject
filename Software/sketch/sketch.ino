#include <Servo.h>
#include <Adafruit_MotorShield.h>

using MotorShield = Adafruit_MotorShield;
using Motor = Adafruit_DCMotor;
using Time = unsigned long long;

enum Side {Left, Right};

enum Cube {Dense, Light};
// dense - blue box, green led

enum Task {WaitingForStart, 
            MovingForward, 
            Reversing, 
            Turning, 
            OpeningArms, 
            ClosingArms, 
            Detecting};

// Digital button class. Constructed by providing digital pin number
class Button {
private:
  int pin;
public:
  Button () = default;
  Button (int p) {
    pin = p;
    pinMode(pin, INPUT_PULLUP);
  }

  // Reading button state
  bool isPressed() {
    return digitalRead(pin);
  }
};

// Digital line sensor class. Constructed by providing digital pin number
class LineSensor {
private: 
  int pin;
public:
  LineSensor () = default;
  LineSensor (int p) {
    pin = p;
    pinMode(pin, INPUT);
  }

  // Reading line sensor state
  bool read() {
    return digitalRead(pin);
  }
};

// Analogue IR cube detector. Constructed by providing analogue pin number. Analogue threshold can be modified here 
class CubeDetector {
private:
  static const double threshold = 90;
  int reading, lastReading;
  int pin;
public:
  CubeDetector () = default;
  CubeDetector (int p) {
    pin = p;
    reading = -1;
    pinMode(pin, INPUT);
  }

  // To be run when arms are closed
  void grab() {
    if (analogRead(pin) > threshold) reading = Light;
    else reading = Dense;
  }

  // To be run when arms are opened
  void drop() {
    lastReading = reading;
    reading = -1;
  }

  // Accessing cube detection reading
  int getReading() {
    return reading;
  }

  // Accessing previous cube detection reading
  int getLastReading() {
    return lastReading;
  }
};

// Digital output LED
class LED {
private:
  int pin;
public:
  LED () = default;
  LED (int p) {
    pin = p;
    pinMode(pin, OUTPUT);
  }

  void turnOn() {
    digitalWrite(pin, HIGH);
  }

  void turnOff() {
    digitalWrite(pin, LOW);
  }
};

// Main class of the code
class Robot {
private:
  // Output objects
  MotorShield motorshield;
  Motor* motor[2];
  Servo armsServo;
  LED ambient, cubeType[2];
  // Input objects
  LineSensor lineSensor[2];
  CubeDetector cubeDetector;
  Button mainSwitch;
  // State variables
  int phase;
  Task task;
  // Task state variables
  bool line; // true if stop movement on intersection or stop rotation on line detection
  Time targetMovementTime; // time to drive forward or rotate ToDo: make an angle or distace
  Side rotationDirection;
  // Motors control
  int currentMotorSpeed[2];
  int maxSpeed = 255;
  // Motors constants
  Time effectIntervalMotor = 100; //ms
  int motorStep = 10;
  // Timing
  Time currentTime, previousTime; //ms
  Time startTime; //ms
  Time lastMotorUpdate; //ms
  // Rotation control
  int currentDirection, lastDirection, startDirection;
  // Intersection detection control
  bool currentlyOnIntersection, previouslyOnIntersection;
  // Arms movement control
  bool armsMoving;

  // CALIBRATION CONSTANTS
  Time armsMovingTime = 0.5e3;
  Time time180 = 3.2e3;
  Time time90 = 2e3;
  Time reversingTime = 2e3;
  // arms
  double closedAngle=70, openAngle=155;
  // times to be driven from last intersection to reach each cube
  Time cubeTime[3] = {13.5e3, 2.8e3, 2.2e3};
  Time dropOffTime[3] = {1.8e3, 1.2e3, 0.8e3};

  // Set motor on side 'side' to speed 'speed'
  void setMotorSpeed(Side side, int speed){
    if (speed < 0) speed = 0;
    if (speed > maxSpeed) speed = maxSpeed;
    currentMotorSpeed[side] = speed;
    lastMotorUpdate = currentTime;
    motor[side]->setSpeed(speed);
  }

  // Return 0 if the robot is aligned with the line, 
  // -1 if the robot is turning left (right sensor on the line)
  //  1 if the robot is turning right (left sensor on the line)
  int getDirection() {
    bool readingLeft = lineSensor[Left].read();
    bool readingRight = lineSensor[Right].read();
    // turning left
    if (!readingLeft && readingRight) return -1;
    // turning right
    if (readingLeft && !readingRight) return 1;
    // going straight
    return 0;
  }

public:
  // default constructor necessary to create global object Robot
  Robot() = default;
  // main constructor, taking all relevant pin numbers as arguments
  Robot (
    int leftMotorPort, 
    int rightMotorPort,
    int leftLineSensorPin,
    int rightLineSensorPin,
    int servoPin,
    int ambientLEDPin,
    int cubeTypeLEDPin1,
    int cubeTypeLEDPin2,
    int buttonPin,
    int cubePin) {
      // Motorshield & motors initalization
      motorshield = Adafruit_MotorShield();
      motor[Left] = motorshield.getMotor(leftMotorPort);
      motor[Right] = motorshield.getMotor(rightMotorPort);

      // Servo initialization
      if (servoPin==1) servoPin = 10;
      else if (servoPin==2) servoPin = 9;
      armsServo.attach(servoPin);

      // Line sensors initialization
      lineSensor[Left] = LineSensor(leftLineSensorPin);
      lineSensor[Right] = LineSensor(rightLineSensorPin);

      // Cube detector initialization
      cubeDetector = CubeDetector(cubePin);

      // LEDs initialization
      ambient = LED(ambientLEDPin);
      cubeType[Dense] = LED(cubeTypeLEDPin1);
      cubeType[Light] = LED(cubeTypeLEDPin2);

      // Main switch initialization
      mainSwitch = Button(buttonPin);

      // Catch motorshield errors
      if (!motorshield.begin()); // toDo: handle error

      // Initialize robot's state variables
      currentTime = 0; //ms
      startTime = 0; //ms
      lastMotorUpdate = 0; //ms

      currentMotorSpeed[Left] = 0;
      currentMotorSpeed[Right] = 0;

      currentDirection = 0;
      lastDirection = 0;

      currentlyOnIntersection = false;
      previouslyOnIntersection = false;

      armsMoving = false;

      // Set initial task and phase
      task = WaitingForStart;
      phase = 0;
  }

  // Drive forward with max speed
  void start() {
    startTime = currentTime;
    setMotorSpeed(Left, maxSpeed);
    setMotorSpeed(Right, maxSpeed);
    motor[Left]->run(FORWARD);
    motor[Right]->run(FORWARD);
  }

  // Reverse with max speed
  void reverse() {
    startTime = currentTime;
    setMotorSpeed(Left, maxSpeed);
    setMotorSpeed(Right, maxSpeed);
    motor[Left]->run(BACKWARD);
    motor[Right]->run(BACKWARD);
  }

  // Start rotation with max rotation speed, 
  // in the direction (left/right) specified in variable 'rotationDirection'
  void startRotation() {
    startTime = currentTime;
    startDirection = getDirection();
    setMotorSpeed(Left, maxSpeed);
    setMotorSpeed(Right, maxSpeed);
    if (rotationDirection==Left) {
      motor[Left]->run(BACKWARD);
      motor[Right]->run(FORWARD);
    } else {
      motor[Left]->run(FORWARD);
      motor[Right]->run(BACKWARD);
    }
  }

  // Stop both motors
  void stop() {
    setMotorSpeed(Left, 0);
    setMotorSpeed(Right, 0);
    motor[Left]->run(RELEASE);
    motor[Right]->run(RELEASE);
  }

  // Move arms to the closed angle
  void closeArms() {
    armsMoving = true;
    startTime = currentTime;
    armsServo.write(closedAngle);
  }

  // Move arms to the opened angle
  void openArms() {
    armsMoving = true;
    startTime = currentTime;
    armsServo.write(openAngle);
  }

  // Stop arms
  // required to update armsMoving variable, used by 'isMoving' function, to blink the ambient diode
  void stopArms() {
    armsMoving = false;
  }

  // Function to be run when arms are closed. cubeDetector is a private member hence the need for this function
  void grab() {
    cubeDetector.grab();
  }

  // Function to be run when arms are opened. cubeDetector is a private member hence the need for this function
  void drop() {
    cubeDetector.drop();
  }

  // Line following code
  void followLine() {
    int direction = getDirection();
    if (direction!=0 && currentTime-lastMotorUpdate > effectIntervalMotor) {
      if (direction==-1) setMotorSpeed(Right, currentMotorSpeed[Right]-motorStep);
      else if (direction==1) setMotorSpeed(Left, currentMotorSpeed[Left]-motorStep);
    } else if (direction==0) {
      setMotorSpeed(Left, maxSpeed);
      setMotorSpeed(Right, maxSpeed);
    }
  }

  // Reading button state. mainSwitch is a private member hence the need for this function
  bool buttonPressed() {
    return mainSwitch.isPressed();
  }

  // return whether the robot is moving or not. Movement defined as one of the motors moving or arms moving
  bool isMoving() {
    return currentMotorSpeed[Left] > 0 || currentMotorSpeed[Right] > 0 || armsMoving;
  }

  // Function to check whether the robot has returned to the line
  bool foundLine() {
    lastDirection = currentDirection;
    currentDirection = getDirection();

    if (currentDirection==0) {
      if (rotationDirection==Left) return lastDirection==1;
      else return lastDirection==-1;
    }
    return false;
  }

  // Function to check whether the robot has reached an interesection
  bool foundIntersection() {
    previouslyOnIntersection = currentlyOnIntersection;
    currentlyOnIntersection = lineSensor[Left].read() && lineSensor[Right].read();
    return currentlyOnIntersection && !previouslyOnIntersection;
  }

  // Function to check whether a given amount of time has elapsed.
  // It was supposed to be replaced with distance measurements, 
  // but there was no time to make calibrations neccessary for this change
  bool timedOut(Time targetTime) {
    return targetTime <= currentTime-startTime;
  }

  // Function to trigger robot stopping if the destianation
  // specified using variables 'line' and 'targetMovementTime' has been reached
  bool reachedLinearDestination() {
    if (line) return foundIntersection();
    else return timedOut(targetMovementTime);
  }

  // Function to trigger robot stopping if the destianation
  // specified using variables 'line' and 'targetMovementTime' has been reached
  bool reachedTurningDestination() {
    if (line) return foundLine() && timedOut(targetMovementTime);
    else return timedOut(targetMovementTime);
  }

  // Function to trigger robot stopping if the destianation
  // specified using variable 'targetMovementTime' has been reached
  bool reachedArmsDestination() {
    return timedOut(targetMovementTime);
  }

  // Return current task - private member
  Task getTask() {
    return task;
  }

  // Update Current time
  void updateTime() {
    previousTime = currentTime;
    currentTime = static_cast<Time>(millis());
  }

  // Blink the amber movememt led and switch cube type indicating leds
  void blinkLeds() {
    if (isMoving()) {
      ambient.turnOn();
    } else {
      ambient.turnOff();
    }
    if (cubeDetector.getReading()==-1) {
      cubeType[Dense].turnOff();
      cubeType[Light].turnOff();
    } else if (cubeDetector.getReading()==Dense) {
      cubeType[Dense].turnOn();
    } else if (cubeDetector.getReading()==Light) {
      cubeType[Light].turnOn();
    }
  }

  // ROBOT'S TASKS SEQUENCE
  // (please read this function after reading everything else in the code, especially the loop's function content)
  // Robot's movement and all other actions are set here. After a given phase has finished, 
  // next task and all its parameters are set. The parameters include mainly stop condition.

  // Target movement time is the time after which the robot is stopped if Line is set to false.
  // Line is set to true if the robot is to stop on intersection in linear momvement or 
  // stop when finds the line in rotation.
  // If Line is set to true, the line finding condition will be checked only after the targetMovementTime has elapsed
  void nextTask() {
    // START
    if (phase==0) { // on button press - > open arms
      task = OpeningArms;
      targetMovementTime = armsMovingTime;
    } else if (phase==1){ // arms opened - > go forward
      task = MovingForward;
      line = true;
    } else if (phase==2){ // box's intersection
      task = MovingForward;
      line = true;
    } 
    //  FIRST CUBE
    else if (phase==3){ // dropoff intersection
      task = MovingForward;
      line = false;
      // from the cube dropoff intersection, carry on for cubeTime[0] time
      targetMovementTime = cubeTime[0];
    } else if (phase==4){ // destination
      task = ClosingArms;
      targetMovementTime = armsMovingTime;
    } else if (phase==5){ // arms closed
      task = Detecting;
    } else if (phase==6) {
      task = MovingForward;
      line = false;
      targetMovementTime = 2.5e3;
    } else if (phase==7){ // made room for turn
      task = Turning;
      rotationDirection = Left;
      line = true;
      targetMovementTime = time180;
    } else if (phase==8){ // turned back
      task = OpeningArms;
      targetMovementTime = armsMovingTime;
    } else if (phase==9){ // opened arms
      task = MovingForward;
      line = true;
    }  else if (phase==10) { // on dropoff intersection
      task = MovingForward;
      line = false;
      targetMovementTime =1e3;
    } else if (phase==11) { // rear axis over dropoff intersection
      task = Turning;
      if (cubeDetector.getReading()==Dense) rotationDirection = Left;
      if (cubeDetector.getReading()==Light) rotationDirection = Right;
      line = false;
      targetMovementTime = time90;
    } else if (phase==12) { // turned into the box
      task = MovingForward;
      line = false;
      targetMovementTime = dropOffTime[0];
    } else if (phase==13) { // reached point of dropping the cube
      drop();
      task = Reversing;
      targetMovementTime = dropOffTime[0];
    } else if (phase==14) { // reversed
      task = Turning;
      if (cubeDetector.getLastReading()==Dense) rotationDirection = Left;
      if (cubeDetector.getLastReading()==Light) rotationDirection = Right;
      line = true;
      targetMovementTime = time90 - 0.5e3;
    }
    // SECOND CUBE
    else if (phase==15){ // turned back to the line
      task = MovingForward;
      line = true;
    } else if (phase==16){ // 1st cube intersection
      task = MovingForward;
      line = false;
      // from the 1st cube's intersection, carry on for cubeTime[1] time
      targetMovementTime = cubeTime[1];
    } else if (phase==17){ // destination
      task = ClosingArms;
      targetMovementTime = armsMovingTime;
    } else if (phase==18){ // arms closed
      task = Detecting;
    } else if (phase==19) {
      task = Reversing;
      line = false;
      targetMovementTime = 1e3;
    } else if (phase==20){ // made room for turn
      task = Turning;
      rotationDirection = Left;
      line = true;
      targetMovementTime = time180;
    } else if (phase==21){ // turned back
      task = OpeningArms;
      targetMovementTime = armsMovingTime;
    } else if (phase==22){ // opened arms
      task = MovingForward;
      line = true;
    }  else if (phase==23) { // on dropoff intersection
      task = MovingForward;
      line = false;
      targetMovementTime = 1e3;
    } else if (phase==24) { // rear axis over dropoff intersection
      task = Turning;
      if (cubeDetector.getReading()==Dense) rotationDirection = Left;
      if (cubeDetector.getReading()==Light) rotationDirection = Right;
      line = false;
      targetMovementTime = time90;
    } else if (phase==25) { // turned into the box
      task = MovingForward;
      line = false;
      targetMovementTime = dropOffTime[1];
    } else if (phase==26) { // reached point of dropping the cube
      drop();
      task = Reversing;
      targetMovementTime = dropOffTime[1];
    } else if (phase==27) { // reversed
      task = Turning;
      if (cubeDetector.getLastReading()==Dense) rotationDirection = Left;
      if (cubeDetector.getLastReading()==Light) rotationDirection = Right;
      line = true;
      targetMovementTime = time90 - 0.5e3;
    } 
    // THIRD CUBE
    else if (phase==28){ // turned back to the line
      task = MovingForward;
      line = true;
    } else if (phase==29){ // 1st cube intersection
      task = MovingForward;
      line = true;
    } else if (phase==30) { // end box intersection
      task = MovingForward;
      line = false;
      targetMovementTime = cubeTime[2];
    } else if (phase==31){ // destination
      task = ClosingArms;
      targetMovementTime = armsMovingTime;
    } else if (phase==32){ // arms closed
      task = Detecting;
    } else if (phase==33) {
      task = Reversing;
      line = false;
      targetMovementTime = reversingTime;
    } else if (phase==34){ // made room for turn
      task = Turning;
      rotationDirection = Left;
      line = true;
      targetMovementTime = time180;
    } else if (phase==35){ // turned back
      task = OpeningArms;
      targetMovementTime = armsMovingTime;
    } else if (phase==36){ // opened arms
      task = MovingForward;
      line = true;
    } else if (phase==37){ // 1st cube intersection
      task = MovingForward;
      line = true;
    } else if (phase==38) { // on dropoff intersection
      task = MovingForward;
      line = false;
      targetMovementTime = 1e3;
    } else if (phase==39) { // rear axis over dropoff intersection
      task = Turning;
      if (cubeDetector.getReading()==Dense) rotationDirection = Left;
      if (cubeDetector.getReading()==Light) rotationDirection = Right;
      line = false;
      targetMovementTime = time90;
    } else if (phase==40) { // turned into the box
      task = MovingForward;
      line = false;
      // from the third box's beginning, carry on for cubeTime[2] time
      targetMovementTime = dropOffTime[2];
    } else if (phase==41) { // reached point of dropping the cube
      drop();
      task = Reversing;
      targetMovementTime = dropOffTime[2];
    } else if (phase==42) { // reversed
      task = Turning;
      if (cubeDetector.getLastReading()==Dense) rotationDirection = Right;
      if (cubeDetector.getLastReading()==Light) rotationDirection = Left;
      line = true;
      targetMovementTime = time90 - 0.5e3;
    }
    // END
    else if (phase==43) {
      task = MovingForward;
      line = true;
    } else if (phase==44) {
      task = Turning;
      line = true;
      targetMovementTime = time180;
    } else if (phase==45) {
      task = Reversing;
      line = false;
      targetMovementTime = 4e3;
    } else if (phase==46) {
      task = WaitingForStart;
    }
    ++phase;
  }
};

//
// EXECUTION
//

Robot robot;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Porous a drink");
  // initialize robot object using the relevant pin numbers
  robot = Robot(1, 2, 11, 10, 2, 0, 12, 13, 7, 1, 2, 3);
}

// in the loop, exectue all functions that need to be run in every interation 
// and then run functions specific to currently performed task
void loop() {
  // every iteration:
  robot.updateTime();
  robot.blinkLeds();
  // currently performed task:
  Task task = robot.getTask();
  if (task == WaitingForStart) {
      robot.updateCubeDetector();
      if (robot.buttonPressed()) {
        robot.nextTask(); 
      }
  } else if (task == MovingForward) {
    if (!robot.isMoving()) {
      robot.start();
    } else if (!robot.reachedLinearDestination()) {
      robot.followLine();
    } else {
      robot.stop();
      robot.nextTask();
    }
  } else if (task == Reversing) {
    if (!robot.isMoving()) {
      robot.reverse();
    } else if (!robot.reachedLinearDestination()) {
      // carry on, do nothing
    } else {
      robot.stop();
      robot.nextTask();
    }
  } else if (task == Turning) {
    if (!robot.isMoving()) {
      robot.startRotation();
    } else if (!robot.reachedTurningDestination()) {
      // carry on, do nothing
    } else {
      robot.stop();
      robot.nextTask();
    }
  } else if (task == ClosingArms) {
    if (!robot.isMoving()) {
      robot.closeArms();
    } else if (robot.reachedArmsDestination()) {
      robot.stopArms();
      robot.nextTask();
    }
  } else if (task == OpeningArms) {
    if (!robot.isMoving()) {
      robot.openArms();
    } else if (robot.reachedArmsDestination()) {
      robot.stopArms();
      robot.nextTask();
    }
  } else if (task == Detecting) {
    robot.grab();
    robot.nextTask();
  }
}
