#include <Servo.h>
#include <Adafruit_MotorShield.h>

using MotorShield = Adafruit_MotorShield;
using Motor = Adafruit_DCMotor;
using Time = unsigned long long;

enum Side {Left, Right};

enum Task {WaitingForStart, 
            MovingForward, 
            Reversing, 
            Turning, 
            OpeningArms, 
            ClosingArms, 
            Detecting};

// toDo: add previous reading here so no need to store previous intersection and all of that
class LineSensor {
private: 
  int pin;
public:
  LineSensor () = default;
  LineSensor (int p) {
    pin = p;
    pinMode(pin, INPUT);
  }

  bool read() {
    return digitalRead(pin);
  }
};

class Robot {
private:
  // output
  MotorShield motorshield;
  Motor* motor[2];
  Servo armsServo;
  // input
  LineSensor lineSensor[2];
  // State variables
  int phase;
  Task task;
  int currentCube;
  // movement state variables
  bool line; // true if stop movement on intersection or stop rotation on line detection
  Time targetMovementTime; // time to drive forward or rotate ToDo: make an angle or distace
  Side rotationDirection;
  //
  int currentMotorSpeed[2];
  int maxSpeed = 255;
  // time
  Time currentTime, previousTime; //ms
  Time startTime; //ms
  Time lastMotorUpdate; //ms
  // control
  Time effectIntervalMotor = 100; //ms
  int motorStep = 10;
  // for the rotation
  int currentDirection, lastDirection, startDirection;
  // for intersection
  bool currentlyOnIntersection, previouslyOnIntersection;
  // for arms
  bool armsMoving;
  // distance from the ramp
  double distanceFromRamp; // m
  // CALIBRATION CONSTANTS
  Time armsMovingTime = 0.5e3;
  Time time180 = 3.5e3;
  Time time90 = 2.15e3;
  Time reversingTime = 3e3;
  // arms
  // double closedAngle=45, openAngle=135;
  double closedAngle=45, openAngle=135;
  // toDo: use distances instead
  Time cubeTime[3] = {14.5e3, 1e3, 1e3};
  Time dropOffTime[3] = {3e3, 2e3, 1e3};

  void setMotorSpeed(Side side, int speed){
    if (speed < 0) speed = 0;
    if (speed > maxSpeed) speed = maxSpeed;
    currentMotorSpeed[side] = speed;
    lastMotorUpdate = currentTime;
    motor[side]->setSpeed(speed);
  }

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
  Robot() = default;
  Robot (
    int leftMotorPort, 
    int rightMotorPort,
    int leftLineSensorPin,
    int rightLineSensorPin,
    int servoPin) {
      // think about breaking this into several functions for clarity
      motorshield = Adafruit_MotorShield();
      motor[Left] = motorshield.getMotor(leftMotorPort);
      motor[Right] = motorshield.getMotor(rightMotorPort);

      if (servoPin==1) servoPin = 10;
      else if (servoPin==2) servoPin = 9;
      armsServo.attach(servoPin);

      lineSensor[Left] = LineSensor(leftLineSensorPin);
      lineSensor[Right] = LineSensor(rightLineSensorPin);

      if (!motorshield.begin()); // toDo: handle error

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

      task = WaitingForStart;
      phase = 0;
      currentCube = 1;
  }

  void start() {
    startTime = currentTime;
    setMotorSpeed(Left, maxSpeed);
    setMotorSpeed(Right, maxSpeed);
    motor[Left]->run(FORWARD);
    motor[Right]->run(FORWARD);
  }

  void reverse() {
    startTime = currentTime;
    setMotorSpeed(Left, maxSpeed);
    setMotorSpeed(Right, maxSpeed);
    motor[Left]->run(BACKWARD);
    motor[Right]->run(BACKWARD);
  }

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

  void stop() {
    setMotorSpeed(Left, 0);
    setMotorSpeed(Right, 0);
    motor[Left]->run(RELEASE);
    motor[Right]->run(RELEASE);
  }

  void closeArms() {
    armsMoving = true;
    startTime = currentTime;
    armsServo.write(closedAngle);
  }

  void openArms() {
    armsMoving = true;
    startTime = currentTime;
    armsServo.write(openAngle);
  }

  void stopArms() {
    armsMoving = false;
  }

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

  bool isMoving() {
    return currentMotorSpeed[Left] > 0 || currentMotorSpeed[Right] > 0 || armsMoving;
  }

  bool foundLine() {
    lastDirection = currentDirection;
    currentDirection = getDirection();

    if (currentDirection==0) {
      if (rotationDirection==Left) return lastDirection==1;
      else return lastDirection==-1;
    }
    return false;
  }

  bool foundIntersection() {
    previouslyOnIntersection = currentlyOnIntersection;
    currentlyOnIntersection = lineSensor[Left].read() && lineSensor[Right].read();
    return currentlyOnIntersection && !previouslyOnIntersection;
  }

  bool timedOut(Time targetTime) {
    return targetTime <= currentTime-startTime;
  }

  bool rotatedByAngle(double deg) {
    return (currentTime-startTime)*angularSpeed >= deg;
  }

  bool reachedDistance(double dist) {
    return distanceFromRamp >= dist;
  }

  bool reachedLinearDestination() {
    if (line) return foundIntersection();
    else return timedOut(targetMovementTime);
  }

  bool reachedTurningDestination() {
    if (line) return foundLine() && timedOut(targetMovementTime);
    else return timedOut(targetMovementTime);
  }

  bool reachedArmsDestination() {
    return timedOut(targetMovementTime);
  }

  Task getTask() {
    return task;
  }

  int getCurrentCube() {
    return currentCube;
  }

  void updateTime() {
    previousTime = currentTime;
    currentTime = static_cast<Time>(millis());
  }

  void updatePosition() {
    // ToDo: discuss with the team
    if (currentTime-startTime <= rampTime) distanceFromRamp = 0;
    else {
      double currentSpeed = flatSpeed * (min(currentMotorSpeed[Left], currentMotorSpeed[Right])/maxSpeed);
      distanceFromRamp += (currentTime-previousTime) * currentSpeed;
    }
  }

  void blinkLeds() {
    // ToDo: implement
    if (isMoving()) {
      //blink
    }
  }

  void nextTask() {
    // START
    if (phase==0) { // on button press
      task = OpeningArms;
      targetMovementTime = armsMovingTime;
    } else if (phase==1){ // arms opened
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
      targetMovementTime = cubeTime[0];
    } else if (phase==4){ // destination
      task = ClosingArms;
      targetMovementTime = armsMovingTime;
    } else if (phase==5){ // arms closed
      task = Detecting;
    } else if (phase==6) {
      task = MovingForward;
      line = false;
      targetMovementTime = 3e3;
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
      targetMovementTime = 1.75e3;
    } else if (phase==11) { // rear axis over dropoff intersection
      task = Turning;
      rotationDirection = Right;
      line = false;
      targetMovementTime = time90;
    } else if (phase==12) { // turned into the box
      task = MovingForward;
      line = false;
      targetMovementTime = dropOffTime[0];
    } else if (phase==13) { // reached point of dropping the cube
      task = Reversing;
      targetMovementTime = dropOffTime[0];
    } else if (phase==14) { // reversed
      task = Turning;
      rotationDirection = Right;
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
      targetMovementTime = cubeTime[1];
    } else if (phase==17){ // destination
      task = ClosingArms;
      targetMovementTime = armsMovingTime;
    } else if (phase==18){ // arms closed
      task = Detecting;
    } else if (phase==19) {
      task = Reversing;
      line = false;
      targetMovementTime = reversingTime;
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
      targetMovementTime = 1.75e3;
    } else if (phase==24) { // rear axis over dropoff intersection
      task = Turning;
      rotationDirection = Right;
      line = false;
      targetMovementTime = time90;
    } else if (phase==25) { // turned into the box
      task = MovingForward;
      line = false;
      targetMovementTime = dropOffTime[1];
    } else if (phase==26) { // reached point of dropping the cube
      task = Reversing;
      targetMovementTime = dropOffTime[1];
    } else if (phase==27) { // reversed
      task = Turning;
      rotationDirection = Right;
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
      targetMovementTime = 1.75e3;
    } else if (phase==39) { // rear axis over dropoff intersection
      task = Turning;
      rotationDirection = Right;
      line = false;
      targetMovementTime = time90;
    } else if (phase==40) { // turned into the box
      task = MovingForward;
      line = false;
      targetMovementTime = dropOffTime[2];
    } else if (phase==41) { // reached point of dropping the cube
      task = Reversing;
      targetMovementTime = dropOffTime[2];
    } else if (phase==42) { // reversed
      task = Turning;
      rotationDirection = Right;
      line = true;
      targetMovementTime = time90 - 0.5e3;
    } else if (phase==43) { // turned back to the line
      task = Reversing;
      line = true;
    }
    // END
    else if (phase==44) {
      task = Turning;
      rotationDirection = Left;
      line = true;
      targetMovementTime = 3.5e3;
    } else if (phase==45) {
      task = MovingForward;
      line = true;
    } else if (phase==46) {
      task = Turning;
      line = true;
      targetMovementTime = 3.5e3;
    } else if (phase==47) {
      task = Reversing;
      line = false;
      targetMovementTime = 4.5e3;
    } else if (phase==48) {
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
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Porous a drink");
  robot = Robot(3, 4, 12, 13, 1);
}

void loop() {
  robot.updateTime();
  robot.updatePosition();
  robot.blinkLeds();
  Task task = robot.getTask();
  if (task == WaitingForStart) {
      //if (robot.buttonOn()) {
      if (robot.timedOut(3e3)) {
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
    // toDo: implement detection
  }
}
