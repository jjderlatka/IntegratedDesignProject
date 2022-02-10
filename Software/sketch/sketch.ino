#include <Servo.h>
#include <Adafruit_MotorShield.h>

using MotorShield = Adafruit_MotorShield;
using Motor = Adafruit_DCMotor;
using Time = unsigned long long;

enum Side {Left, Right};
// if movement time set to 0, use intersection
enum Phase {WaitingForStart, 
            MovingForward, 
            Reversing, 
            Turning, 
            OpeningArms, 
            ClosingArms, 
            Detect};

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
  Phase phase;
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
  // distance from the ramp
  double distanceFromRamp; // m
  // CALIBRATION CONSTANTS
  Time rampTime = 13405; //ms
  double flatSpeed = 0.09940028 * 1e-3; // m/ms
  // TODO: WRONG
  //double angularSpeed = 19.416 * 1e-3; // deg / ms
  double angularSpeed = 30 * 1e-3; // deg / ms
  // arms
  // double closedAngle=45, openAngle=135;
  double closedAngle=45, openAngle=135;
  // toDo: use distances instead
  Time cubeTime[3] = {24e3};

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

      phase = WaitingForStart;
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
    startTime = currentTime;
    armsServo.write(closedAngle);
  }

  void openArms() {
    startTime = currentTime;
    armsServo.write(openAngle);
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
    return currentMotorSpeed[Left] > 0 || currentMotorSpeed[Right] > 0;
  }

  bool foundLine(Side turningDirection) {
    lastDirection = currentDirection;
    currentDirection = getDirection();

    if (currentDirection==0) {
      if (turningDirection==Left) return lastDirection==1;
      else return lastDirection==-1;
    }
    return false;
  }

  bool foundIntersection() {
    bool readingLeft = lineSensor[Left].read();
    bool readingRight = lineSensor[Right].read();
    return readingLeft && readingRight;
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

  Phase getPhase() {
    return phase;
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

  // toDo: advance phases in correct order, set line and target movmement times
  void advancePhase() {
    switch (phase) {
      case WaitingForStart:
        phase = MovingForward;
        break;
      case MovingForward:
        phase = ClosingArms;
        break;
      case ClosingArms:
        phase = MovingForwrd2;
        break;
      case MovingForwrd2:
        phase = Turning;
        break;
      case Turning:
        phase = OpeningArms;
        break;
      case OpeningArms:
        phase = MovingBack;
        break;
      case MovingBack:
        phase = Turning2;
        break;
      case Turning2:
        phase = Moving3;
        break;
      case Moving3:
        phase = Reversing;
        break;
      case Reversing:
        phase = Turning3;
        break;
      case Turning3:
        phase = Moving4;
        break;
      case Moving4:
        phase = WaitingForStart;
        break;
    }
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
  Phase phase = robot.getPhase();
  if (phase == WaitingForStart) {
      Serial.println("Waiting for start");
      //if (robot.buttonOn()) {
      if (robot.timedOut(3e3)) {
        robot.openArms();
        robot.advancePhase();
      }
  } else if (phase == MovingForward) {
    if (!robot.isMoving()) {
      robot.start();
    } else if (!robot.reachedLinearDestination()) {
      robot.followLine();
    } else {
      robot.stop();
      robot.advancePhase();
    }
  } else if (phase == Reversing) {
    if (!robot.isMoving()) {
      robot.start();
    } else if (!robot.reachedLinearDestination()) {
      // carry on, do nothing
    } else {
      robot.stop();
      robot.advancePhase();
    }
  } else if (phase == Turning) {
    if (!robot.isMoving()) {
      robot.startRotation();
    } else if (!robot.reachedTurningDestination()) {
      // carry on, do nothing
    } else {
      robot.stop();
      robot.advancePhase();
    }
  } else if (phase == ClosingArms) {
    robot.closeArms();
    if (robot.reachedArmsDestination()) {
      robot.advancePhase();
    }
  } else if (phase == OpeningArms) {
    robot.openArms();
    if (robot.reachedArmsDestination()) {
      robot.advancePhase();
    }
  } else if (phase == Detect) {
    // toDo: implement detection
  }
}
