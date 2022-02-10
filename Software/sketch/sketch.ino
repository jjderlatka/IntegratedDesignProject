#include <Servo.h>
#include <Adafruit_MotorShield.h>

// todo 
// foudn line fix
// speed updates fix to make dist accurate

using MotorShield = Adafruit_MotorShield;
using Motor = Adafruit_DCMotor;
using Time = unsigned long long; // toDo: make sure no name collisions

enum Side {Left, Right};
enum Phase {WaitingForStart, MovingForward, ClosingArms, MovingForwrd2, Turning, OpeningArms, MovingBack, Turning2, Moving3};

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
  // calibration constants
  Time rampTime = 13405; //ms
  double flatSpeed = 0.09940028 * 1e-3; // m/ms
  // TODO: WRONG
  //double angularSpeed = 19.416 * 1e-3; // deg / ms
  double angularSpeed = 30 * 1e-3; // deg / ms
  // for the rotation
  int currentDirection, lastDirection, startDirection;
  // distance from the ramp
  double distanceFromRamp; // m
  // arms
  // double closedAngle=45, openAngle=135;
  double closedAngle=45, openAngle=135;

  void setMotorSpeed(Side side, int speed){
    // ToDo: catch if speed is a value outside of limits
    if (speed < 0) speed = 0;
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
  }

  void start() {
    // ToDo: implement steady acceleration
    // maybe add it in the set motor speed function
    // could save requested speed as a variable and move towards it gradually
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

  void startRotation(Side side) {
    startTime = currentTime;
    startDirection = getDirection();
    setMotorSpeed(Left, maxSpeed);
    setMotorSpeed(Right, maxSpeed);
    if (side==Left) {
      motor[Left]->run(BACKWARD);
      motor[Right]->run(FORWARD);
    } else {
      motor[Left]->run(FORWARD);
      motor[Right]->run(BACKWARD);
    }
  }

  void stop() {
    // ToDo: implement steady decceleration
    // as for acceleration
    setMotorSpeed(Left, 0);
    setMotorSpeed(Right, 0);
    motor[Left]->run(RELEASE);
    motor[Right]->run(RELEASE);
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

  bool foundLine(Side turningDirection) {
    lastDirection = currentDirection;
    currentDirection = getDirection();

    if (currentDirection==0) {
      if (turningDirection==Left) return lastDirection==1;
      else return lastDirection==-1;
    }

    //return currentDirection!=0;
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
    Serial.println((currentTime-startTime)*angularSpeed);
    Serial.println(deg);
    return (currentTime-startTime)*angularSpeed >= deg;
  }

  bool reachedDistance(double dist) {
    return distanceFromRamp >= dist;
  }

  Phase getPhase() {
    return phase;
  }

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
        phase = WaitingForStart;
        break;
    }
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

  void closeArms() {
    armsServo.write(closedAngle);
  }

  void openArms() {
    armsServo.write(openAngle);
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
  switch (robot.getPhase()) {
   // WaitingForStart, MovingForward, ClosingArms, MovingForwrd2, Turning, OpeningArms, MovingBack
   // find box - closing time
   // find forwward time
    case WaitingForStart:
      Serial.println("Waiting for start");
      delay(3000); // ToDo: remove
      robot.openArms();
      robot.start();
      robot.advancePhase();
      break;
    case MovingForward:
      Serial.println("Moving");
      //if (robot.reachedDistance(0.001)) {
      if (robot.timedOut(24e3)) {
        robot.stop();
        robot.closeArms();
        robot.advancePhase();
      } else {
        robot.followLine();
      }
      break;
    case ClosingArms:
      Serial.println("Closing arms");
      delay(2e3); // ToDo Remove
      robot.start();
      robot.advancePhase();
      break;
    case MovingForwrd2:
      if (robot.timedOut(4e3)) {
        robot.stop();
        robot.startRotation(Left);
        robot.advancePhase();
      } else {
        robot.followLine();
      }
      break;
    case Turning:
      Serial.println("Turning");
      if (robot.rotatedByAngle(100) && robot.foundLine(Left)) {
      //if (true) {
        robot.stop();
        robot.openArms();
        robot.advancePhase();
      }
      break;
    case OpeningArms:
      robot.start();
      robot.advancePhase();
      break;
    case MovingBack:
      Serial.println("Moving back");
      if (robot.timedOut(15e3)) {
      //if (true) {
        robot.stop();
        robot.startRotation(Left);
        robot.advancePhase();
      } else {
        robot.followLine();
      }
      break;
    case Turning2:
      if (robot.timedOut(2e3)) {
        robot.stop();
        robot.start();
        robot.advancePhase();
      }
      break;
    case Moving3:
      if (robot.timedOut(1e3)) {
        robot.stop();
      }
      break;
  }
}
