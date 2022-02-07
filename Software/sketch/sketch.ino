#include <Servo.h>
#include <Adafruit_MotorShield.h>

using MotorShield = Adafruit_MotorShield;
using Motor = Adafruit_DCMotor;
using Time = unsigned long long; // toDo: make sure no name collisions

enum Side {Left, Right};
enum Phase {WaitingForStart, Moving, Stopped};
enum Direction {Backward, Forward};

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
  // Servo armsServo;
  // input
  LineSensor lineSensor[2][2];
  // State variables
  Phase phase;
  Direction movementDirection;
  //
  int currentMotorSpeed[2];
  int maxSpeed = 255;
  int currentPosition; //mm
  // time
  Time currentTime; //ms
  Time startTime; //ms
  Time lastMotorUpdate; //ms
  // control
  Time effectIntervalMotor; //ms
  int motorStep = 10;


  void updatePosition() {
    // ToDo: discuss with the team
  }

  // void setServoAngle(int angle) {
  //  // ToDo: throw exception if angle out of limits
  //  // for arms, take account of the gearing, 
  //  // using a gearing config variable
  //  armsServo.write(angle);
  // }

  void setMotorSpeed(Side side, int speed){
    // ToDo: catch if speed is a value outside of limits
    if (speed < 0) speed = 0;
    if (speed > maxSpeed) speed = maxSpeed;
    currentMotorSpeed[side] = speed;
    lastMotorUpdate = currentTime;
    motor[side]->setSpeed(speed);
  }

  int getAlignment(Direction direction) {
    bool readingLeft = lineSensor[direction][Left].read();
    bool readingRight = lineSensor[direction][Right].read();
    // turning left
    if (!readingLeft && readingRight) return -1;
    // turning right
    if (readingLeft && !readingRight) return 1;
    // going straight
    return 0;
  }

public:
  Robot() = default;
  Robot (/*int servoPin,*/
    int leftMotorPort, 
    int rightMotorPort,
    int frontLeftLineSensorPin,
    int frontRightLineSensorPin,
    int rearLeftLineSensorPin,
    int rearRightLineSensorPin) {
      // think about breaking this into several functions for clarity
      motorshield = Adafruit_MotorShield();
      motor[Left] = motorshield.getMotor(leftMotorPort);
      motor[Right] = motorshield.getMotor(rightMotorPort);
      //armsServo.attach(servoPin);

      lineSensor[Forward][Left] = LineSensor(frontLeftLineSensorPin);
      lineSensor[Forward][Right] = LineSensor(frontRightLineSensorPin);
      lineSensor[Backward][Left] = LineSensor(rearLeftLineSensorPin);
      lineSensor[Backward][Right] = LineSensor(rearRightLineSensorPin);

      if (!motorshield.begin()); // toDo: handle error

      currentTime = 0; //ms
      startTime = 0; //ms
      lastMotorUpdate = 0; //ms
      effectIntervalMotor = 100; //ms

      currentMotorSpeed[Left] = 0;
      currentMotorSpeed[Right] = 0;
      currentPosition = 0; // mm

      phase = WaitingForStart;
      movementDirection = Backward; // because its switched in the loop
  }

  void start() {
    // ToDo: implement steady acceleration
    // maybe add it in the set motor speed function
    // could save requested speed as a variable and move towards it gradually
    startTime = currentTime;
    setMotorSpeed(Left, maxSpeed);
    setMotorSpeed(Right, maxSpeed);
    if (movementDirection == Forward) {
      motor[Left]->run(FORWARD);
      motor[Right]->run(FORWARD);
    } else {
      motor[Left]->run(BACKWARD);
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
    int alignment = getAlignment(movementDirection);
    // not aligned
    if (alignment!=0 && currentTime-lastMotorUpdate > effectIntervalMotor) {
        if (alignment==-1) setMotorSpeed(Right, currentMotorSpeed[Right]-motorStep);
        else if (alignment==1) setMotorSpeed(Left, currentMotorSpeed[Left]-motorStep);
    // aligned
    } else if (alignment==0) {
      setMotorSpeed(Left, maxSpeed);
      setMotorSpeed(Right, maxSpeed);
    }
  }

  bool reachedDestination() {
    int targetTime = 10 * 1000; //10s
    return targetTime <= currentTime-startTime;
  }

  Phase getPhase() {
    return phase;
  }

  void advancePhase() {
    switch (phase) {
      case WaitingForStart:
        phase = Moving;
        break;
      case Moving:
        phase = Stopped;
        break;
      case Stopped:
        phase = WaitingForStart;
        break;
    }
  }

  void updateTime() {
    currentTime = static_cast<Time>(millis());
  }

  void switchDirection() {
    if (movementDirection==Forward) movementDirection=Backward;
    else movementDirection=Forward;
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
  robot = Robot(3, 4, 2, 4, 12, 13);
}

void loop() {
  robot.updateTime();
  switch (robot.getPhase()) {
    case WaitingForStart:
      Serial.println("Waiting for start");
      delay(1000); // ToDo: remove
      robot.switchDirection();
      robot.start();
      robot.advancePhase();
      break;
    case Moving:
      Serial.println("Moving");
      if (!robot.reachedDestination()) robot.followLine();
      else robot.advancePhase();
      break;
    case Stopped:
      Serial.println("Stopped");
      robot.stop();
      delay(1000); // ToDo: remove
      robot.advancePhase();
      break;
  }
}
