#include <Servo.h>
#include <Adafruit_MotorShield.h>

using MotorShield = Adafruit_MotorShield;
using Motor = Adafruit_DCMotor;
using Time = unsigned long long; // toDo: make sure no name collisions

enum Side {Left, Right};
enum Phase {WaitingForStart, Moving, Stopped};

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
  LineSensor lineSensor[2];
  // State variables
  Phase phase;
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
  // calibration constants
  Time rampTime = 13405; //ms
  double flatSpeed = 0.09940028 * 1e-3; // m/ms


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
  Robot (/*int servoPin,*/
    int leftMotorPort, 
    int rightMotorPort,
    int leftLineSensorPin,
    int rightLineSensorPin) {
      // think about breaking this into several functions for clarity
      motorshield = Adafruit_MotorShield();
      motor[Left] = motorshield.getMotor(leftMotorPort);
      motor[Right] = motorshield.getMotor(rightMotorPort);
      //armsServo.attach(servoPin);

      lineSensor[Left] = LineSensor(leftLineSensorPin);
      lineSensor[Right] = LineSensor(rightLineSensorPin);

      if (!motorshield.begin()); // toDo: handle error

      currentTime = 0; //ms
      startTime = 0; //ms
      lastMotorUpdate = 0; //ms
      effectIntervalMotor = 100; //ms

      currentMotorSpeed[Left] = 0;
      currentMotorSpeed[Right] = 0;
      currentPosition = 0; // mm

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
    Serial.println("Start");
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

  bool reachedDestination() {
    /*int targetTime = 60 * 1000; //10s
    return targetTime <= currentTime-startTime;*/
    double targetDistance = 0.5; //m (behind the ramp beginning)
    Time targetTime = rampTime + (Time)(targetDistance / flatSpeed);
    return targetTime <= currentTime-startTime;
  }

  Phase getPhase() {
    Serial.println(phase);
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
};

//
// EXECUTION
//

Robot robot;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Porous a drink");
  robot = Robot(3, 4, 2, 4);
}

void loop() {
  robot.updateTime();
  switch (robot.getPhase()) {
    case WaitingForStart:
      Serial.println("Waiting for start");
      delay(1000); // ToDo: remove
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
      break;
  }
}
