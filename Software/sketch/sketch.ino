#include <Servo.h>
#include <Adafruit_MotorShield.h>

using MotorShield = Adafruit_MotorShield;
using Motor = Adafruit_DCMotor;

enum Side {left, right};

class Robot {
private:
  // output
  MotorShield motorshield;
  Motor* motors[2];
  Servo armsServo;
  // input
  // -
  // State variables

  void setServoAngle(int angle) {
    // ToDo: throw exception if angle out of limits
    // for arms, take account of the gearing, 
    // using a gearing config variable
    armsServo.write(angle);
  }

  void setMotorSpeed(Side side, int speed){
    // ToDo: catch if speed is a value outside of limits
    motor[side]->setSpeed(speed);
  }

public:
  Robot ();
  void initialize(int servoPin, 
    int leftMotorPort, 
    int rightMotorPort) {
      motorshield = Adafruit_MotorShield();
      motor[left] = motorshield.getMotor(leftMotorPort);
      motor[right] = motorshield.getMotor(rightMotorPort);
      armsServo.attach(servoPin);
      
      if (!robot.motorshield.begin()) throw;
  }
};

//
// EXECUTION
//

Robot robot;

void setup() {
  // put your setup code here, to run once:
  robot.initialize(1, 2, 3);
}

void loop() {
}


/*

enum Phase {movementForward, armsClosing, movementBack, rotation, armsOpening};

class Robot {
private:
  int currentCube = 1;
  Phase currentPhase;
  bool leftLineSensorOK, rightLineSensorOK;
public:
  Robot() {
    currentCube = 1;
    currentPhase = movementForward;
  }
  void followLine();
  bool reachedDestination();
};

//EXECUTION

Robot robot;

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!robot.reachedDestination()) {
    robot.followLine();
  } else {
    
  }

}
*/