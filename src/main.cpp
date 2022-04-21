/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       19480                                                     */
/*    Created:      Wed Apr 20 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

const float WHEEL_DIAMETER = 10.16 * 0.01; // m
const float WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;
// const float rotConst = 0;
// const float liftRatio = 0;
const float speed = 50;
// bool taskk = 0;
// bool up = 1;
// float diag_wheel_dist = 45;
// float robot_circumference = 0.5*2*3.1415926536*diag_wheel_dist;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;

// define your global instances of motors and other devices here
competition Competition;
controller Controller1 = vex::controller();
motor TLMotor = vex::motor(vex::PORT8);
motor TRMotor = vex::motor(vex::PORT10, true);
motor BLMotor = vex::motor(vex::PORT7);
motor BRMotor = motor(vex::PORT9, true);
motor arm1 = motor(vex::PORT11, gearSetting::ratio36_1);
motor arm2 = motor(vex::PORT12, gearSetting::ratio36_1, true);
motor conveyor = motor(PORT13);
pneumatics pist1 = pneumatics(Brain.ThreeWirePort.A);
pneumatics pist2 = pneumatics(Brain.ThreeWirePort.B);
pneumatics pist3 = pneumatics(Brain.ThreeWirePort.C);

void L1();
void R1();
void R2();
void X();
float setspeed = 1;

/*________________________________________________________AUTONOMOUS________________________________________________________*/


//counterclockwise
void move(float dist) {

  TLMotor.spinFor(-dist/WHEEL_CIRCUMFERENCE, rev, 150, velocityUnits::pct, true);
  TRMotor.spinFor(dist/WHEEL_CIRCUMFERENCE, rev, 150, velocityUnits::pct, true);
  BLMotor.spinFor(-dist/WHEEL_CIRCUMFERENCE, rev, 150, velocityUnits::pct, false);
  BRMotor.spinFor(dist/WHEEL_CIRCUMFERENCE, rev, 150, velocityUnits::pct, false);

}

float robot_diam = 21.2602916255 * 2.54 * 0.01;
float robot_circum = M_PI * robot_diam;
// 14x16 inches, 21.2602916255 inches diam from TR to BL wheels

void turn(float angle) {

  TLMotor.spinFor((angle/360) * robot_circum, rev, 150, velocityUnits::pct, true);
  TRMotor.spinFor((angle/360) * robot_circum, rev, 150, velocityUnits::pct, true);
  BLMotor.spinFor((angle/360) * robot_circum, rev, 150, velocityUnits::pct, false);
  BRMotor.spinFor((angle/360) * robot_circum, rev, 150, velocityUnits::pct, false);

}

void swingturn(float angle, bool t_state) {

  if (t_state) {
    TLMotor.spinFor((angle/360) * robot_circum, rev, 150, velocityUnits::pct, true);
    TRMotor.spinFor((angle/360) * robot_circum, rev, 100, velocityUnits::pct, true);
    BLMotor.spinFor((angle/360) * robot_circum, rev, 150, velocityUnits::pct, false);
    BRMotor.spinFor((angle/360) * robot_circum, rev, 100, velocityUnits::pct, false);
  }
  
  else {
    TLMotor.spinFor((angle/360) * robot_circum, rev, 100, velocityUnits::pct, true);
    TRMotor.spinFor((angle/360) * robot_circum, rev, 150, velocityUnits::pct, true);
    BLMotor.spinFor((angle/360) * robot_circum, rev, 100, velocityUnits::pct, false);
    BRMotor.spinFor((angle/360) * robot_circum, rev, 150, velocityUnits::pct, false);
  }
}

void rev_swingturn(float angle, bool t_state) {

  if (t_state) {
    TLMotor.spinFor(-(angle/360) * robot_circum, rev, 150, velocityUnits::pct, true);
    TRMotor.spinFor(-(angle/360) * robot_circum, rev, 100, velocityUnits::pct, true);
    BLMotor.spinFor(-(angle/360) * robot_circum, rev, 150, velocityUnits::pct, false);
    BRMotor.spinFor(-(angle/360) * robot_circum, rev, 100, velocityUnits::pct, false);
  }
  
  else {
    TLMotor.spinFor(-(angle/360) * robot_circum, rev, 100, velocityUnits::pct, true);
    TRMotor.spinFor(-(angle/360) * robot_circum, rev, 150, velocityUnits::pct, true);
    BLMotor.spinFor(-(angle/360) * robot_circum, rev, 100, velocityUnits::pct, false);
    BRMotor.spinFor(-(angle/360) * robot_circum, rev, 150, velocityUnits::pct, false);
  }
}


void lift(float angle, bool state1) {
  if (state1) {
    arm1.spinFor(100, deg, false);
    arm2.spinFor(100, deg, false);
    Controller1.Screen.clearScreen();
    Controller1.Screen.print(state1);
    state1 = 0;
  }

  //move down
  else {
    arm1.spinFor(-100, deg, false);
    arm2.spinFor(-100, deg, false);
    Controller1.Screen.clearScreen();
    Controller1.Screen.print(state1);
    state1 = 1;
  }
}

int signnum_c(int x) {
  if (x < 0) {
    return -1;
  }
  else if (x > 0) {
    return 1;
  }
  return 0;
}

//settings
double kP = 0;
double kI = 0;
double kD = 0;

double t_kP = 0;
double t_kI = 0;
double t_kD = 0;

//autonomous settings
int desiredValue = 200;
//change in time between sensor data
float dT = 20;

int error = 0; //positional error
int preverror = 0; //position 20 ms ago
int derivative; //error - preverror
int totalerror; //sum of error
int maxTurnIntegral = 300;
int maxIntegral = 300;
int integralBound;

//turn movement
int turnDesiredValue = 200;

int turnError = 0; //positional error
int turnPreverror = 0; //position 20 ms ago
int turnDerivative; //error - preverror
int turnTotalError; //sum of error

double motorPower;
double turnMotorPower;

bool resetDriveSensors = false;

//var for use

bool enabledrivePID = true;
int drivePID() {

  while (enabledrivePID) {

    if (resetDriveSensors) {
       
      resetDriveSensors = false;
      TLMotor.setRotation(0, deg);
      TRMotor.setRotation(0, deg);
    }
     
    else

    //get position of motor

    //undefined identifier if this line isn't here... may just be my editor...
    int rightMotorPos = TRMotor.rotation(deg);

    /*------------lateral movement PID------------*/
    //get average of two motor

    int leftMotorPos = TLMotor.rotation(deg);
    int rightMotorPos = TRMotor.rotation(deg);

    int averagePosition;
    averagePosition = (leftMotorPos + rightMotorPos)/2;

    //Proportional
    error = averagePosition - desiredValue;

    //Derivative
    derivative = error - preverror;

    //Velocity -> position -> absement = Integral
    if (abs(error) < integralBound) {
      totalerror += error;
    }

    else {
      totalerror = integralBound - 0.1;
    }

    if (error == 0) {
      totalerror = 0;
    }

    if (abs(totalerror) > maxIntegral) {
      totalerror = signnum_c(totalerror) * maxIntegral;
    }
    //make sure integral doesn't exceed limit

    //kD and kI are better represented as kP/T, and kP*T
    double lateralMotorPower = kP * error + kD * derivative + kI * totalerror;



    /*------------turn movement PID------------*/
    //get difference of wheel turn

    int turnDifference = leftMotorPos - rightMotorPos;

    //Proportional
    turnError = turnDifference - turnDesiredValue;
 
    //Derivative
    turnDerivative = turnError - turnPreverror;

    //Velocity -> position -> absement = Integral
    if (abs(error) < integralBound) {
      turnTotalError += turnError;
    }

    else {
      turnTotalError = integralBound - 0.1;
    }

    if (turnError == 0) {
      turnTotalError = 0;
    }

    if (abs(turnTotalError) > maxTurnIntegral) {
      turnTotalError = signnum_c(totalerror) * maxTurnIntegral;
    }

    double turnMotorPower = t_kP * turnError + t_kD * turnDerivative + t_kI * turnTotalError; 

    TRMotor.spin(fwd, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    TLMotor.spin(fwd, lateralMotorPower - turnMotorPower, voltageUnits::volt);

    preverror = error;
    vex::task::sleep(dT);

  }
  return 1;
}

void auton(void) {
  rev_swingturn(setspeed*50.0, false);
  move(setspeed*50);
  R2(); //grip base
  move(-setspeed*10);
  turn(setspeed*50);
  lift(10, true); //raise arm
  move(setspeed*50);
  setspeed = 0.5;
  move(setspeed*50);
  turn(-setspeed*50);
  lift(-1, false); //lower arm on base
  R2(); //ungrip
  lift(1, true); //move arm up a bit  
  lift(-50, false); //bring arm down
  turn(setspeed*50);

}

void  preautonn(void){


}

/*________________________________________________________DRIVER________________________________________________________*/

bool state1 = 1;
bool convey_state = 1;

void L1() {
  //pneumatics and conveyor

  if (state1) {
    pist2.open();
    pist3.open();
    state1 = 0;
  }
  else {
    pist2.close();
    pist3.close();
    state1 = 1;
  }

  convey_state = !convey_state;
  Controller1.Screen.print("L1");
  Controller1.Screen.clearScreen();
  
}

bool state2 = 1;

void R1() {
  //arm

  //move up
  if (state2) {
    arm1.spinFor(650, deg, 100, velocityUnits::pct, false);
    arm2.spinFor(650, deg, 100, velocityUnits::pct, false);
    Controller1.Screen.clearScreen();
    Controller1.Screen.print(state2);
    state2 = 0;
  }

  //move down
  else {
    arm1.spinFor(-650, deg, 100, velocityUnits::pct, false);
    arm2.spinFor(-650, deg, 100, velocityUnits::pct, false);
    Controller1.Screen.clearScreen();
    Controller1.Screen.print(state2);
    state2 = 1;
  }

  //this_thread::sleep_for(500);
  Controller1.Screen.print(state2);
}

bool state3 = 1;

void R2() {
//move lock/grip device
  if (state3) {
    pist1.open();
    state3 = 0;
  }
  else {
    pist1.close();
    state3 = 1;
  }
}

void X() {
  //drive up ramp
  TRMotor.spinFor(1, rev, 150, velocityUnits::pct, true);
  TLMotor.spinFor(1, rev, 150, velocityUnits::pct, true);
  BLMotor.spinFor(1, rev, 150, velocityUnits::pct, false);
  BRMotor.spinFor(1, rev, 150, velocityUnits::pct, false);
}

void A()  {
//move goal from black clamp to large arm

}

void B() {
//flip see saw

}

void Y() {

//move goal from large arm to black clamp
}





void usercontrol(void) {

  enabledrivePID = false;
  

  while(1) {

    Controller1.ButtonR1.pressed(R1);
    Controller1.ButtonL1.pressed(L1);
    Controller1.ButtonX.pressed(X);

    if (0) {

    }

    else {
    
    TLMotor.spin(vex::directionType::fwd, speed*(Controller1.Axis3.position() + Controller1.Axis1.position()/2)/8, vex::velocityUnits::pct);
    TRMotor.spin(vex::directionType::fwd, speed*(Controller1.Axis3.position() - Controller1.Axis1.position()/2)/8, vex::velocityUnits::pct);
    BLMotor.spin(vex::directionType::fwd, speed*(Controller1.Axis3.position() + Controller1.Axis1.position()/2)/8, vex::velocityUnits::pct);
    BRMotor.spin(vex::directionType::fwd, speed*(Controller1.Axis3.position() - Controller1.Axis1.position()/2)/8, vex::velocityUnits::pct);
    }

    if (convey_state) {
      conveyor.spin(fwd, 100, pct);
    }
    else {
      conveyor.setBrake(brake);
      conveyor.stop();
    }

  }
}

int main() {

  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);

}