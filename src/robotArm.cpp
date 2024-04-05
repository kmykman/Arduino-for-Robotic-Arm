#include <Arduino.h>
#include <Servo.h>
#include <Stepper.h>

#include "Buzzer.hpp"
#include "RampsStepper.h"
#include "command.h"
#include "config.h"
#include "endstop.h"
#include "fanControl.h"
#include "interpolation.h"
#include "pinout.h"
#include "queue.h"
#include "robotGeometry.h"

Buzzer buzzer(BUZZER_PIN);

Stepper stepper(2400, STEPPER_GRIPPER_PIN_0, STEPPER_GRIPPER_PIN_1,
                STEPPER_GRIPPER_PIN_2, STEPPER_GRIPPER_PIN_3);
RampsStepper stepperRotate(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
RampsStepper stepperLower(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
RampsStepper stepperHigher(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
RampsStepper stepperExtruder(E_STEP_PIN, E_DIR_PIN, E_ENABLE_PIN);

// ENDSTOP OBJECTS
Endstop endstopX(X_MIN_PIN, X_DIR_PIN, X_STEP_PIN, X_ENABLE_PIN, X_MIN_INPUT,
                 X_HOME_STEPS, HOME_DWELL, false);
Endstop endstopY(Y_MIN_PIN, Y_DIR_PIN, Y_STEP_PIN, Y_ENABLE_PIN, Y_MIN_INPUT,
                 Y_HOME_STEPS, HOME_DWELL, false);
Endstop endstopZ(Z_MIN_PIN, Z_DIR_PIN, Z_STEP_PIN, Z_ENABLE_PIN, Z_MIN_INPUT,
                 Z_HOME_STEPS, HOME_DWELL, false);

FanControl fan(FAN_PIN);
RobotGeometry geometry;
Interpolation interpolator;
Queue<Cmd> queue(15);
Command command;

Servo servo;
Servo gripperValve;
Servo rotator;
Servo holder;
int angle = 135;
int angle_offset = 90;
int angle_holder_offset = 110;

bool asked = false;

#ifdef __ROS__
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include "rosserial.h"
//
std_msgs::String status;
ros::Publisher statusPub("status", &status);
//
void gcodeCallback(const std_msgs::String&);
ros::Subscriber<std_msgs::String> gcodeSub("gcode", gcodeCallback);
//
void hardwareReset(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response>
    resetSrv("reset", &hardwareReset);
void homePosition(const std_srvs::Empty::Request& req,
                  std_srvs::Empty::Response& res);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response>
    homePosSrv("home", &homePosition);
void restPosition(const std_srvs::Empty::Request& req,
                  std_srvs::Empty::Response& res);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response>
    restPosSrv("rest", &restPosition);
void bottomPosition(const std_srvs::Empty::Request& req,
                    std_srvs::Empty::Response& res);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response>
    bottomPosSrv("bottom", &bottomPosition);
//
ros::MyNodeHandle nh;

void processCommand(String cmd) {
  command.processMessage(cmd + "\r\n");
  queue.push(command.getCmd());
  status.data = "ok";
}

void gcodeCallback(const std_msgs::String& gcode) {
  if (!queue.isFull()) {
    String message = gcode.data;
    processCommand(message);
  }
  nh.loginfo("gcodeCallback() called");
  // nh.logdebug("Debug Statement");
  // nh.logwarn("Warnings.");
  // nh.logerror("Errors..");
  // nh.logfatal("Fatalities!");
}

void homePosition(const std_srvs::Empty::Request& req,
                  std_srvs::Empty::Response& res) {
  String message = "G1 X0 Y180 Z180";
  processCommand(message);
  nh.loginfo("homePosition() called");
  return;
}

void restPosition(const std_srvs::Empty::Request& req,
                  std_srvs::Empty::Response& res) {
  String message = "G1 X0 Y40 Z70";
  processCommand(message);
  nh.loginfo("restPosition() called");
  return;
}

void bottomPosition(const std_srvs::Empty::Request& req,
                    std_srvs::Empty::Response& res) {
  String message = "G1 X0 Y100 Z0";
  processCommand(message);
  nh.loginfo("bottomPosition() called");
  return;
}

void hardwareReset(const std_srvs::Empty::Request& req,
                   std_srvs::Empty::Response& res) {
  nh.loginfo("hardwareReset() called");
  nh.getHardware()->flush();
  delay(1000);
  void (*resetFunc)(void) = 0;
  resetFunc();
  return;
}

void setupROSserial() {
  Console.println(F("ROSConsole init, wait..."));
  nh.initNode();
  // while (!nh.connected()) {
  //   nh.spinOnce();
  // }
  nh.loginfo("<init>");
  nh.subscribe(gcodeSub);
  nh.advertise(statusPub);
  nh.advertiseService(resetSrv);
  nh.advertiseService(homePosSrv);
  nh.advertiseService(bottomPosSrv);
  nh.advertiseService(restPosSrv);
  nh.negotiateTopics();
  nh.loginfo("<started>");
  Console.println(F("ROSConsole init, done!"));
}

#endif  //__ROS__

void setStepperEnable(bool enable) {
  stepperRotate.enable(enable);
  stepperLower.enable(enable);
  stepperHigher.enable(enable);
  stepperExtruder.enable(enable);
  fan.enable(enable);
}

void homeSequence() {
  setStepperEnable(false);
  fan.enable(true);
  if (HOME_X_STEPPER) {
    endstopX.home(!INVERSE_X_STEPPER);
    endstopY.tillEndstop(!INVERSE_Y_STEPPER);
  } else {
    setStepperEnable(true);
    endstopX.homeOffset(!INVERSE_X_STEPPER);
  }
  if (HOME_Z_STEPPER) {
    endstopZ.home(INVERSE_Z_STEPPER);
  }
  if (HOME_Y_STEPPER) {
    endstopY.home(!INVERSE_Y_STEPPER);
  } else {
    setStepperEnable(true);
    endstopY.homeOffset(!INVERSE_Y_STEPPER);
  }
  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0,
                                INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
}

void xHome() {
  // setStepperEnable(false);
  fan.enable(true);
  if (HOME_Y_STEPPER && HOME_X_STEPPER) {
    endstopY.home(!INVERSE_Y_STEPPER);
  } else {
    setStepperEnable(true);
    endstopY.homeOffset(!INVERSE_Y_STEPPER);
  }
  if (HOME_Z_STEPPER) {
    endstopZ.tillEndstop(INVERSE_Z_STEPPER);
  }
  interpolator.setInterpolation(180, 100, INITIAL_Z, INITIAL_E0,
                                180, 100, INITIAL_Z, INITIAL_E0);
}

void cmdMove(Cmd(&cmd)) {
  interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE,
                                cmd.valueF);
}

void cmdDwell(Cmd(&cmd)) { delay(int(cmd.valueT * 1000)); }

void cmdGripperClose(Cmd(&cmd)) {
  Console.println("Gripper close ");

  // vaccum gripper
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  gripperValve.write(0);  // deactivate the valve, no air through

  servo.write(58);

  // stepper gripper
  stepper.setSpeed(5);
  stepper.step(int(cmd.valueT));
  delay(50);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);
  // printComment("// NOT IMPLEMENTED");
  // printFault();
}

void cmdGripperOpen(Cmd(&cmd)) {
  // vaccum griiper
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  gripperValve.write(180);  // activate the valve, allow air through.

  // servo gripper
  int open_deg=int(cmd.valueT);
  if (open_deg > 90 || open_deg == 0){
    open_deg = 30;
  }
  servo.write(58+open_deg);
}

void cmdStepperOn() { setStepperEnable(true); }

void cmdStepperOff() { setStepperEnable(false); }

void cmdFanOn() { fan.enable(true); }

void cmdFanOff() { fan.enable(false); }

// for rotating gripper
void cmdRotateCW(Cmd(&cmd)) {
  int angle = cmd.valueT;
  if (!isnan(cmd.valueX) && !isnan(cmd.valueY) && !isnan(cmd.valueZ) && cmd.valueT == 0){
    angle = (atan(- cmd.valueX / cmd.valueY) * 180) / M_PI + 90;
    if (cmd.valueX < 0){
      angle -= 10;
    }
    else if (cmd.valueX > 0){
      angle += 4;
    }
  }
  else if (cmd.valueT < -90 || cmd.valueT > 90){
    angle = 90;
  }
  else{
    angle += 90;
  }
  rotator.write(angle);
}

void cmdHolderRotateCW() {
  holder.write(170-angle_holder_offset);
  // Console.println("Rotate clockwise");
  delay(1000);
  holder.write(170);
  // Console.println("Rotate counterclockwise");
}


void handleAsErr(Cmd(&cmd)) {
  printComment("Unknown Cmd " + String(cmd.id) + String(cmd.num) + " (queued)");
  printFault();
}

void executeCommand(Cmd cmd) {
  if (cmd.id == -1) {
    String msg = "parsing Error";
    printComment(msg);
    handleAsErr(cmd);
    return;
  }

  if (cmd.valueX == NAN) {
    cmd.valueX = interpolator.getXPosmm();
  }
  if (cmd.valueY == NAN) {
    cmd.valueY = interpolator.getYPosmm();
  }
  if (cmd.valueZ == NAN) {
    cmd.valueZ = interpolator.getZPosmm();
  }
  if (cmd.valueE == NAN) {
    cmd.valueE = interpolator.getEPosmm();
  }

  // decide what to do
  if (cmd.id == 'G') {
    switch (cmd.num) {
      case 0:
        cmdMove(cmd);
        break;
      case 1:
        cmdMove(cmd);
        break;
      case 4:
        cmdDwell(cmd);
        break;
      case 28:
        homeSequence();
        cmdRotateCW(cmd);
        break;
      // case 21: break; //set to mm
      // case 90: cmdToAbsolute(); break;
      // case 91: cmdToRelative(); break;
      // case 92: cmdSetPosition(cmd); break;
      default:
        handleAsErr(cmd);
    }
  } else if (cmd.id == 'M') {
    switch (cmd.num) {
      case 1:
        cmdRotateCW(cmd);
        cmdMove(cmd);
        break;
      case 3:
        cmdGripperClose(cmd);
        break;
      case 5:
        cmdGripperOpen(cmd);
        break;
      case 17:
        cmdStepperOn();
        break;
      case 18:
        cmdStepperOff();
        break;
      case 106:
        cmdFanOn();
        break;
      case 107:
        cmdFanOff();
        break;
      case 200:
        cmdRotateCW(cmd);
        break;
      case 202:
        cmdHolderRotateCW();
        break;
        break;
      default:
        handleAsErr(cmd);
    }
  } else {
    handleAsErr(cmd);
  }
}

void loop() {
  // update and Calculate all Positions, Geometry and Drive all Motors...
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(),
               interpolator.getZPosmm());
  stepperRotate.stepToPositionRad(geometry.getRotRad());
  stepperLower.stepToPositionRad(geometry.getLowRad());
  stepperHigher.stepToPositionRad(geometry.getHighRad());
  stepperExtruder.stepToPositionRad(interpolator.getEPosmm());
  stepperRotate.update();
  stepperLower.update();
  stepperHigher.update();
  fan.update();

#ifdef __ROS__
  if (status.data != NULL) {
    statusPub.publish(&status);
    status.data = NULL;
  }
  nh.spinOnce();
#endif

  if (!queue.isFull()) {
    if (command.handleGcode()) {
      queue.push(command.getCmd());
      printOk();
    }
  }

  if ((!queue.isEmpty()) && interpolator.isFinished()) {
    executeCommand(queue.pop());
    Serial1.print("A" + String(command.getACKno()));
    Console.println("--> A" + String(command.getACKno()));
  }

  if (millis() % 500 < 250) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

void setup() {
  Console.begin(115200);
  BlueTooth.begin(38400);

  // various pins..
  pinMode(HEATER_0_PIN, OUTPUT);
  pinMode(HEATER_1_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // unused Stepper..
  pinMode(E_STEP_PIN, OUTPUT);
  pinMode(E_DIR_PIN, OUTPUT);
  pinMode(E_ENABLE_PIN, OUTPUT);

  // unused Stepper..
  pinMode(Q_STEP_PIN, OUTPUT);
  pinMode(Q_DIR_PIN, OUTPUT);
  pinMode(Q_ENABLE_PIN, OUTPUT);

  // stepper gripper
  pinMode(STEPPER_GRIPPER_PIN_0, OUTPUT);
  pinMode(STEPPER_GRIPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_GRIPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_GRIPPER_PIN_3, OUTPUT);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);

  // vaccum gripper
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  gripperValve.attach(VALVE_PIN);
  gripperValve.write(0);

  // servo gripper
  servo.attach(SERVO_PIN);
  servo.write(70);

  // servo rotator
  rotator.attach(SERVO_ROTATE_PIN);
  rotator.write(90);

  // servo holder
  holder.attach(SERVO_HOLDER_PIN);
  holder.write(170);

  // reduction of steppers..
  stepperHigher.setReductionRatio(
      32.0 / 9.0, 200 * 16);  // big gear: 32, small gear: 9, steps per rev:
                              // 200, microsteps: 16
  stepperLower.setReductionRatio(32.0 / 9.0, 200 * 16);
  stepperRotate.setReductionRatio(32.0 / 9.0, 200 * 16);
  stepperExtruder.setReductionRatio(32.0 / 9.0, 200 * 16);

  // start positions..
  stepperHigher.setPositionRad(PI / 2.0);  // 90°
  stepperLower.setPositionRad(0);          // 0°
  stepperRotate.setPositionRad(0);         // 0°
  stepperExtruder.setPositionRad(0);

  // enable and init..

  if (HOME_ON_BOOT) {  // HOME DURING SETUP() IF HOME_ON_BOOT ENABLED
    homeSequence();
  } else {
    setStepperEnable(false);
  }

  // interpolator.setInterpolation(0, 180, 180, 0, 0, 180, 180, 0);
  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0,
                                INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
#ifdef __ROS__
  setupROSserial();
#endif
  buzzer.beepShort();
  Console.println("started");
  // Serial1.println("started");
}