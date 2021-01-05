//20sffactory community_robot v0.21

#include "config.h"
#include "pinout.h"
#include "logger.h"

#include <Arduino.h>

#include "robotGeometry.h"
#include "interpolation.h"
#include "fanControl.h"
#include "RampsStepper.h"
#include "queue.h"
#include "command.h"
#include "byj_gripper.h"
#include "equipment.h"
#include "endstop.h"

//STEPPER OBJECTS
RampsStepper stepperHigher(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, INVERSE_X_STEPPER);
RampsStepper stepperLower(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, INVERSE_Y_STEPPER);
RampsStepper stepperRotate(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, INVERSE_Z_STEPPER);

//RAIL OBJECTS
RampsStepper stepperRail(E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, INVERSE_E0_STEPPER);
Endstop endstopE0(E0_MIN_PIN, E0_DIR_PIN, E0_STEP_PIN, E0_ENABLE_PIN, E0_MIN_INPUT, E0_HOME_STEPS, HOME_DWELL);

//ENDSTOP OBJECTS
Endstop endstopX(X_MIN_PIN, X_DIR_PIN, X_STEP_PIN, X_ENABLE_PIN, X_MIN_INPUT, X_HOME_STEPS, HOME_DWELL);
Endstop endstopY(Y_MIN_PIN, Y_DIR_PIN, Y_STEP_PIN, Y_ENABLE_PIN, Y_MIN_INPUT, Y_HOME_STEPS, HOME_DWELL);
Endstop endstopZ(Z_MIN_PIN, Z_DIR_PIN, Z_STEP_PIN, Z_ENABLE_PIN, Z_MIN_INPUT, Z_HOME_STEPS, HOME_DWELL);

//EQUIPMENT OBJECTS
BYJ_Gripper byj_gripper(BYJ_PIN_0, BYJ_PIN_1, BYJ_PIN_2, BYJ_PIN_3, GRIP_STEPS);
Equipment laser(LASER_PIN);
Equipment pump(PUMP_PIN);
Equipment led(LED_PIN);
FanControl fan(FAN_PIN, FAN_DELAY);

//EXECUTION & COMMAND OBJECTS
RobotGeometry geometry(END_EFFECTOR_OFFSET, SHANK_LENGTH);
Interpolation interpolator;
Queue<Cmd> queue(QUEUE_SIZE);
Command command;

void setup()
{
  Serial.begin(BAUD);
  
  stepperHigher.setPositionRad(PI / 2.0); // 90°
  stepperLower.setPositionRad(0);         // 0°
  stepperRotate.setPositionRad(0);        // 0°
  stepperRail.setPosition(0);
  if (HOME_ON_BOOT) { //HOME DURING SETUP() IF HOME_ON_BOOT ENABLED
    homeSequence(); 
    Logger::logINFO("Robot Online.");
  } else {
    setStepperEnable(false); //ROBOT ADJUSTABLE BY HAND AFTER TURNING ON
    if (HOME_X_STEPPER && HOME_Y_STEPPER && !HOME_Z_STEPPER){
      Logger::logINFO("Robot Online.");
      Logger::logINFO("Rotate robot to face front centre & send G28.");
    }
    if (HOME_X_STEPPER && HOME_Y_STEPPER && HOME_Z_STEPPER){
      Logger::logINFO("Robot Online.");
      Logger::logINFO("Send G28 to calibrate.");
    }
    if (!HOME_X_STEPPER && !HOME_Y_STEPPER){
      Logger::logINFO("Robot Online.");
      Logger::logINFO("Home robot manually and send G28 to calibrate.");
    }
  }
  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
}

void loop() {
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  stepperRotate.stepToPositionRad(geometry.getRotRad());
  stepperLower.stepToPositionRad(geometry.getLowRad());
  stepperHigher.stepToPositionRad(geometry.getHighRad());
  if (RAIL){
    stepperRail.stepToPositionMM(interpolator.getEPosmm(), STEPS_PER_MM_RAIL);
  }
  stepperRotate.update();
  stepperLower.update();
  stepperHigher.update();
  if (RAIL){
    stepperRail.update();
  }
  fan.update();
  if (!queue.isFull()) {
    if (command.handleGcode()) {
      queue.push(command.getCmd());
    }
  }
  if ((!queue.isEmpty()) && interpolator.isFinished()) {
    executeCommand(queue.pop());
    if (PRINT_REPLY) {Serial.println(PRINT_REPLY_MSG);}
  }

  if (millis() % 500 < 250) {
    led.cmdOn();
  }
  else {
    led.cmdOff();
  }
}

void executeCommand(Cmd cmd) {

  if (cmd.id == -1) {
    printErr();
    return;
  }

  Point origin = interpolator.getOrigin();
  
  if (cmd.id == 'G') {
    float target[4];
    float offset[3];
    String logMsg = "// ";
    
    switch (cmd.num) {
      case 0:
      case 1:
        fan.enable(true);
        cmdMove(cmd,interpolator.getPosmm(), interpolator.getOrigin(), command.isRelativeCoord);
        Logger::logINFO("// Move to: X" + String(cmd.valueX) + " Y" + String(cmd.valueY) + " Z" + String(cmd.valueZ) + " E" + String(cmd.valueE));
        interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE, cmd.valueF);
        break;
      //case 1: cmdMove(cmd); break;
      case 2:
      case 3: 
        fan.enable(true);
        cmdMove(cmd,interpolator.getPosmm(), interpolator.getOrigin(), command.isRelativeCoord);
        
        logMsg += cmd.num ==2 ? "CW " : "CCW";
        logMsg += "ARC to: X" + String(cmd.valueX) + " Y" + String(cmd.valueY) + " Z" + String(cmd.valueZ) + " E" + String(cmd.valueE);
        logMsg += " I" + String(cmd.valueI) + " J" + String(cmd.valueJ) + " K" + String(cmd.valueK);
        Logger::logINFO(logMsg);
        
        target[X_AXIS] = cmd.valueX;
        target[Y_AXIS] = cmd.valueY; 
        target[Z_AXIS] = cmd.valueZ; 
        target[E_AXIS] = cmd.valueE; 
        offset[X_AXIS] = cmd.valueI;
        offset[Y_AXIS] = cmd.valueJ;
        offset[Z_AXIS] = cmd.valueK;
        if((!isnan(offset[X_AXIS]) && !isnan(offset[Y_AXIS])) || (!isnan(offset[X_AXIS]) && !isnan(offset[Z_AXIS])) || (!isnan(offset[Y_AXIS]) && !isnan(offset[Z_AXIS]))) {
          interpolator.setArcInterpolation(target, offset, cmd.valueF, cmd.num == 2);
        } else {
          printErr();
        }
      break;
      case 4: cmdDwell(cmd); break;
      case 28:
        homeSequence(); 
        break;
      case 90: command.cmdToAbsolute(); break; // ABSOLUTE COORDINATE MODE
      case 91: command.cmdToRelative(); break; // RELATIVE COORDINATE MODE
      case 92:  
        cmdMove(cmd,interpolator.getPosmm(), interpolator.getDefaultOrigin(), false);
        Point newOrigin;
        newOrigin.xmm = interpolator.getXPosmm() - cmd.valueX;
        newOrigin.ymm = interpolator.getYPosmm() - cmd.valueY;
        newOrigin.zmm = interpolator.getZPosmm() - cmd.valueZ;
        newOrigin.emm = interpolator.getEPosmm() - cmd.valueE;
        interpolator.setOrigin(newOrigin);
        break; 
      default: printErr();
    }
  }
  else if (cmd.id == 'M') {
    switch (cmd.num) {
      case 1: pump.cmdOn(); break;
      case 2: pump.cmdOff(); break;
      case 3: byj_gripper.cmdOn(); break;
      case 5: byj_gripper.cmdOff(); break;
      case 6: laser.cmdOn(); break;
      case 7: laser.cmdOff(); break;
      case 17: setStepperEnable(true); break;
      case 18: setStepperEnable(false); break;
      case 106: fan.enable(true); break;
      case 107: fan.enable(false); break;
      case 114: command.cmdGetPosition(interpolator.getPosmm(), stepperHigher.getPosition(), stepperLower.getPosition(), stepperRotate.getPosition()); break;// Return the current positions of all axis 
      default: printErr(); 
    }
  }
  else {
    printErr();
  }
}

void setStepperEnable(bool enable){
  stepperRotate.enable(enable);
  stepperLower.enable(enable);
  stepperHigher.enable(enable);
  if(RAIL){
    stepperRail.enable(enable);
  }
  fan.enable(enable);
}

void homeSequence(){
  setStepperEnable(false);
  fan.enable(true);
  if (HOME_Y_STEPPER){
    endstopY.home(!INVERSE_Y_STEPPER); //INDICATE STEPPER HOMING DIRECTION
  }
  if (HOME_X_STEPPER){
    endstopX.home(!INVERSE_X_STEPPER); //INDICATE STEPPER HOMING DIRECTION
  }
  if (HOME_Z_STEPPER){
    endstopZ.home(INVERSE_Z_STEPPER); //INDICATE STEPPER HOMING DIRECDTION
  }
  if (RAIL){
    if (HOME_E0_STEPPER){
      endstopE0.home(!INVERSE_E0_STEPPER); //
    }
  }
  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
  interpolator.setOrigin(interpolator.getDefaultOrigin());
}
