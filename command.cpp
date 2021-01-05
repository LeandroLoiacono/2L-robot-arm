#include "command.h"
#include "logger.h"
#include <Arduino.h>

Command::Command() {
  //initialize Command to a zero-move value;
  new_command.valueX = NAN; 
  new_command.valueY = NAN;
  new_command.valueZ = NAN;
  new_command.valueF = 0;
  new_command.valueE = NAN;
  new_command.valueS = 0;  
  new_command.valueI = NAN; 
  new_command.valueJ = NAN; 
  new_command.valueK = NAN;
  message = "";
  isRelativeCoord = false;
}

bool Command::handleGcode() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
       return false; 
    }
    if (c == '\r') {
       bool b = processMessage(message);
       message = "";
       return b;
    } else {
       message += c; 
    }
  }
  return false;
}

bool Command::processMessage(String msg){

  new_command.valueX = NAN; 
  new_command.valueY = NAN;
  new_command.valueZ = NAN;
  new_command.valueE = NAN;
  new_command.valueF = 0;
  new_command.valueS = 0;  
  new_command.valueI = NAN; 
  new_command.valueJ = NAN; 
  new_command.valueK = NAN;
  msg.toUpperCase();
  msg.replace(" ", "");
  int active_index = 0;
  new_command.id = msg[active_index];
  if((new_command.id != 'G') && (new_command.id != 'M')){
    printErr();
    return false;
  }

  active_index++;
  int temp_index = active_index;
  while (temp_index<msg.length() && !isAlpha(msg[temp_index])){
    temp_index++;
  }
  new_command.num = msg.substring(active_index, temp_index).toInt();
  active_index = temp_index;
  temp_index++;
  while (temp_index<msg.length()){
    while (!isAlpha(msg[temp_index]) || msg[temp_index]=='.'){
      temp_index++;
      if (temp_index == msg.length()){
        break;
      }
    }
    value_segment(msg.substring(active_index, temp_index));
    active_index = temp_index;
    temp_index++;
  }
  return true;
}

void Command::value_segment(String msg_segment){
  float msg_value = msg_segment.substring(1).toFloat();
  switch (msg_segment[0]){
    case 'X': new_command.valueX = msg_value; break;
    case 'Y': new_command.valueY = msg_value; break;
    case 'Z': new_command.valueZ = msg_value; break;
    case 'E': new_command.valueE = msg_value; break;
    case 'F': new_command.valueF = msg_value; break;
    case 'S': new_command.valueS = msg_value; break;
    case 'I': new_command.valueI = msg_value; break;
    case 'J': new_command.valueJ = msg_value; break;
    case 'K': new_command.valueK = msg_value; break;
  }
}


Cmd Command::getCmd() const {
  return new_command; 
}

void Command::cmdGetPosition(Point pos, float highRad, float lowRad, float rotRad){
  if(isRelativeCoord) {
    Serial.println("// Relative Coordinate Mode //");
  } else {
    Serial.println("// Absolute Coordinate Mode //");
  }
  Serial.print("// Now at: X");
  Serial.print(pos.xmm);
  Serial.print(" Y");
  Serial.print(pos.ymm);
  Serial.print(" Z");
  Serial.print(pos.zmm);
  Serial.print(" E");
  Serial.println(pos.emm);
  Serial.print("// Radians - HIGH ");
  Serial.print(highRad);
  Serial.print(" LOW ");
  Serial.print(lowRad);
  Serial.print(" ROT ");
  Serial.println(rotRad);
}

void Command::cmdToRelative(){
  isRelativeCoord = true;
  Serial.println("// Relative Coordinate Mode //");
}

void Command::cmdToAbsolute(){
  isRelativeCoord = false;
  Serial.println("// Absolute Coordinate Mode //");
}

void cmdMove(Cmd(&cmd), Point pos, Point origin, bool isRelativeCoord){
  if(isRelativeCoord == true){
    cmd.valueX = isnan(cmd.valueX) ? cmd.valueX + pos.xmm : 0.0;
    cmd.valueY = isnan(cmd.valueY) ? cmd.valueY + pos.ymm : 0.0;
    cmd.valueZ = isnan(cmd.valueZ) ? cmd.valueZ + pos.zmm : 0.0;
    cmd.valueE = isnan(cmd.valueE) ? cmd.valueE + pos.emm : 0.0; 
  } else {
    cmd.valueX = isnan(cmd.valueX) ? pos.xmm : cmd.valueX + origin.xmm;
    cmd.valueY = isnan(cmd.valueY) ? pos.ymm : cmd.valueY + origin.ymm;
    cmd.valueZ = isnan(cmd.valueZ) ? pos.zmm : cmd.valueZ + origin.zmm;
    cmd.valueE = isnan(cmd.valueE) ? pos.emm : cmd.valueE + origin.emm; 
  }  
}

void cmdDwell(Cmd(&cmd)){
  delay(int(cmd.valueS * 1000));
}

void printErr() {
  Logger::logERROR("Incorrect Command");
}
