#include "config.h"
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
  new_command.valueR = false;
  message = "";
  serial3_message = "";
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

bool Command::handleSerial3Gcode() {
  if (Serial3.available()) {
    char c = Serial3.read();
    if (c == '\n') {
       return false; 
    }
    if (c == '\r') {
       if(serial3_message.length() > 3 && serial3_message.startsWith(">>>")) {
         String serail3_command = serial3_message.substring(3);
         bool b = processMessage(serail3_command);
         serial3_message = "";
         return b;
       } else {
         Serial.println(serial3_message);
         serial3_message = "";
         return false;
       }
    } else {
       serial3_message += c; 
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
  new_command.valueR = false;
  msg.toUpperCase();
  msg.replace(" ", "");
  if(msg[msg.length() - 1] == 'R') {
    msg += '1'; 
  }
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
  Logger::logINFO(msg_segment);
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
    case 'R': new_command.valueR = msg_value > 0; break;
  }
}


Cmd Command::getCmd() const {
  return new_command; 
}

void Command::cmdGetPosition(Point pos, float highRad, float lowRad, float rotRad){
  if(isRelativeCoord) {
    Serial.println("// Relative Coordinate Mode //");
    #if USE_SERIAL3
      Serial3.print("// Relative Coordinate Mode //\\r\\n");
      
    #endif
  } else {
    Serial.println("// Absolute Coordinate Mode //");
    #if USE_SERIAL3
      Serial3.print("// Absolute Coordinate Mode //\\r\\n");
    #endif
  }
  String output = "// Now at: X " + String(pos.xmm) + 
    " Y" + String(pos.ymm) + " Z" + String(pos.zmm) + 
    " E" + String(pos.emm);
  Serial.println(output);
  #if USE_SERIAL3
    output += "\\r\\n";
    Serial3.print(output);
  #endif
  output = "// Radians - HIGH " + String(highRad) + 
    " LOW " + String(lowRad) + " ROT " + String(rotRad);
  Serial.println(output);
  #if USE_SERIAL3
    output += "\\r\\n";
    Serial3.print(output);
    Serial3.flush();
  #endif
}

void Command::cmdToRelative(){
  isRelativeCoord = true;
  Serial.println("// Relative Coordinate Mode //");
  #if USE_SERIAL3
    Serial3.print("// Relative Coordinate Mode //\\r\\n");
    Serial3.flush();
  #endif
}

void Command::cmdToAbsolute(){
  isRelativeCoord = false;
  Serial.println("// Absolute Coordinate Mode //");
  #if USE_SERIAL3
    Serial3.print("// Absolute Coordinate Mode //\\r\\n");
    Serial3.flush();
  #endif
}

void cmdMove(Cmd(&cmd), Point pos, Point origin, bool isRelativeCoord){
  if(isRelativeCoord == true){
    cmd.valueX = isnan(cmd.valueX) ? pos.xmm : cmd.valueX + pos.xmm;
    cmd.valueY = isnan(cmd.valueY) ? pos.ymm : cmd.valueY + pos.ymm;
    cmd.valueZ = isnan(cmd.valueZ) ? pos.zmm : cmd.valueZ + pos.zmm;
    cmd.valueE = isnan(cmd.valueE) ? pos.emm : cmd.valueE + pos.emm; 
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
