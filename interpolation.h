#ifndef INTERPOLATION_H_
#define INTERPOLATION_H_
#include <Arduino.h>

#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7
#define DEFAULT_ARC_TOLERANCE 0.002 
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 12
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3

#define STATE_LINEAR_INTERPOLATION 0
#define STATE_ARC_INTERPOLATION 1
#define STATE_STEP_INTERPOLATION 2
#define STATE_NONE -1

struct Point {
  float xmm;
  float ymm;
  float zmm;
  float emm;
};

struct Segment {
  Point p;
  float v;
};

class Interpolation {
public:
  Interpolation();
  //void resetInterpolation(float px, float py, float pz);
  //void resetInterpolation(float p1x, float p1y, float p1z, float p2x, float p2y, float p2z);
  //void resetInterpolation(Point p0, Point p1);
  
  void Interpolation::setOrigin(Point origin_param);
  Point Interpolation::getOrigin() const;  
  Point getDefaultOrigin() const;
  void setCurrentPos(float px, float py, float pz, float pe);
  void setInterpolation(float px, float py, float pz, float pe, float v = 0);
  void setInterpolation(float p1x, float p1y, float p1z, float p1e, float p2x, float p2y, float p2z, float p2e, float av = 0);
  
  void setCurrentPos(Point p);
  void setInterpolation(Point p1, float v = 0);
  void setInterpolation(Point p0, Point p1, float v = 0);
  void setInterpolation(Point p0, Point p1, float av, boolean is_segment);

  void setToolOffset(float toolOffset_x, float toolOffset_y, float toolOffset_z);
  
  void handleSegments();
  void updateActualPosition();
  void setArcInterpolation(float *target, float *offset_param, float feed_rate, boolean is_clockwise_arc); //, uint8_t extruder);
  bool isFinished() const;

  //void preventOverflow(float target[4]);
  bool isAllowedPosition(float target[4]);
  
  float getXPosmm() const;
  float getYPosmm() const;
  float getZPosmm() const;
  float getEPosmm() const;
  //float getIPosmm() const;
  //float getJPosmm() const;
  //float getKPosmm() const;
  Point getPosmm() const;
  
private:
  float getProgress();
  float getDistance() const;
  void clamp_to_software_endstops(float target[3]);
  void linearUpdate();
  void arcUpdate();
  void stepUpdate();
  byte state;
  
  long startTime;  
  
  Point origin;
  float xStartmm;
  float yStartmm;
  float zStartmm;
  float eStartmm;
  float xDelta;
  float yDelta;
  float zDelta;
  float eDelta;
  float xPosmm;
  float yPosmm;
  float zPosmm;
  float ePosmm;
  float v;
  float tmul;
  float radius;
  uint8_t axis_0;
  uint8_t axis_1;
  uint8_t axis_linear;
  float center_axis0;
  float center_axis1;
  float r_axis0;
  float r_axis1;
  float* offset;
  float* target;
  float arc_start[4];
  float arc_target[4];
  float angular_travel;
  float linear_travel;
  float extruder_travel;
  float feed_rate;
  uint8_t endstate;
  uint16_t segments_nr;
  uint16_t segments_index;
  float toolOffset[3];
};

#endif
