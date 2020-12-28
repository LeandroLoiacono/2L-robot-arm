#include "interpolation.h"
#include "config.h"
#include "queue.h"
#include "logger.h"


#define E_AXIS 3

void Interpolation::setCurrentPos(float px, float py, float pz, float pe) {
  Point p;
  p.xmm = px;
  p.ymm = py;
  p.zmm = pz;
  p.emm = pe;
  setCurrentPos(p);
}

void Interpolation::setInterpolation(float px, float py, float pz, float pe, float v) {
  Point p;
  p.xmm = px;
  p.ymm = py;
  p.zmm = pz;
  p.emm = pe;
  setInterpolation(p, v);
}

void Interpolation::setInterpolation(float p1x, float p1y, float p1z, float p1e, float p2x, float p2y, float p2z, float p2e, float v) {
  Point p1;
  Point p2;
  p1.xmm = p1x;
  p1.ymm = p1y;
  p1.zmm = p1z;
  p1.emm = p1e;
  p2.xmm = p2x;
  p2.ymm = p2y;
  p2.zmm = p2z;
  p2.emm = p2e;
  setInterpolation(p1, p2, v);
}

void Interpolation::setInterpolation(Point p1, float v) {
  Point p0;
  p0.xmm = xStartmm + xDelta;
  p0.ymm = yStartmm + yDelta;
  p0.zmm = zStartmm + zDelta;
  p0.emm = eStartmm + eDelta;
  setInterpolation(p0, p1, v);
}

void Interpolation::setInterpolation(Point p0, Point p1, float av) {
  setInterpolation(p0, p1, av, false);
}

void Interpolation::setInterpolation(Point p0, Point p1, float av, boolean is_segment) {
  v = av; //mm/s
  
  float a = (p1.xmm - p0.xmm);
  float b = (p1.ymm - p0.ymm);
  float c = (p1.zmm - p0.zmm);
  float e = abs(p1.emm - p0.emm);
  float dist = sqrt(a*a + b*b + c*c);

  if (dist < e) {
    dist = e; 
  }
//  if (dist = 0) {
//    dist = e;
//  } 
  if (v < 5) { //includes 0 = default value
    v = sqrt(dist) * 10; //set a good value for v
  }
  if (v < 5) {
     v = 5; 
  }

  if(is_segment) {
    tmul = v / dist; //100;
  } else {
    tmul = v / dist;
  }
  //Serial.print("tmul: ");
  //Serial.println(tmul);
  
  xStartmm = p0.xmm;
  yStartmm = p0.ymm;
  zStartmm = p0.zmm;
  eStartmm = p0.emm;
  
  xDelta = (p1.xmm - p0.xmm);
  yDelta = (p1.ymm - p0.ymm);
  zDelta = (p1.zmm - p0.zmm);
  eDelta = (p1.emm - p0.emm);
   
  state = STATE_LINEAR_INTERPOLATION;
  if(is_segment) {
    endstate = STATE_ARC_INTERPOLATION;
  } else {
    endstate = STATE_NONE;
  }
  
  startTime = micros();
}

void Interpolation::setCurrentPos(Point p) {
  xStartmm = p.xmm;
  yStartmm = p.ymm;
  zStartmm = p.zmm;
  eStartmm = p.emm;
  xDelta = 0;
  yDelta = 0;
  zDelta = 0;
  eDelta = 0;
  segments_nr = 0;
  segments_index = 0;
}


void Interpolation::updateActualPosition() {
  switch (state) {
    case 0:
      linearUpdate();
      break;
    case 1:
      arcUpdate();
    default:
      return;
  }    
}

float Interpolation::getProgress() {
  long microsek = micros();
  float t = (microsek - startTime) / 1000000.0;
  float progress;
  
  switch (SPEED_PROFILE){
    // FLAT SPEED CURVE
    case 0:
      progress = t * tmul;
      if (progress >= 1.0){
        progress = 1.0;
        state = endstate;
      }
      break;
    // ARCTAN APPROX
    case 1:
      progress = atan((PI * t * tmul) - (PI * 0.5)) * 0.5 + 0.5;
      if (progress >= 1.0) {
        progress = 1.0; 
        state = endstate;
      }
      break;
    // COSIN APPROX
    case 2:
      progress = -cos(t * tmul * PI) * 0.5 + 0.5;
      if ((t * tmul) >= 1.0) {
        progress = 1.0; 
        state = endstate;
      }
      break;
  }
  return progress;
}

void Interpolation::linearUpdate() {
  float progress = getProgress();

  float target[4] = {xStartmm + progress * xDelta, yStartmm + progress * yDelta, zStartmm + progress * zDelta, eStartmm + progress * eDelta};
  preventOverflow(target);
  
  xPosmm = target[X_AXIS];
  yPosmm = target[Y_AXIS];
  zPosmm = target[Z_AXIS];
  ePosmm = target[E_AXIS];
  
}

void Interpolation::arcUpdate() {
  
  //String log = "Center: [" + String(center_axis0) + "," + String(center_axis1) + "]";
  //Serial.println(log);
  
  float theta_per_segment = angular_travel/segments_nr;
  float linear_per_segment = linear_travel/segments_nr;
  //float extruder_per_segment = extruder_travel/segments_nr;
  
  // Vector rotation matrix values
  float cos_T = 1-0.5*theta_per_segment*theta_per_segment; // Small angle approximation
  float sin_T = theta_per_segment;
  
  int8_t count = 0;
  float sin_Ti;
  float cos_Ti;
  float r_axisi;
  Point segment_start;
  segment_start = getPosmm();
  Point segment_target;
    
  //for (i=1; i < segments; i++) { // Increment (segments-1)
  if(segments_index < segments_nr) {
    if (count < N_ARC_CORRECTION) {
      // Apply vector rotation matrix 
      r_axisi = r_axis0*sin_T + r_axis1*cos_T;
      r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
      r_axis1 = r_axisi;
      count++;
    } else {
      // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
      // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
      cos_Ti = cos(segments_index*theta_per_segment);
      sin_Ti = sin(segments_index*theta_per_segment);
      r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
      r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
      count = 0;
    }

    float segment_target_arr[4];
    
    // Update arc_target location
    segment_target_arr[axis_0] = (center_axis0 + r_axis0);
    segment_target_arr[axis_1] = (center_axis1 + r_axis1);
    segment_target_arr[axis_linear] = arc_start[axis_linear] + segments_index * linear_per_segment;
    segment_target_arr[E_AXIS] = eStartmm;

    
    segment_target.xmm = segment_target_arr[X_AXIS];
    segment_target.ymm = segment_target_arr[Y_AXIS];
    segment_target.zmm = segment_target_arr[Z_AXIS];
    segment_target.emm = segment_target_arr[E_AXIS];
    //String log = "Arc: segment_index: " + String(segments_index) + ",[" + String(segment_start.xmm) + "," + String(segment_start.ymm) + ","+ String(segment_start.zmm) + "] -> [" + String(segment_target.xmm) + "," + String(segment_target.ymm) + ","+ String(segment_target.zmm) + "]";
    //Serial.println(log);
    
    //clamp_to_software_endstops(arc_target);
    setInterpolation(segment_start, segment_target, feed_rate, true);
    /*xPosmm = segment_target.xmm;
    yPosmm = segment_target.ymm;
    zPosmm = segment_target.zmm;
    ePosmm = segment_target.emm;*/
  }
  //}
  
  if(segments_index == segments_nr) {
    segment_target.xmm = arc_target[X_AXIS];
    segment_target.ymm = arc_target[Y_AXIS];
    segment_target.zmm = arc_target[Z_AXIS];
    segment_target.emm = arc_target[E_AXIS];
    // Ensure last segment arrives at target location.
    String log = "Arc: LAST: " + String(segments_index) + ",[" + String(segment_start.xmm) + "," + String(segment_start.ymm) + ","+ String(segment_start.zmm) + "] -> [" + String(segment_target.xmm) + "," + String(segment_target.ymm) + ","+ String(segment_target.zmm) + "]";
    Logger::logDEBUG(log);
    setInterpolation(segment_start, segment_target, feed_rate, false);
    segments_index++;
    Logger::logDEBUG("Arc interpolation Done");
  } else if(segments_index < segments_nr) {
    segments_index++;
  } else {
    STATE_NONE;
  }
}

bool Interpolation::isFinished() const {
  return state != STATE_ARC_INTERPOLATION && state != STATE_LINEAR_INTERPOLATION; 
}

float Interpolation::getXPosmm() const {
  return xPosmm;
}

float Interpolation::getYPosmm() const {
  return yPosmm;
}

float Interpolation::getZPosmm() const {
  return zPosmm;
}

float Interpolation::getEPosmm() const {
  return ePosmm;
}

Point Interpolation::getPosmm() const {
  Point p;
  p.xmm = xPosmm;
  p.ymm = yPosmm;
  p.zmm = zPosmm;
  p.emm = ePosmm;
  return p;
}

float Interpolation::getDistance() const {
  
}

void Interpolation::setArcInterpolation(float *target_param, float *offset_param, float feed_rate_param, boolean is_clockwise_arc) {
  arc_start[X_AXIS] = getXPosmm();
  arc_start[Y_AXIS] = getYPosmm();
  arc_start[Z_AXIS] = getZPosmm();
  arc_start[E_AXIS] = getEPosmm();
  target = target_param;
  offset = offset_param;
  feed_rate = feed_rate_param;
  arc_target[X_AXIS] = target_param[X_AXIS];
  arc_target[Y_AXIS] = target_param[Y_AXIS];
  arc_target[Z_AXIS] = target_param[Z_AXIS];
  arc_target[E_AXIS] = target_param[E_AXIS];

  if(!isnan(offset[X_AXIS]) && !isnan(offset[Y_AXIS])) {
    if(isnan(offset[Z_AXIS])) { offset[Z_AXIS] = 0; };
    radius = hypot(offset[X_AXIS],offset[Y_AXIS]);
    axis_0 = X_AXIS;
    axis_1 = Y_AXIS;
    axis_linear = Z_AXIS;
  } else if(!isnan(offset[X_AXIS]) && !isnan(offset[Z_AXIS])) {
    if(isnan(offset[Y_AXIS])) { offset[Y_AXIS] = 0; };
    radius = hypot(offset[X_AXIS],offset[Z_AXIS]);
    axis_0 = X_AXIS;
    axis_1 = Z_AXIS;
    axis_linear = Y_AXIS;
  } else if (!isnan(offset[Y_AXIS]) && !isnan(offset[Z_AXIS])){
    if(isnan(offset[X_AXIS])) { offset[X_AXIS] = 0; };
    radius = hypot(offset[Y_AXIS],offset[Z_AXIS]);
    axis_0 = Y_AXIS;
    axis_1 = Z_AXIS;
    axis_linear = X_AXIS;
  }

  String log = "Axis: " + String(axis_0) + "," + String(axis_1) + "," + String(axis_linear);
  Serial.println(log);

  log = "Start: [" + String(arc_start[axis_0]) + "," + String(arc_start[axis_1]) + "], target: [" + String(arc_target[axis_0]) + "," + String(arc_target[axis_1]) + "]";
  Serial.println(log);
  
  center_axis0 = arc_start[axis_0] + offset[axis_0];
  center_axis1 = arc_start[axis_1] + offset[axis_1];
  linear_travel = target[axis_linear] - arc_start[axis_linear];
  //extruder_travel = extruder;
  r_axis0 = -offset[axis_0];  // Radius vector from center to current location
  r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;
  
  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  angular_travel = 
    atan2(
        r_axis0 * rt_axis1
      - r_axis1 * rt_axis0, 
        r_axis0 * rt_axis0
      + r_axis1 * rt_axis1 
    );
  if (angular_travel < 0) { angular_travel += 2*M_PI; }
  if (is_clockwise_arc) { angular_travel -= 2*M_PI; }
  
  //20141002:full circle for G03 did not work, e.g. G03 X80 Y80 I20 J0 F2000 is giving an Angle of zero so head is not moving
  //to compensate when start pos = target pos && angle is zero -> angle = 2Pi
  if (arc_start[axis_0] == target[axis_0] && arc_start[axis_1] == target[axis_1] && angular_travel == 0)
  {
    angular_travel += 2*M_PI;
  }
  //end fix G03
  
  float millimeters_of_travel = hypot(angular_travel*radius,fabs(linear_travel));
  if (millimeters_of_travel < 0.001) { return; }
  segments_nr = floor(millimeters_of_travel/MM_PER_ARC_SEGMENT);
  log = "Angular Travel: " + String(angular_travel) + ", center: [" + String(center_axis0) + "," + String(center_axis1) + "], r: " + String(radius) + ",lt: " + String(fabs(linear_travel)) +", seg:" + segments_nr;
  Serial.println(log);
  if(segments_nr == 0) segments_nr = 1;
  segments_index = 1;
  /*  
    // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
    // by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
    // all segments.
    if (invert_feed_rate) { feed_rate *= segments; }
  */
  state = STATE_ARC_INTERPOLATION;
}

void Interpolation::preventOverflow(float target[4])
{ 
  if(target[X_AXIS] < X_MIN) target[X_AXIS] = X_MIN;
  if(target[Y_AXIS] < Y_MIN) target[Y_AXIS] = Y_MIN;
  if(target[Z_AXIS] < Z_MIN) target[Z_AXIS] = Z_MIN;
  if(target[E_AXIS] < E_MIN) target[E_AXIS] = E_MIN;

  if (target[X_AXIS] > X_MAX) target[X_AXIS] = X_MAX;
  if (target[Y_AXIS] > Y_MAX) target[Y_AXIS] = Y_MAX;
  if (target[Z_AXIS] > Z_MAX) target[Z_AXIS] = Z_MAX;
  if (target[X_AXIS] > X_MAX) target[X_AXIS] = X_MAX;
  //return target;
}
