//========================================================
// servo.cpp
// servo functions
//========================================================

#include "Arduino.h"
#include "debug.h"
#include "com.h"

#include <PololuMaestro.h>

#ifdef SERIAL_PORT_HARDWARE_OPEN
  //#define maestroSerial SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial Serial3
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(10, 11);
#endif

// create maestro, a MiniMaestro object
MiniMaestro maestro(maestroSerial);
#include "servo.h"

// each of the 12 servos needs to be trimmed so 1,500 usec pulses result in center position
// the controller used has 1/4 usec accuacy so a value 6,000 should correspond to center
const boolean servo_debug = false;
const uint8_t num_servos = NUM_LEGS * NUM_JOINTS_LEG; // calculate once based on NUM_LEGS and NUM_JOINTS_LEG defined in common.h
const uint8_t first_servo = 12; // set to 0 if using Maestro 12 and to 12 if using Maestro 24 with A/D's on first 12 inputs
const uint16_t servo_at_zero[num_servos] = {uint16_t(4*1491.00), uint16_t(4*1489.00), uint16_t(4*1516.75),
                                            uint16_t(4*1471.50), uint16_t(4*1515.50), uint16_t(4*1511.50),
                                            uint16_t(4*1500.75), uint16_t(4*1417.50), uint16_t(4*1564.50),
                                            uint16_t(4*1530.25), uint16_t(4*1476.25), uint16_t(4*1535.00)};
const uint16_t servo_at_max[num_servos] =  {uint16_t(4*1491.00), uint16_t(4*1814.00), uint16_t(4*1843.25),
                                            uint16_t(4*1471.50), uint16_t(4*1848.25), uint16_t(4*1887.25),
                                            uint16_t(4*1500.75), uint16_t(4*1755.25), uint16_t(4*1882.50),
                                            uint16_t(4*1530.25), uint16_t(4*1821.00), uint16_t(4*1853.00)};
const uint16_t servo_at_min[num_servos] =  {uint16_t(4*1491.00), uint16_t(4*1305.00), uint16_t(4*1314.75),
                                            uint16_t(4*1471.50), uint16_t(4*1329.50), uint16_t(4*1329.50),
                                            uint16_t(4*1500.75), uint16_t(4*1256.00), uint16_t(4*1393.00), 
                                            uint16_t(4*1530.25), uint16_t(4*1305.00), uint16_t(4*1363.75)};
//const int16_t SERVO_MAX_PIVOT = (2*SERVO_HALF_PI)/3; //
//const int16_t servo_a_maxngle[] = { SERVO_MAX_PIVOT,  SERVO_HALF_PI,  SERVO_HALF_PI,  SERVO_MAX_PIVOT,  SERVO_HALF_PI,  SERVO_HALF_PI,
//                                    SERVO_MAX_PIVOT,  SERVO_HALF_PI,  SERVO_HALF_PI,  SERVO_MAX_PIVOT,  SERVO_HALF_PI,  SERVO_HALF_PI}; // radians scaled up by
//const int16_t SERVO_MIN_ANGLE[] = {-SERVO_MAX_PIVOT, -my_math_quar_pi, -my_math_quar_pi, -SERVO_MAX_PIVOT, -my_math_quar_pi, -my_math_quar_pi,
//                                   -SERVO_MAX_PIVOT, -my_math_quar_pi, -my_math_quar_pi, -SERVO_MAX_PIVOT, -my_math_quar_pi, -my_math_quar_pi}; // radians scaled up by
float servo_pos_target_scale[num_servos]; // target per radian if radian >= 0.0
float servo_neg_target_scale[num_servos]; // target per radian if radian < 0.0


//========================================================
// servo_setup
//========================================================
void servo_setup(){
  const boolean local_debug = false;
  if (local_debug) DEBUG_PRINTLN("In servo_setup");
  // Set the serial baud rate.
  maestroSerial.begin(115200);  
  //maestroSerial.begin(9600);  
  //set_servos(servo_trim); // set all servos to their center position
  // calculate servo gains which will be used continuously
  // the servo gain is (servo_at_90 - servo_at_zero) / pi 
//  int32_t big_num; // hold intermediate calculations
//  const uint8_t shift_up = 16; // shift up to make big_num
//  const uint8_t final_shift = my_math_radian_shift + servo_target_scale_shift - shift_up; // ex: 2 = 13 + 5 - 16
  // calculate positive and negative angles to target scales for each servo
  for(uint8_t leg=0; leg<NUM_LEGS; leg++){
    for(uint8_t joint=0; joint<NUM_JOINTS_LEG; joint++){
      uint8_t servo = (leg*NUM_JOINTS_LEG) + joint;
      servo_pos_target_scale[servo] = (float(servo_at_max[servo] - servo_at_zero[servo])) / SERVO_MAX_ANGLE[leg][joint];
      servo_neg_target_scale[servo] = (float(servo_at_min[servo] - servo_at_zero[servo])) / SERVO_MIN_ANGLE[leg][joint];
    }
  }
} // end servo_setup


//========================================================
// servo_get_target gets position of servo to target
// target is 4 times the value of the pulse width in usec
//========================================================
uint16_t servo_get_target(uint8_t servo){
  if (servo_debug) DEBUG_PRINT("In servo_get_target, servo: ");
  if (servo_debug) DEBUG_PRINT(servo);
  if (servo_debug) DEBUG_PRINT(", servo + first_servo: ");
  if (servo_debug) DEBUG_PRINT(servo + first_servo);
  if (servo_debug) DEBUG_PRINTLN();
  return maestro.getPosition(servo + first_servo); // servos before first_servo are inputs
} // end servo_get_target


//========================================================
// servo_set_target sets position of servo to target
// target is 4 times the value of the pulse width in usec
//========================================================
void servo_set_target(uint8_t servo, uint16_t target){
  //if (servo_debug) DEBUG_PRINTLN("In servo_set_target");
  if (servo_debug) {
    DEBUG_PRINT("In servo_set_target, servo: ");
    DEBUG_PRINT(servo);
    DEBUG_PRINT(", target: ");
    DEBUG_PRINTLN(target);
  }
  maestro.setTarget(servo + first_servo, target); // servos before first_servo are inputs
}
// end servo_set_target


//========================================================
// servo_set_targets sets position of all 12 servos to targets
// target is 4 times the value of the pulse width in usec
//========================================================
void servo_set_targets(uint16_t target[]){
  if (servo_debug) DEBUG_PRINTLN("In servo_set_targets");
  for(uint8_t i=0; i<num_servos; i++){
    maestro.setTarget(i, target[i]);
  }
} // end servo_set_targets


//========================================================
// servo_angle_to_target sets a servo to a specified angle
// this routine should take into account offsets and non-linearities
//========================================================
uint16_t servo_angle_to_target(uint8_t servo, float radian){
  const boolean local_debug = false;
  if (local_debug){
    DEBUG_PRINT("In servo_angle_to_target, servo: ");
    DEBUG_PRINT(servo);
    DEBUG_PRINT(", radian: ");
    DEBUG_PRINTF("%7.2f", radian);
  }
//  const char routine[] = "servo_angle_to_target";
// const char out_of_range[] = "angle out of range";
  uint16_t target;
  if (radian >= 0.0) {
    target = uint16_t(radian * servo_pos_target_scale[servo]); // convert radians to target scale
    if(target > servo_at_max[servo]) {
      target = servo_at_max[servo];
//      com_err_msg(routine, out_of_range, servo, radian);
    }
  } else {
    target = uint16_t(radian * servo_neg_target_scale[servo]); // convert radians to target scale
    if(target < servo_at_min[servo]) {
      target = servo_at_min[servo];
//      com_err_msg(routine, out_of_range, servo, radian);
    }
  }
  if (servo_debug) {
    DEBUG_PRINT(", target1: ");
    DEBUG_PRINT(target);
  }
  target = target + servo_at_zero[servo]; // add the zero value
  if (servo_debug) {
    DEBUG_PRINT(", target2: ");
    DEBUG_PRINTLN(target);
  }
  return target;
} // end servo_angle_to_target


//========================================================
// servo_set_angle sets a servo to a specified angle
// the angle is specified in radians *2^14
// this routine should take into account offsets and non-linearities
//========================================================
void servo_set_angle(uint8_t servo, float radian){
  if (servo_debug) DEBUG_PRINTLN("In servo_set_angle");
  servo_set_target(servo, servo_angle_to_target(servo, radian)); // make it happen
} // end servo_angle_to_target


//========================================================
// servo_set_angles sets all servos to a specified angles
// the angles are specified in radians *2^14
// this routine should take into account offsets and non-linearities
//========================================================
void servo_set_angles(float radian[NUM_LEGS][NUM_JOINTS_LEG]){
  if (servo_debug) DEBUG_PRINTLN("In servo_set_angles");
  for(uint8_t leg=0; leg<NUM_LEGS; leg++){
    for(uint8_t joint=0; joint<NUM_JOINTS_LEG; joint++){
      servo_set_angle((leg * NUM_JOINTS_LEG) + joint, radian[leg][joint]);
    }
  }
} // end servo_set_angles


////========================================================
//// servo_test
////========================================================
//void servo_test(){
//  if (servo_debug) DEBUG_PRINT("In servo_test");
//  static const uint16_t counts_at_max = 200;
//  static const uint16_t counts_zero_to_max = 200;
//  static const uint16_t counts_at_zero = 100;
//  static const uint16_t counts_zero_to_min = 100;
//  static const uint16_t counts_at_min = 100;
//  static const uint8_t at_max = 0;
//  static const uint8_t zero_to_max = 1;
//  static const uint8_t at_zero = 2;
//  static const uint8_t zero_to_min = 3;
//  static const uint8_t at_min = 4;
//  
//  static uint8_t range = at_max; // start at max
//  static boolean going_down = true; // start at max
//  static boolean positive_angle = true; // start at max
//  static uint16_t counts = counts_at_max; // start at max
//  
//  static uint16_t i = 0; 
//  
//  i++;
//  switch (range){
//    case at_max:
//      if(i > counts){
//        i = 0;
//        going_down = true;
//        range = zero_to_max;
//        counts = counts_zero_to_max;
//      }
//      break;
//    case zero_to_max:
//      if(i > counts){
//        i = 0;
//        if(going_down){
//          range = at_zero;
//          counts = counts_at_zero;
//        } else {
//          range = at_max;
//          counts = counts_at_max;
//        }
//      } else {
//        // still moving
//        move_to(positive_angle, going_down, i, counts);
//      }
//      break;
//    case at_zero:
//      if(i > counts){
//        i = 0;
//        if(going_down){
//          range = zero_to_min;
//          counts = counts_zero_to_min;
//          positive_angle = false;
//        } else {
//          range = zero_to_max;
//          counts = counts_zero_to_max;
//          positive_angle = true;
//        }
//      } else {
//        // wait here
//      }
//      break;
//    case zero_to_min:
//      if(i > counts){
//        i = 0;
//        if(going_down){
//          range = at_min;
//          counts = counts_at_min;
//        } else {
//          range = at_zero;
//          counts = counts_at_zero;
//        }
//      } else {
//        // still moving
//        move_to(positive_angle, going_down, i, counts);
//      }
//      break;
//    case at_min:
//      if(i > counts){
//        i = 0;
//        going_down = false;
//        range = zero_to_min;
//        counts = counts_zero_to_min;
//      } else {
//        // do nothing
//      }
//      break;
//  }
//  if (false) {
//    DEBUG_PRINT("Servo_test \trange: ");
//    DEBUG_PRINT(range);
//    DEBUG_PRINT("\tpositive_angle: ");
//    DEBUG_PRINT(positive_angle);
//    DEBUG_PRINT("\tgoing_down: ");
//    DEBUG_PRINT(going_down);
//    DEBUG_PRINT("\ti: ");
//    DEBUG_PRINTLN(i);
//  }
//} // end servo_test
//
//
////========================================================
//// move_to
//// called by servo test
////========================================================
//void move_to(boolean positive_angle, boolean going_down, uint16_t i, uint16_t total_steps){
//  const boolean local_debug = false;
//  int16_t angle;
//  uint16_t steps;
//  int32_t big_num;
//  if ((going_down && positive_angle) || (!going_down && !positive_angle)) {
//    steps = total_steps - i;
//  } else {
//    steps = i;
//  }
//  if (local_debug) {
//    DEBUG_PRINT("Beg move_to, steps: ");
//    DEBUG_PRINT(steps);
//  }
//  for (uint8_t servo = 0; servo < num_servos; servo++){
//    if (positive_angle) {
//      //angle = (steps * servo_a_maxngle[servo]) / steps; // the intermediate result exceeds int16_t
//      big_num = steps * servo_a_maxngle[servo];
//      angle = int16_t(big_num / total_steps);
//    } else {
//      //angle = (steps * SERVO_MIN_ANGLE[servo]) / steps; // the intermediate result exceeds int16_t
//      big_num = steps * SERVO_MIN_ANGLE[servo];
//      angle = int16_t(big_num / total_steps);
//    }
//    servo_set_angle(servo, angle);
//    if (local_debug) {
//      DEBUG_PRINT("\tservo ");
//      DEBUG_PRINT(servo);
//      DEBUG_PRINT("\t angle: ");
//      DEBUG_PRINT(angle);
//    }
//  }
//  if (local_debug) {
//    DEBUG_PRINTLN();
//  }
//} // end move_to
//
//
////========================================================
//// servo_math_test
////========================================================
//void servo_math_test(){
//  if (servo_debug) DEBUG_PRINTLN("In servo_math_test");
//  uint8_t servo = 1;
//  uint8_t steps = 10;
//  int16_t angle_step = SERVO_HALF_PI / steps;
//  int16_t angle;
//  //uint16_t target;
//  for(int i=0; i<steps+1; i++){
//    if (servo_debug) DEBUG_PRINTLN("In servo_set_angle for loop");
//    angle = i*angle_step;
//    servo_set_angle(servo, angle);
//    //target = servo_get_target(0);
//    Serial.print("\t");
//    Serial.print(i);
//    Serial.print("\t");
//    Serial.print(angle);
//    Serial.print("\t");
//    Serial.println(servo_angle_to_target(servo, angle));
//    delay(1000);
//  }
//} // end servo_math_test
