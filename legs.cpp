//========================================================
// legs.cpp
// leg functions
// functions for updating walker position
//========================================================

#include "Arduino.h"
#include "com.h"
#include "legs.h"
#include "servo.h" // to call servo_set_angles 
#include "debug.h"
//#include "mode.h" // to call mode_print_move_part_points(uint8_t indent)

static float legs_foot_xyz_retracted[NUM_LEGS][XYZ]; // calculated at setup from above
static float legs_foot_xyz_ready[NUM_LEGS][XYZ]; // calculated at setup from above

//boolean legs_are_folded;
//boolean legs_are_unfolded;
float legs_xyz[NUM_LEGS][XYZ]; // xyz location of each foot, in mm relative to body
float legs_v_xyz[NUM_LEGS][XYZ]; // xyz velocity of each foot, in mm/sec relative to body
float legs_angle[NUM_LEGS][NUM_JOINTS_LEG]; // joint angles of each foot, in radians
//boolean legs_supporting[NUM_LEGS]; // if true, that leg is on the ground, only one may not be supporting 

//========================================================
// legs setup

//========================================================
void legs_setup(uint8_t indent){
  const boolean local_debug = true;
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg legs_setup");
  }
  //legs_are_folded = false; // don't know the state
  //legs_are_unfolded = false; // don't know the state
//  for (uint8_t leg = 0; leg < NUM_LEGS; leg++){
//    for (uint8_t joint = 0; joint < NUM_JOINTS_LEG; joint++){
//      // set each to their max angle (folded)
//      legs_angle[leg][joint] = SERVO_MAX_ANGLE[leg][joint];
//    }
//  }
  legs_compute_retracted_and_ready(indent+1);
  for(uint8_t leg=0; leg<NUM_LEGS; leg++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      legs_xyz[leg][coor] = legs_foot_xyz_retracted[leg][coor]; // set initial leg coordinates to "retracted"
      legs_v_xyz[leg][coor] = 0.0; // set initial leg velocities to 0.0
    }
  }
  if(local_debug) legs_print_values("legs_xyz", legs_xyz, indent+1);
  legs_angles(legs_xyz, legs_angle); // update leg angles
  if(local_debug) legs_print_values("legs_angle", legs_angle, indent+1);
  servo_set_angles(legs_angle); // update servos
//  legs_are_folded = true; // now we know the state
//  legs_are_unfolded = false; // now we know the state
//  legs_active_sequence = LEGS_SEQ_FOLDED; // not in any sequence yet
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End legs_setup");
  }
} // end legs_setup


//========================================================
// legs_print_values
//========================================================
void legs_print_values(String value_name, float value[NUM_LEGS][XYZ], uint8_t indent){
  com_indent(indent);
  Serial.print("Beg legs_print_values, ");
  Serial.println(value_name);
  for(uint8_t leg=0; leg<NUM_LEGS; leg++){
    com_indent(indent+1);
    for(uint8_t coor=0; coor<XYZ; coor++){
      Serial.printf("%7.2f ", value[leg][coor]);
    }
    Serial.println();
  }
  com_indent(indent);
  Serial.println("End legs_print_values, ");
} // end legs_print_values


//========================================================
// legs_compute_retracted_and_ready
//========================================================
void legs_compute_retracted_and_ready(uint8_t indent){
  const boolean local_debug = false;
  if(local_debug) DEBUG_PRINTLN("Beg legs_compute_retracted_and_ready()");
  static const uint8_t xi = 0;
  static const uint8_t yi = 1;
//  static const uint8_t z = 2;
  
  for(uint8_t leg=0; leg<NUM_LEGS; leg++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      if((leg == 1) || (leg == 3)){
        if(coor == xi){
          legs_foot_xyz_retracted[leg][xi] = LEGS_FOOT_XYZ_SIGNS[leg][xi] * LEGS_XYZ_RETRACTED[yi];
        } else if (coor == yi){
          legs_foot_xyz_retracted[leg][yi] = LEGS_FOOT_XYZ_SIGNS[leg][yi] * LEGS_XYZ_RETRACTED[xi];
        } else {
          legs_foot_xyz_retracted[leg][coor] = LEGS_FOOT_XYZ_SIGNS[leg][coor] * LEGS_XYZ_RETRACTED[coor];
        }
      } else {
        legs_foot_xyz_retracted[leg][coor] = LEGS_FOOT_XYZ_SIGNS[leg][coor] * LEGS_XYZ_RETRACTED[coor];
      }
      legs_foot_xyz_ready[leg][coor] = LEGS_FOOT_XYZ_SIGNS[leg][coor] * LEGS_XYZ_READY[coor];
    }
  }  
  if(local_debug) DEBUG_PRINTLN("End legs_compute_retracted_and_ready()");
} // end legs_compute_retracted_and_ready


////========================================================
//// legs update
////========================================================
//void legs_update(){
//  switch (legs_active_sequence){
//    case LEGS_SEQ_UNFOLDING:
//      // continue unfolding
//      
//      break;
//    case LEGS_SEQ_FOLDING:
//      // continue folding
//      break;
//    case LEGS_SEQ_MOVING:
//      // continue moving
//      break;
//  }
//} // end legs_update


//========================================================
// legs position_tests
// tests:
//   void legs_position(float the_time, float v_max[XYZ], float a_max[XYZ], int8_t dir[XYZ], float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], float legs_position[3]){
//   void legs_coor_move_points(float move_xyz[XYZ], float v_max[XYZ], float a_max[XYZ], int8_t dir[XYZ], float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM]){
//   float legs_distance(float the_time, float v_max, float a_max, float move_point[3][LEGS_MOVE_TD_NUM]){
//   void legs_move_point(float v_max, float a_max, float distance, float move_point[3][LEGS_MOVE_TD_NUM], uint8_t indent){
//========================================================
void legs_position_tests(uint8_t indent){
  com_indent(indent);
  Serial.println("In legs_position_tests");
  //     LEGS_PARAM_NAME[LEGS_PARAM_NUM] = {"DIST", "DIR", "UPDN", "V_MAX", "A_MAX", "V_BEG", "V_END"};
  float parameters[XYZ][LEGS_PARAM_NUM] = {{   0.0,   1.0,   -1.0,   400.0,   200.0,     0.0,     0.0},
                                           { 300.0,   1.0,   -1.0,   400.0,   200.0,     0.0,     0.0},
                                           {  50.0,   1.0,    1.0,   400.0,   200.0,     0.0,     0.0}};
//  int8_t dir[3];
  for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
    com_indent(indent);
    Serial.print("\t");
    Serial.print(LEGS_PARAM_NAME[param]);
    Serial.print("\t");
    for(uint8_t coor=0; coor<XYZ; coor++){
      Serial.print("\t");
      Serial.print(parameters[coor][param]);
    }
    Serial.println();
  }
  float move_points[XYZ][LEGS_MOVE_POINT_NUM][2];
  legs_coor_move_points(parameters, move_points, indent+1);
  Serial.println("  after legs_coor_move_points()");
  for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
    DEBUG_INDENT(indent);
    Serial.print("\t");
    Serial.print(LEGS_PARAM_NAME[param]);
    Serial.print("\t");
    for(uint8_t coor=0; coor<XYZ; coor++){
      Serial.print("\t");
      Serial.print(parameters[coor][param]);
    }
    Serial.println();
  }
//  Serial.println();
//  Serial.print("dir:\t\t");
//  for(uint8_t coor=0; coor<3; coor++){
//    Serial.print("\t");
//    Serial.print(dir[coor]);
//  }
//  Serial.println();
  com_indent(indent);
  Serial.println("\tPoint_\t\ttd__\tX___\tY___\tZ___");
  for(uint8_t point=0; point<3; point++){
    com_indent(indent);
    for(uint8_t td=0; td<2; td++){
      Serial.print("\t");
      Serial.print(LEGS_MOVE_NAME[point]);
      Serial.print("\t");
      Serial.print(LEGS_MOVE_TD_NAME[td]);
      for(uint8_t coor=0; coor<3; coor++){
        Serial.print("\t");
        Serial.print(move_points[coor][point][td]);
      }
      Serial.println();
    }
  }
  for(uint8_t point=3; point<7; point++){
    com_indent(indent);
    for(uint8_t td=0; td<2; td++){
      Serial.print("\t");
      Serial.print(LEGS_MOVE_NAME[point]);
      Serial.print("\t");
      Serial.print(LEGS_MOVE_TD_NAME[td]);
      Serial.print("\t");
      Serial.print("\t");
      for(uint8_t coor=2; coor<3; coor++){
        Serial.print("\t");
        Serial.print(move_points[coor][point][td]);
      }
      Serial.println();
    }
  }
  // now calculate and display the x, y, z positions for the sequence over time
  float leg_position[3];
  uint8_t time_points = 50; // number of points to show for test
  float the_time;
  float end_time = move_points[0][0][0]; // x, point0, time
  com_indent(indent);
  Serial.println("\t\tTime_\tX___\tY___\tZ___");
  for(uint8_t point = 0; point<time_points; point++){
    the_time = float(point)/float(time_points-1)*end_time;
    legs_position(the_time, parameters, move_points, leg_position, indent+1);
    com_indent(indent);
    Serial.print("\t\t");
    Serial.print(the_time);
    for(uint8_t i=0; i<3; i++){
      Serial.print("\t");
      Serial.print(leg_position[i]);     
    }
    Serial.println();
  }
  com_indent(indent);
  Serial.println("End legs_position_tests");
} // end legs_position_tests


//========================================================
// legs position
// computes the x, y, z position for a given time based on 
// maximum velocities: {v_max_x, v_max_y, v_max_z}
// maximum accelerations: {a_max_x, a_max_y, a_max_z}
// and three arrays of move points
//   for x: {{end_time, end_dist}, {dec_time, dec_dist}, {cv_time, cv_dist}, , , , }
//   for y: {{end_time, end_dist}, {dec_time, dec_dist}, {cv_time, cv_dist}, , , , }
//   for z: {{end_time, end_dist}, {dec2_time, dec2_dist}, {cv2_time, cv2_dist}, {acc2_time, acc2_dist}, {wait_time, wait_dist}, {dec1_time, dec1_dist}, {cv1_time, cv1_dist}}
// remember, distances are alway positive!
//========================================================
void legs_position(float the_time, float parameters[XYZ][LEGS_PARAM_NUM], float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], float leg_position[XYZ], uint8_t indent){
  // get the x and y positions which only acc, optional constant vel, dec
  static const boolean local_debug = true;
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg legs_position");
  }
  for (uint8_t coor=0; coor<XYZ; coor++){
    leg_position[coor] = legs_distance(the_time, parameters[coor], move_points[coor], indent+1);
    if(parameters[coor][LEGS_PARAM_DIR] < 0.0) leg_position[coor] = -leg_position[coor];
  }
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End legs_position");
  }
} // end legs_position


//========================================================
// legs distance
// computes the distance for a given time based on a maximum velocity, maximum acceleration
// and an array of move points {{end_time, end_dist}, {dec_time, dec_dist}, {cv_time, cv_dist}}
// remember, distances are alway positive!
//========================================================
float legs_distance(float the_time, float parameters[LEGS_PARAM_NUM], float move_point[LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], uint8_t indent){
  float updn = parameters[LEGS_PARAM_UPDN];
  float v_max = parameters[LEGS_PARAM_V_MAX];
  float a_max = parameters[LEGS_PARAM_A_MAX];
  float v_beg = parameters[LEGS_PARAM_V_BEG];
  float v_end = parameters[LEGS_PARAM_V_END];
  float delta_time;
  float distance;

  static const boolean local_debug = true;
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINT("Beg legs_distance, the_time: ");
    DEBUG_PRINTF("%7.2f", the_time);
    DEBUG_PRINT(", updn: ");
    DEBUG_PRINTF("%7.2f", updn);
    DEBUG_PRINT(", v_max: ");
    DEBUG_PRINTF("%7.2f", v_max);
    DEBUG_PRINT(", a_max: ");
    DEBUG_PRINTF("%7.2f", a_max);
    DEBUG_PRINT(", v_beg: ");
    DEBUG_PRINTF("%7.2f", v_beg);
    DEBUG_PRINT(", v_end: ");
    DEBUG_PRINTF("%7.2f\n", v_end);
  }

  boolean up_down = updn > 0.0;
  if(the_time > move_point[LEGS_MOVE_END][LEGS_MOVE_TIME]){
    // after end of move
    delta_time = the_time - move_point[LEGS_MOVE_END][LEGS_MOVE_TIME]; // in this case it is time after the end
    if(local_debug) DEBUG_PRINTLN("at 0");
    distance = move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] + delta_time * v_end; // no more acc/dec, just the v_end
  } else if(the_time > move_point[LEGS_MOVE_DEC2][LEGS_MOVE_TIME]){

    // in deceleration 2, near end of 2nd movement
    delta_time = move_point[LEGS_MOVE_END][LEGS_MOVE_TIME] - the_time; // in this case it is time before the end
    if(up_down){
      if(local_debug) DEBUG_PRINTLN("at 1");
      distance = move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] + (0.5 * a_max * delta_time * delta_time); // ignores non-zero v_end
    } else {
      if(local_debug) DEBUG_PRINTLN("at 2");
      if(local_debug){
        DEBUG_PRINT("move_point[LEGS_MOVE_END][LEGS_MOVE_DIST]: ");
        DEBUG_PRINTF("%7.2f", move_point[LEGS_MOVE_END][LEGS_MOVE_DIST]);
        DEBUG_PRINT(", delta_time: ");
        DEBUG_PRINTF("%7.2f", delta_time);
        DEBUG_PRINT(", v_end: ");
        DEBUG_PRINTF("%7.2f", v_end);
        DEBUG_PRINT(", a_max: ");
        DEBUG_PRINTF("%7.2f", a_max);
        DEBUG_PRINT(", minus term: ");
        DEBUG_PRINTF("%7.2f\n", ((delta_time * v_end) + (0.5 * a_max * delta_time * delta_time)));
      }
      distance = move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] - ((delta_time * v_end) + (0.5 * a_max * delta_time * delta_time)); // handles non-zero v_end
    }
  } else if(the_time > move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME]){

    // during constant velocity 2, in middle of 2nd movement
    delta_time = the_time - move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME];
    if(up_down){
      if(local_debug) DEBUG_PRINTLN("at 3");
      distance = move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST] - (delta_time * v_max); // if up_down the cv2 is negative
    } else {
      if(local_debug) DEBUG_PRINTLN("at 4");
      distance = move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST] + (delta_time * v_max);
    }
  } else if(!up_down){

    // in acc if only one movement
    if(local_debug) DEBUG_PRINTLN("at 5");
    distance = (v_beg * the_time) + (0.5 * a_max * the_time * the_time); // handles non-zero v_beg
  } else if(the_time > move_point[LEGS_MOVE_ACC2][LEGS_MOVE_TIME]){

    // in acceleration 2, near start of 2nd movement
    delta_time = the_time - move_point[LEGS_MOVE_ACC2][LEGS_MOVE_TIME];
    if(local_debug) DEBUG_PRINTLN("at 6");
    distance = move_point[LEGS_MOVE_ACC2][LEGS_MOVE_DIST] - 0.5 * a_max * delta_time * delta_time; // ignores non-zero v_beg
  } else if(the_time > move_point[LEGS_MOVE_WAIT][LEGS_MOVE_TIME]){

    // in wait period, between two movements
    if(local_debug) DEBUG_PRINTLN("at 7");
    distance = move_point[LEGS_MOVE_WAIT][LEGS_MOVE_DIST];
  } else if(the_time > move_point[LEGS_MOVE_DEC1][LEGS_MOVE_TIME]){

    // in dec 1, near end of 1st movement
    delta_time = move_point[LEGS_MOVE_WAIT][LEGS_MOVE_TIME] - the_time; // the time before the end of the dec
    if(local_debug) DEBUG_PRINTLN("at 8");
    distance = move_point[LEGS_MOVE_WAIT][LEGS_MOVE_DIST] - 0.5 * a_max * delta_time * delta_time;
  } else if(the_time > move_point[LEGS_MOVE_CV1][LEGS_MOVE_TIME]){

    // in cv 1, in middle of 1st movement
    delta_time = the_time - move_point[LEGS_MOVE_CV1][LEGS_MOVE_TIME];
    if(local_debug) DEBUG_PRINTLN("at 9");
    distance = move_point[LEGS_MOVE_CV1][LEGS_MOVE_DIST] + v_max * delta_time;
  } else {

    // in acceleration 1, near start of 1st movement
    distance = 0.5 * a_max * the_time * the_time;
  }
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINT("End legs_distance, distance: ");
    DEBUG_PRINTLN(distance);
  }
  return distance;
} // end legs_distance


//========================================================
// legs_print_parameters
//========================================================
void legs_print_parameters(float parameters[XYZ][LEGS_PARAM_NUM], uint8_t indent){
  String coor_name[XYZ] = {"x:", "y:", "z:"};
  DEBUG_INDENT(indent);
  DEBUG_PRINTLN("Beg legs_print_parameters");
  DEBUG_INDENT(indent+1);
  DEBUG_PRINT("\t");
  for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
    DEBUG_PRINTF("  %s\t",LEGS_PARAM_NAME[param].c_str());
  }
  DEBUG_PRINTLN();
  for(uint8_t coor=0; coor<XYZ; coor++){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("%s",coor_name[coor].c_str());
    for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
      DEBUG_PRINTF("\t%7.2f",parameters[coor][param]);
    }
    DEBUG_PRINTLN();
  }
  DEBUG_INDENT(indent);
  DEBUG_PRINTLN("End legs_print_parameters");
} // end legs_print_parameters


//========================================================
// legs_print_move_points
//========================================================
void legs_print_move_points(float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], uint8_t indent){
  String coor_name[XYZ] = {"x", "y", "z"};
  DEBUG_INDENT(indent);
  DEBUG_PRINTLN("Beg legs_print_move_points");
  DEBUG_INDENT(indent+1);
  DEBUG_PRINT("\t\t");
  for(uint8_t point=0; point<LEGS_MOVE_POINT_NUM; point++){
    DEBUG_PRINTF("  %s\t",LEGS_MOVE_NAME[point].c_str());
  }
  DEBUG_PRINTLN();
  for(uint8_t coor=0; coor<XYZ; coor++){
    for(uint8_t td=0; td<LEGS_MOVE_TD_NUM; td++){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF("%s, %s:", coor_name[coor].c_str(), LEGS_MOVE_TD_NAME[td].c_str());
      for(uint8_t point=0; point<LEGS_MOVE_POINT_NUM; point++){
        DEBUG_PRINTF("\t%7.2f",move_points[coor][point][td]);
      }
      DEBUG_PRINTLN();
    }
  }
  DEBUG_INDENT(indent);
  DEBUG_PRINTLN("End legs_print_move_points");
} // end legs_print_move_points


//========================================================
// legs_coor_move_points
// computes an array of move points {{end_time, end_dist}, {dec_time, dec_dist}, {cv_time, cv_dist}}
// based on a distance, maximum velocity, maximum acceleration, beginning velocity, ending velocity
// const String LEGS_PARAM_NAME[] = {"dist", "v_max", "a_max", "v_beg", "v_end"};
// the v_max[] and a_max[] x or y values will probably be changed so that the x and y moves happen during the same time period
// the move_xyz z value may be reduced in order to allow up_down motion during the horizontal move
// note the path starts at x=0, y=0, z=0 and will change z to the specified z value and then return it to zero if parameters[zi][LEGS_PARAM_UPDN] is > 0.0
// remember, distances are alway positive! If parameters[coor][LEGS_PARAM_DIST] is not, it will become positive and the sign of parameters[coor][LEGS_PARAM_DIR] will be reversed
//========================================================
void legs_coor_move_points(float parameters[XYZ][LEGS_PARAM_NUM], float move_points[XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM], uint8_t indent){
  static const boolean local_debug = true;
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg legs_coor_move_points()");
  }
  static const uint8_t xi = 0; // coor index for x in move_point(s)
  static const uint8_t yi = 1; // coor index for y in move_point(s)
  static const uint8_t zi = 2; // coor index for z in move_point(s)
  //static const uint8_t ti = 0; // index for time in move_point(s)
  //static const uint8_t LEGS_MOVE_DIST = 1; // index for LEGS_MOVE_DISTst in move_point(s)
  // first convert postitions to distances and sign
  float z; // distance (positive value)
//  float x, y, z; // distance (positive value)
//  float dir_x, dir_y, dir_z; // direction (-1.0 or 1.0)
//  float x_abs, y_abs, z_abs;
  for(uint8_t coor=0; coor<XYZ; coor++){
    if(parameters[coor][LEGS_PARAM_DIST] >= 0.0){
      parameters[coor][LEGS_PARAM_DIR] = 1.0;
    } else {
      parameters[coor][LEGS_PARAM_DIST] = -parameters[coor][LEGS_PARAM_DIST];
      parameters[coor][LEGS_PARAM_DIR] = -1.0;      
    }
  }
//  com_sign_mag(parameters[xi][LEGS_PARAM_DIST], &dir[xi], &x_abs);
//  com_sign_mag(parameters[yi][LEGS_PARAM_DIST], &dir[yi], &y_abs);
//  com_sign_mag(parameters[zi][LEGS_PARAM_DIST], &dir[zi], &z_abs);
//  x = parameters[xi][LEGS_PARAM_DIST]; // non-negative value only
//  y = parameters[yi][LEGS_PARAM_DIST]; // non-negative value only
  z = parameters[zi][LEGS_PARAM_DIST]; // non-negative value only
//  dir_x = parameters[xi][LEGS_PARAM_DIR]; // direction (-1.0 or 1.0)
//  dir_y = parameters[yi][LEGS_PARAM_DIR]; // direction (-1.0 or 1.0)
//  dir_z = parameters[zi][LEGS_PARAM_DIR]; // direction (-1.0 or 1.0)
  if(local_debug){
    legs_print_parameters(parameters, indent+1);
//    DEBUG_INDENT(indent+1);
//    DEBUG_PRINT("\t\t __X___\t __Y___\t __Z___\n");
//    DEBUG_INDENT(indent+1);
//    DEBUG_PRINTF("DIR : \t%7.2f\t%7.2f\t%7.2f\n", dir_x, dir_y, dir_z);
//    DEBUG_INDENT(indent+1);
//    DEBUG_PRINTF("DIST: \t%7.2f\t%7.2f\t%7.2f\n", x, y, z);
//    DEBUG_INDENT(indent+1);
//    DEBUG_PRINT("UPDN: ");     
//    for(uint8_t coor=0; coor<XYZ; coor++){
//      DEBUG_PRINTF("\t%7.2f", parameters[coor][LEGS_PARAM_UPDN]);
//    }
//    DEBUG_PRINTLN();
  }

  // now compute the x and y times
  legs_move_point(parameters[xi], move_points[xi], indent+1); // calculate the move_points for x
  legs_move_point(parameters[yi], move_points[yi], indent+1); // calculate the move_points for y
  legs_move_point(parameters[zi], move_points[zi], indent+1); // calculate the move_points for z
  if(local_debug){
    legs_print_move_points(move_points, indent+1);
//    DEBUG_INDENT(indent+1);
//    DEBUG_PRINTF("X, end_t: %7.2f", move_points[xi][LEGS_MOVE_END][LEGS_MOVE_TIME]);
//    DEBUG_PRINTF(", dec_t: %7.2f", move_points[xi][LEGS_MOVE_DEC2][LEGS_MOVE_TIME]);
//    DEBUG_PRINTF(", cv_t: %7.2f\n", move_points[xi][LEGS_MOVE_CV2][LEGS_MOVE_TIME]);
//    DEBUG_INDENT(indent+1);
//    DEBUG_PRINTF("Y, end_t: %7.2f", move_points[yi][LEGS_MOVE_END][LEGS_MOVE_TIME]);
//    DEBUG_PRINTF(", dec_t: %7.2f", move_points[yi][LEGS_MOVE_DEC2][LEGS_MOVE_TIME]);
//    DEBUG_PRINTF(", cv_t: %7.2f\n", move_points[yi][LEGS_MOVE_CV2][LEGS_MOVE_TIME]);
  }

  // find the slowest time and use that for the move
  float scale;
//  static const float zero_thresh = 0.00001;
  // assume x move time was longer so slow down the y move
  uint8_t  fast_coor = yi;
  uint8_t  slow_coor = xi;
  if((abs(move_points[xi][LEGS_MOVE_END][LEGS_MOVE_TIME]) < COM_ZERO) && (abs(move_points[yi][LEGS_MOVE_END][LEGS_MOVE_TIME]) < COM_ZERO)){
    // both move times are zero, set scale = 1.0
    scale = 1.0;
  } else {
    if(move_points[xi][LEGS_MOVE_END][LEGS_MOVE_TIME] < move_points[yi][LEGS_MOVE_END][LEGS_MOVE_TIME]){
      // y move time was longer so slow down the x move
      fast_coor = xi;
      slow_coor = yi;
    } else {
      // don't need to do anything, already assumed x move time was longer so slow down the y move
    }
//    scale = move_points[fast_coor][LEGS_MOVE_END][LEGS_MOVE_TIME] / move_points[slow_coor][LEGS_MOVE_END][LEGS_MOVE_TIME]; // less than 1.0
  }
//  if(local_debug){
//    DEBUG_INDENT(indent+1);
//    DEBUG_PRINTF("h scale: %7.2f\n", scale);
//  }
//
//  parameters[fast_coor][LEGS_PARAM_V_MAX] = parameters[fast_coor][LEGS_PARAM_V_MAX] * scale;
//  parameters[fast_coor][LEGS_PARAM_A_MAX] = parameters[fast_coor][LEGS_PARAM_A_MAX] * scale * scale;
  float h_time = move_points[slow_coor][LEGS_MOVE_END][LEGS_MOVE_TIME]; // horizontal move time is the longer of the two
//  legs_move_scale_a_max(parameters[fast_coor], h_time, indent+1); // scale LEGS_PARAM_A_MAX in the fast coor so that the fast move happens in h_time
  legs_move_point_scale_a_max(parameters[fast_coor], move_points[fast_coor], h_time, indent+1); // now recompute the move_points for the fast coordinate that we slowed down
//  move_points[fast_coor][LEGS_MOVE_END][LEGS_MOVE_TIME] = h_time; // in case the distance was zero the time would be zero, this sets it to the horizontal time
//
//  for (uint8_t xy=0; xy<2; xy++){
//    for (uint8_t point=0; point<3; point++){
//      move_points[xy][point][LEGS_MOVE_TIME] = move_points[zi][point][LEGS_MOVE_TIME]; // keep the same times for x and y as the horizontal
//      move_points[xy][point][LEGS_MOVE_DIST] = xy_scale[xy] * move_points[zi][point][LEGS_MOVE_DIST]; // scale the distances for x and y from the horizontal
//    }
//    parameters[xy][LEGS_PARAM_V_MAX] = parameters[xy][LEGS_PARAM_V_MAX] * xy_scale[xy];
//    parameters[xy][LEGS_PARAM_A_MAX] = parameters[xy][LEGS_PARAM_A_MAX] * xy_scale[xy];
//  }
  
  // now compute the z times and distances
  legs_move_point(parameters[zi], move_points[zi], indent+1); // calculate points for the initial upward vertical move
  float v_time = move_points[zi][LEGS_MOVE_END][LEGS_MOVE_TIME]; // z, at end, time
  if(parameters[zi][LEGS_PARAM_UPDN] > 0.0){
    // this is an up/down motion
    // check to see if we can make a vertical move up and a vertical move down in the time of the horizontal move
    // RIGHT NOW THIS ASSUMES v_beg = v_end = 0, the following needs to be changed to accomodate v_beg and v_end !!!
    // RIGHT NOW THIS ASSUMES that the foot will end at a displacement of 0.0, the DIST value is used as the height that the foot will be lifted.
    float half_h_time = 0.5 * h_time;
    if(local_debug){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINT("v_time: ");
      DEBUG_PRINTF("%7.2f", v_time);
      DEBUG_PRINT(", half_h_time: ");
      DEBUG_PRINTF("%7.2f\n", half_h_time);
    }
    if(v_time <= half_h_time){
      // good, we can do the vertical up and down in h_time
      if(local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTLN("good, we can do the vertical up and down in h_time.");
      }
    }else{
      // we will reduce the vertical distance to fit the vertical up and down in h_time
      // in other words reduce the vertical to fit the vertical acc up dec up (or down) in half_h_time
      z = 0.25 * parameters[zi][LEGS_PARAM_A_MAX] * half_h_time * half_h_time; // total z = 2.0 * 0.5*a_max*(half_h_time/2)*(half_h_time/2)
      parameters[zi][LEGS_PARAM_DIST] = z; // update the parameters with the new distance
      legs_move_point(parameters[zi], move_points[zi], indent+1); // recalculate points for the initial upward new vertical move
      v_time = move_points[zi][LEGS_MOVE_END][LEGS_MOVE_TIME]; // z, at end, time
      if(local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTLN("we can't do the vertical up and down in h_time.");
        DEBUG_INDENT(indent+1);
        DEBUG_PRINT("Reducing the  vertical move to: ");
        DEBUG_PRINTF("%7.2f\n", z);
      }
    }
    // now shift the values and compute the other move_points so we include the downward part of the vertical move
    // for z: {{end_time, end_dist}, {dec2_time, dec2_dist}, {cv2_time, cv2_dist}, {acc2_time, acc2_dist}, {wait_time, wait_dist}, {dec1_time, dec1_dist}, {cv1_time, cv1_dist}}
    // get some info on acc/dec and cv periods
    float acc_t =  move_points[zi][LEGS_MOVE_CV2][LEGS_MOVE_TIME]; // all 4 acc/dec periods have the same time
    float acc_d =  move_points[zi][LEGS_MOVE_CV2][LEGS_MOVE_DIST]; // all 4 acc/dec periods have the same distance
    float cv_t =  move_points[zi][LEGS_MOVE_DEC2][LEGS_MOVE_TIME] - acc_t; // both cv periods have the same time
    float cv_d =  move_points[zi][LEGS_MOVE_DEC2][LEGS_MOVE_DIST] - acc_d; // both cv periods have the same distance
    // calculate time and distance for the first 3 points from the start
    move_points[zi][LEGS_MOVE_CV1][LEGS_MOVE_TIME] = acc_t; // at start of cv 1
    move_points[zi][LEGS_MOVE_CV1][LEGS_MOVE_DIST] = acc_d; // at start of cv 1
    move_points[zi][LEGS_MOVE_DEC1][LEGS_MOVE_TIME] = move_points[zi][LEGS_MOVE_CV1][LEGS_MOVE_TIME] + cv_t; // at start of dec 1
    move_points[zi][LEGS_MOVE_DEC1][LEGS_MOVE_DIST] = move_points[zi][LEGS_MOVE_CV1][LEGS_MOVE_DIST] + cv_d; // at start of dec 1
    move_points[zi][LEGS_MOVE_WAIT][LEGS_MOVE_TIME] = move_points[zi][LEGS_MOVE_DEC1][LEGS_MOVE_TIME] + acc_t; // at start of wait
    move_points[zi][LEGS_MOVE_WAIT][LEGS_MOVE_DIST] = move_points[zi][LEGS_MOVE_DEC1][LEGS_MOVE_DIST] + acc_d; // at start of wait
    // calculate time and distance for the last 4 points from the end
    move_points[zi][LEGS_MOVE_END][LEGS_MOVE_TIME] = h_time; // the end of the horizontal move
    move_points[zi][LEGS_MOVE_END][LEGS_MOVE_DIST] = 0.0; // the end of the horizontal move, back to z = 0.0
    move_points[zi][LEGS_MOVE_DEC2][LEGS_MOVE_TIME] = move_points[zi][LEGS_MOVE_END][LEGS_MOVE_TIME] - acc_t; // at start of dec 2
    move_points[zi][LEGS_MOVE_DEC2][LEGS_MOVE_DIST] = move_points[zi][LEGS_MOVE_END][LEGS_MOVE_DIST] + acc_d; // at start of dec 2
    move_points[zi][LEGS_MOVE_CV2][LEGS_MOVE_TIME] = move_points[zi][LEGS_MOVE_DEC2][LEGS_MOVE_TIME] - cv_t; // at start of cv 2
    move_points[zi][LEGS_MOVE_CV2][LEGS_MOVE_DIST] = move_points[zi][LEGS_MOVE_DEC2][LEGS_MOVE_DIST] + cv_d; // at start of cv 2
    move_points[zi][LEGS_MOVE_ACC2][LEGS_MOVE_TIME] = move_points[zi][LEGS_MOVE_CV2][LEGS_MOVE_TIME] - acc_t; // at start of acc 2
    move_points[zi][LEGS_MOVE_ACC2][LEGS_MOVE_DIST] = move_points[zi][LEGS_MOVE_CV2][LEGS_MOVE_DIST] + acc_d; // at start of acc 2
  } else {
    // this is not an up/down move
    if(local_debug){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTLN("this is not an up/down move");
    }
    if(v_time <= h_time){
      // if the z move (v_time) is shorter than the h_time (slowest time for x or y) then we're going to have to scale to slow z !!!
      // xy move time was longer so slow down the z move
      if(local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTLN("the z move (v_time) is shorter than the h_time");
      }
      scale = v_time/h_time;
      fast_coor = zi;
      //slow_coor = yi;
      parameters[fast_coor][LEGS_PARAM_V_MAX] = parameters[fast_coor][LEGS_PARAM_V_MAX] * scale;
      parameters[fast_coor][LEGS_PARAM_A_MAX] = parameters[fast_coor][LEGS_PARAM_A_MAX] * scale * scale;
      legs_move_point(parameters[fast_coor], move_points[fast_coor], indent+1); // now recompute the move_points for the fast coordinate that we slowed down
      move_points[fast_coor][LEGS_MOVE_END][LEGS_MOVE_TIME] = h_time; // in case the distance was zero the time would be zero, this sets it to the horizontal time
    } else {
      // if the z move (v_time) is longer than the h_time (slowest time for x or y) then we're going to have to scale to slow x and y !!!
      // z move time was longer so slow down the xy move
      if(local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTLN("the z move (v_time) is longer than the h_time");
      }
      scale = h_time/v_time;
      //slow_coor = zi;
      fast_coor = xi;
      parameters[fast_coor][LEGS_PARAM_V_MAX] = parameters[fast_coor][LEGS_PARAM_V_MAX] * scale;
      parameters[fast_coor][LEGS_PARAM_A_MAX] = parameters[fast_coor][LEGS_PARAM_A_MAX] * scale * scale;
      legs_move_point(parameters[fast_coor], move_points[fast_coor], indent+1); // now recompute the move_points for the fast coordinate that we slowed down
      move_points[fast_coor][LEGS_MOVE_END][LEGS_MOVE_TIME] = v_time; // in case the distance was zero the time would be zero, this sets it to the vertical time
      fast_coor = yi;
      parameters[fast_coor][LEGS_PARAM_V_MAX] = parameters[fast_coor][LEGS_PARAM_V_MAX] * scale;
      parameters[fast_coor][LEGS_PARAM_A_MAX] = parameters[fast_coor][LEGS_PARAM_A_MAX] * scale * scale;
      legs_move_point(parameters[fast_coor], move_points[fast_coor], indent+1); // now recompute the move_points for the fast coordinate that we slowed down
      move_points[fast_coor][LEGS_MOVE_END][LEGS_MOVE_TIME] = v_time; // in case the distance was zero the time would be zero, this sets it to the vertical time
    }
    
//      if(local_debug){
//        DEBUG_INDENT(indent+1);
//        DEBUG_PRINTLN("the z move (v_time) is longer than the h_time");
//        //mode_print_move_part_points(indent+1);
//        DEBUG_INDENT(indent+1);
//        DEBUG_PRINT("Z, end_t: ");
//        DEBUG_PRINT(move_points[zi][LEGS_MOVE_END][LEGS_MOVE_TIME]);
//        DEBUG_PRINT(", dec_t: ");
//        DEBUG_PRINT(move_points[zi][LEGS_MOVE_DEC2][LEGS_MOVE_TIME]);
//        DEBUG_PRINT(", cv_t: ");
//        DEBUG_PRINTLN(move_points[zi][LEGS_MOVE_CV2][LEGS_MOVE_TIME]);
//      }
//      // for now just set the end time for all coordinates to the z end time !!!
//      move_points[xi][LEGS_MOVE_END][LEGS_MOVE_TIME] = move_points[zi][LEGS_MOVE_END][LEGS_MOVE_TIME];
//      move_points[yi][LEGS_MOVE_END][LEGS_MOVE_TIME] = move_points[zi][LEGS_MOVE_END][LEGS_MOVE_TIME];
  }
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End legs_coor_move_points");
  }
} // end legs_coor_move_points


//========================================================
// legs_move_point_scale_a_max
// calculates a scaled a_max to cause the move to happen during specified time = t
// note that in some cases of large v_beg and v_end that the a_max may be negative!
//========================================================
void legs_move_point_scale_a_max(float parameters[LEGS_PARAM_NUM], float move_point[3][LEGS_MOVE_TD_NUM], float t, uint8_t indent){
  const char routine[] = "legs_move_point_scale_a_max";
  const char term_lt_zero[] = "term less than zero";
  const char time_le_zero[] = "time less than or equal to zero";

  static const boolean local_debug = true;
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg legs_move_point_scale_a_max");
  }
  
  if(t <= COM_ZERO) com_err_msg(routine, time_le_zero); // error!
  
  // need to solve a quadratic of form a*a_max^2 + b*a_max + c = 0.0
  float distance = parameters[LEGS_PARAM_DIST]; // desired move distance
  float v_max = parameters[LEGS_PARAM_V_MAX]; // maximum allowed velocity
  float a_max = parameters[LEGS_PARAM_A_MAX]; // maximum allowed acceleration
  //float a_max; // maximum allowed acceleration
  float v_beg = parameters[LEGS_PARAM_V_BEG]; // velocity at beginning of move
  float v_end = parameters[LEGS_PARAM_V_END]; // velocity at end of move

  if(local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("distance: %7.2f", distance);
    DEBUG_PRINTF(", v_max: %7.2f", v_max);
    DEBUG_PRINTF(", a_max: %7.2f", a_max);
    DEBUG_PRINTF(", v_beg: %7.2f", v_beg);
    DEBUG_PRINTF(", v_end: %7.2f\n", v_end);
  }
  
  float a = t*t; // t^2
  float b = 2.0 * t * (v_beg + v_end) - 4.0 * distance; // 2*t*(v_beg + v_end) - 4*d
  float c = - (v_beg - v_end) * (v_beg - v_end); // -(v_beg-v_end)^2
  float term = b * b - (4.0 * a * c); // b^2-4*a*c
  if(abs(term) < COM_ZERO) term = 0.0; // in case term is close to zero, make it zero
  if(term < 0.0) com_err_msg(routine, term_lt_zero); // error!
  
  if(local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF(">>>> term is: %7.2f\n", term);
  }
  float sqrt_term = sqrt(term); // sqrt(b^2-4*a*c)
  if(local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF(">>>> sqrt_term is: %7.2f\n", sqrt_term);
  }
  float a_max_plus = -b + sqrt_term; // we want either the sum, or the difference
  float a_max_minus = -b - sqrt_term; // we want either the sum, or the difference
  if(abs(a_max_plus) >= abs(a_max_minus)){
    a_max = a_max_plus / (2.0 * a);
  } else {
    a_max = a_max_minus / (2.0 * a);
  }
  if(local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF(">>>> a_max is: %7.2f\n", a_max);
  }
  parameters[LEGS_PARAM_A_MAX] = a_max; // maximum allowed acceleration

  if(local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("distance: %7.2f", distance);
    DEBUG_PRINTF(", v_max: %7.2f", v_max);
    DEBUG_PRINTF(", a_max: %7.2f", a_max);
    DEBUG_PRINTF(", v_beg: %7.2f", v_beg);
    DEBUG_PRINTF(", v_end: %7.2f\n", v_end);
  }

  if((abs(distance) < COM_ZERO) && (abs(v_beg) < COM_ZERO) && (abs(v_end) < COM_ZERO)){
    // don't have to do any calculations, just set everything to zero
    for(uint8_t point=0; point<LEGS_MOVE_POINT_NUM; point++){
      for(uint8_t td=0; td<LEGS_MOVE_TD_NUM; td++){
        move_point[point][td] = 0.0;
      }
    }
    move_point[LEGS_MOVE_END][LEGS_MOVE_TIME] = t; // set the end time to the requested time
  } else {
    // non-zero move request
    float v_mid = 0.5*(t*a_max + v_beg + v_end);
    float t_beg = (v_mid - v_beg)/a_max;
    float t_end = (v_mid - v_end)/a_max;
    float acc_term = 0.5*a_max*t*t;
    float d_beg = v_beg*t_beg + acc_term;
    float d_end = v_end*t_end + acc_term;

    if(local_debug){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF("v_mid: %7.2f", v_mid);
      DEBUG_PRINTF(", t_beg: %7.2f", t_beg);
      DEBUG_PRINTF(", t_end: %7.2f", t_end);
      DEBUG_PRINTF(", d_beg: %7.2f", d_beg);
      DEBUG_PRINTF(", d_end: %7.2f\n", d_end);
    }

    move_point[LEGS_MOVE_END][LEGS_MOVE_TIME] = t; // end time
    move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] = distance; // end distance
    move_point[LEGS_MOVE_DEC2][LEGS_MOVE_TIME] = t_beg; // dec time
    move_point[LEGS_MOVE_DEC2][LEGS_MOVE_DIST] = d_beg; // dec distance
    move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME] = t_beg; // cv time, shouldn't be required but...
    move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST] = d_beg; // cv LEGS_MOVE_DISTst, shouldn't be required but...
  }
  
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End legs_move_point_scale_a_max");
  }
} // end legs_move_point_scale_a_max


//========================================================
// legs move point
// computes the key points required to move a given distance based on a maximum velocity and acceleration
// the returned array includes time and distance to cv (constant velocity / stop accelerating), to dec (decelerating) and to end
// calculates an array with times and distances of {{end_time, end_dist}, {dec_time, dec_dist}, {cv_time, cv_dist}} 
// the array could be used along with v_max, a_max to calculate the correct position at any given time
//========================================================
void legs_move_point(float parameters[LEGS_PARAM_NUM], float move_point[3][LEGS_MOVE_TD_NUM], uint8_t indent){
  float distance = parameters[LEGS_PARAM_DIST]; // desired move distance
  float v_max = parameters[LEGS_PARAM_V_MAX]; // maximum allowed velocity
  float a_max = parameters[LEGS_PARAM_A_MAX]; // maximum allowed acceleration
  float v_beg = parameters[LEGS_PARAM_V_BEG]; // velocity at beginning of move
  float v_end = parameters[LEGS_PARAM_V_END]; // velocity at end of move
  const char routine[] = "legs_move_point";
  const char v_beg_gt_v_max[] = "abs(v_beg) greater than v_max";
  const char v_end_gt_v_max[] = "abs(v_end) greater than v_max";

  static const boolean local_debug = true;
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg legs_move_point");
  }
  if(abs(v_beg) > v_max){
    com_err_msg(routine, v_beg_gt_v_max);
    Serial.print("abs(v_beg): ");
    Serial.print(abs(v_beg));
    Serial.print(" > v_max: ");
    Serial.println(v_max);
  }
  if(abs(v_end) > v_max){
    com_err_msg(routine, v_end_gt_v_max);
    Serial.print("abs(v_end): ");
    Serial.print(abs(v_end));
    Serial.print(" > v_max: ");
    Serial.println(v_max);
  }

  if(local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("distance: %7.2f", distance);
    DEBUG_PRINTF(", v_max: %7.2f", v_max);
    DEBUG_PRINTF(", a_max: %7.2f", a_max);
    DEBUG_PRINTF(", v_beg: %7.2f", v_beg);
    DEBUG_PRINTF(", v_end: %7.2f\n", v_end);
  }

  if((abs(distance) < COM_ZERO) && (abs(v_beg) < COM_ZERO) && (abs(v_end) < COM_ZERO)){
    // don't have to do any calculations, just set everything to zero
    for(uint8_t point=0; point<LEGS_MOVE_POINT_NUM; point++){
      for(uint8_t td=0; td<LEGS_MOVE_TD_NUM; td++){
        move_point[point][td] = 0.0;
      }
    }
  } else {
    // non-zero move request
    float t_to_vm = (v_max - v_beg) / a_max; // time to v_max from beginning
    float d_to_vm = (v_beg * t_to_vm) + (0.5 * a_max * t_to_vm * t_to_vm); // distance to v_max from beginning
    float t_fm_vm = (v_max - v_end) / a_max; // time from v_max to end
    float d_fm_vm = (v_end * t_fm_vm) + (0.5 * a_max * t_fm_vm * t_fm_vm); // distance from v_max to end
    if(local_debug){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF("t_to_vm: %7.2f", t_to_vm);
      DEBUG_PRINTF(", d_to_vm: %7.2f", d_to_vm);
      DEBUG_PRINTF(", t_fm_vm: %7.2f", t_fm_vm);
      DEBUG_PRINTF(", d_fm_vm: %7.2f\n", d_fm_vm);
    }
    float end_time;
    if((d_to_vm + d_fm_vm) < distance){
      // limited by v_max
      if(local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTLN("limited by v_max");
      }
      end_time = ((distance - (d_to_vm + d_fm_vm)) / v_max) + t_to_vm + t_fm_vm; // end_time when limited by v_max
      move_point[LEGS_MOVE_END][LEGS_MOVE_TIME] = end_time; // end_time
      move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] = distance;
      move_point[LEGS_MOVE_DEC2][LEGS_MOVE_TIME] = end_time - t_fm_vm; // dec time
      move_point[LEGS_MOVE_DEC2][LEGS_MOVE_DIST] = distance - d_fm_vm; // dec distance
      move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME] = t_to_vm; // t_to_mv = start of cv_time
      move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST] = d_to_vm;
    }else{
      // limited by a_max, never get to v_max
      float v_mid = sqrt(distance * a_max + 0.5*(v_beg * v_beg + v_end * v_end)); // v_mid = sqrt(distance*a_max + 0.5*(v_beg^2 + v_end^2))
      float t_beg = (v_mid - v_beg) / a_max; // time from beginning to mid point
      float t_end = (v_mid - v_end) / a_max; // time from end to mid point
      //float d_beg = (v_beg * t_beg) + (0.5 * a_max * t_beg * t_beg); // distance from beginning to mid point, not required
      float d_end = (v_end * t_end) + (0.5 * a_max * t_end * t_end); // distance from end to mid point
      end_time = t_beg + t_end; // total time
      if(local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTF("limited by a_max, never get to v_max, v_mid: %7.2f", v_mid);
        DEBUG_PRINTF(", t_beg: %7.2f", t_beg);
        DEBUG_PRINTF(", t_end: %7.2f", t_end);
        DEBUG_PRINTF(", d_end: %7.2f", d_end);
        DEBUG_PRINTF(", end_time: %7.2f\n", end_time);
      }
      move_point[LEGS_MOVE_END][LEGS_MOVE_TIME] = end_time; // end time
      move_point[LEGS_MOVE_END][LEGS_MOVE_DIST] = distance; // end distance
      move_point[LEGS_MOVE_DEC2][LEGS_MOVE_TIME] = end_time - t_end; // dec time
      move_point[LEGS_MOVE_DEC2][LEGS_MOVE_DIST] = distance - d_end; // dec distance
      move_point[LEGS_MOVE_CV2][LEGS_MOVE_TIME] = end_time - t_end; // cv time, shouldn't be required but...
      move_point[LEGS_MOVE_CV2][LEGS_MOVE_DIST] = distance - d_end; // cv LEGS_MOVE_DISTst, shouldn't be required but...
    }
  }
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End legs_move_point");
  }
} // end legs_move_point

  
//========================================================
// legs angles
// computes the angles for the three leg joints for all legs (angle_phk[foot][phk]
// input the xyz coordiates of each leg foot (foot_xyz[foot][xyz])
//========================================================
void legs_angles(float foot_xyz[NUM_LEGS][3], float angle_phk[NUM_LEGS][NUM_JOINTS_LEG]){
  const boolean local_debug = false;
  if(local_debug) DEBUG_PRINTLN("In legs_angles(): ");
  for(uint8_t leg=0; leg<NUM_LEGS; leg++){
    leg_angles(leg, foot_xyz[leg], angle_phk[leg]);
  }
  if(local_debug){
    for(uint8_t leg=0; leg<NUM_LEGS; leg++){
      for(uint8_t joint=0; joint<NUM_JOINTS_LEG; joint++){
        DEBUG_PRINT("\t");
        DEBUG_PRINTF("%7.2f", angle_phk[leg][joint]);
      }
      DEBUG_PRINTLN();
    }
  }
} // legs_angles


//========================================================
// leg angles
// computes the angles for the three leg joints
// input leg index and the xyz coordiates of the leg foot
//========================================================
void leg_angles(uint8_t leg, float foot_xyz[3], float angle[NUM_JOINTS_LEG]){
  const boolean local_debug = false;
  if(local_debug) DEBUG_PRINT("In leg_angles: ");
  //const static uint8_t dim = 3;
  const static uint8_t x = 0;
  const static uint8_t y = 1;
  const static uint8_t z = 2;
  const static uint8_t a = 3;
  const static uint8_t pivot = 0;
  const static uint8_t hip = 1;
  const static uint8_t knee = 2;
  const static float pivot_xyza[NUM_LEGS][4] = {
    { LEGS_CENTER_2_PIVOT_XY,  LEGS_CENTER_2_PIVOT_XY, 0.0,  0.78540},
    { LEGS_CENTER_2_PIVOT_XY, -LEGS_CENTER_2_PIVOT_XY, 0.0,  2.35619},
    {-LEGS_CENTER_2_PIVOT_XY, -LEGS_CENTER_2_PIVOT_XY, 0.0, -2.35619},
    {-LEGS_CENTER_2_PIVOT_XY,  LEGS_CENTER_2_PIVOT_XY, 0.0, -0.78540}
  };
  float pivot2foot_xyz[XYZ];
  for(uint8_t i=0; i<XYZ; i++){
    pivot2foot_xyz[i] = foot_xyz[i] - pivot_xyza[leg][i];
  }
  float pivot_angle = atan2(pivot2foot_xyz[x],pivot2foot_xyz[y]);
  float pivot2foot_gnd = sqrt((pivot2foot_xyz[x] * pivot2foot_xyz[x]) + (pivot2foot_xyz[y] * pivot2foot_xyz[y]));
  float hip2foot_gnd = pivot2foot_gnd - LEGS_PIVOT_2_HIP;
  float hip2foot = sqrt((pivot2foot_xyz[z] * pivot2foot_xyz[z]) + (hip2foot_gnd * hip2foot_gnd));
  float hip2foot_angle = asin(pivot2foot_xyz[z] / hip2foot);
  float knee_angle = acos((LEGS_HIP_2_KNEE_SQR_PLUS_KNEE2FOOT_SQR - (hip2foot * hip2foot)) / LEGS_TWO_HIP2KNEE_TIMES_KNEE2FOOT);
  float thigh2hipfoot_angle = acos(((hip2foot * hip2foot) + LEGS_HIP_2_KNEE_SQR_MINUS_KNEE2FOOT_SQR) / (hip2foot * LEGS_TWO_HIP_2_KNEE));
  angle[hip] = hip2foot_angle + thigh2hipfoot_angle;
  angle[pivot] = pivot_angle - pivot_xyza[leg][a];
  angle[knee] = LEGS_PI_MINUS_KNEE_ANGLE_OFFSET - knee_angle;
  // we are not checking to make sure we haven't exceed the max angles
  // we will check for that in the call to servo_set_angle_to_target()
  if(local_debug){
    DEBUG_PRINT(", pivot2foot_x: ");
    DEBUG_PRINTF("%7.2f", pivot2foot_xyz[x]);
    DEBUG_PRINT(", pivot2foot_y: ");
    DEBUG_PRINTF("%7.2f", pivot2foot_xyz[y]);
    DEBUG_PRINT(", pivot_angle: ");
    DEBUG_PRINTF("%7.2f", pivot_angle);
    DEBUG_PRINT(", pivot2foot_gnd: ");
    DEBUG_PRINTF("%7.2f", pivot2foot_gnd);
    DEBUG_PRINT(", hip2foot_gnd: ");
    DEBUG_PRINTF("%7.2f", hip2foot_gnd);
    DEBUG_PRINT(", hip2foot: ");
    DEBUG_PRINTF("%7.2f", hip2foot);
    DEBUG_PRINT(", hip2foot_angle: ");
    DEBUG_PRINTF("%7.2f\n", hip2foot_angle);
  }
} // end leg_angles


////========================================================
//// legs_change_sequence
//// called from mode to start a new leg sequence
////========================================================
//boolean legs_change_sequence(uint8_t new_request){
//  boolean request_is_valid;
//  switch (new_request){
//    case LEGS_SEQ_FOLDED:
//      switch (legs_active_sequence){
//        case LEGS_SEQ_FOLDED:
//        case LEGS_SEQ_FOLDING:
//          // currently LEGS_SEQ_FOLDED or LEGS_SEQ_FOLDING
//          // request to be folded
//          // OK, no change
//          request_is_valid = true;
//          break;
//        case LEGS_SEQ_UNFOLDED:
//        case LEGS_SEQ_UNFOLDING:
//          // currently LEGS_SEQ_UNFOLDING or LEGS_SEQ_UNFOLDED
//          // request to be folded
//          // OK, change to folding
//          legs_active_sequence = LEGS_SEQ_FOLDING;
//          request_is_valid = true;
//          break;
//        case LEGS_SEQ_MOVING:
//          // currently LEGS_SEQ_MOVING
//          // request to be folded
//          // Error, can't change to folded from moving, must change to unfolded (not moving) first
//          request_is_valid = true;
//          break;
//      }
//      break;
//    case LEGS_SEQ_UNFOLDING:
//      // Error, can't request a change to a transitory state
//      request_is_valid = false;
//      break;
//    case LEGS_SEQ_UNFOLDED:
//      switch (legs_active_sequence){
//        case LEGS_SEQ_FOLDED:
//        case LEGS_SEQ_FOLDING:
//          // currently LEGS_SEQ_FOLDED or LEGS_SEQ_FOLDING
//          // request to be unfolded
//          // OK, change to unfolding
//          legs_active_sequence = LEGS_SEQ_UNFOLDING;
//          request_is_valid = true;
//          break;
//        case LEGS_SEQ_UNFOLDING:
//        case LEGS_SEQ_UNFOLDED:
//          // currently LEGS_SEQ_UNFOLDING or LEGS_SEQ_UNFOLDED
//          // request to be unfolded
//          // OK, no change
//          request_is_valid = true;
//          break;
//        case LEGS_SEQ_MOVING:
//          // currently LEGS_SEQ_MOVING
//          // request to be unfolded
//          // OK, 
//          request_is_valid = false;
//          break;
//      }
//      break;
//    case LEGS_SEQ_FOLDING:
//      request_is_valid = true;
//      break;
//    case LEGS_SEQ_MOVING:
//      request_is_valid = true;
//      break;
//  }
//  return request_is_valid;
//} // end legs_change_sequence
//
//
////========================================================
//// legs sequence
//// returns legs_active_sequence
////========================================================
//uint8_t legs_sequence(void){
//  return legs_active_sequence;
//}
//
//
////========================================================
//// legs folded
//// returns true if legs_active_sequence == LEGS_SEQ_FOLDED
////========================================================
//boolean legs_folded(void){
//  return (legs_active_sequence == LEGS_SEQ_FOLDED);
//}
//
//
////========================================================
//// legs unfolded
//// returns true if legs_active_sequence == LEGS_SEQ_UNFOLDED
////========================================================
//boolean legs_unfolded(void){
//  return (legs_active_sequence == LEGS_SEQ_UNFOLDED);
//}
//
//
////========================================================
//// legs fold
//// sequence to fold legs, could be from any starting position
//// returns true when done
////========================================================
//boolean legs_fold(void){
//  if (legs_are_folded) {
//    return true; 
//  } else {
//    // fold legs
//    return false; // only if still in process of folding
//  }
//} // end legs_fold
//
//
////========================================================
//// legs unfold
//// sequence to unfold legs, could be from any starting position
//// returns true when done
////========================================================
//boolean legs_unfold(void){
//  if (legs_are_unfolded) {
//    return true; 
//  } else {
//    // unfold legs
//    return false; // only if still in process of unfolding
//  }
//} // end legs_unfold
