//========================================================
// mode.cpp
// mode variable and functions
// =======================================================

#include "Arduino.h"
#include "debug.h"
#include "com.h"
#include "mode.h"
#include "body.h"
#include "sbus.h"
#include "servo.h"
#include "legs.h"

mode_phase_t mode_phase; // mode and phase
mode_values_t mode_values;  // commands from radio [fold, vx, vy, vt, height, angle]
mode_seq_t mode_seq[MODE_NUM][MODE_PHASE_MAX_NUM]; // mode phase sequences (end of move xyz position and velocities)
float move_part_points[MODE_SEQ_PART_NUM][XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM]; // move points for moving parts
float move_part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM]; // parameters for each of the part move
float move_part_xyz[MODE_PART_NUM][XYZ]; // xyz coordinates of each part (each leg and the body)
float move_part_beg_xyz[MODE_PART_NUM][XYZ]; // xyz coordinates of each part at the beginning of the current move seq (each leg and the body)
uint8_t move_part_activity[MODE_PART_NUM]; // updated between move sequences, indicates the activity and type of each part (the part type: MODE_PART_TYPE_ & {"SUPPORT_LEG", "ACTIVE_LEG", "STATIC_BODY", "ACTIVE_BODY"})
uint8_t move_part_seq_part[MODE_PART_NUM]; // updated between move sequences, indicates which seq_part (if any) is associated with the part
//float move_part_v_end


//========================================================
// mode_setup()
// initialize mode_phase()
//========================================================
void mode_setup(uint8_t indent) {
  mode_phase.alternate = 0; // need to initialize this once only?
  mode_create_sequences(indent+1); // create the sequences that will be used over and over
  mode_initialize_parts(indent+1); // sets all of the move_part_xyz[MODE_PART_NUM][XYZ] and move_part_beg_xyz[MODE_PART_NUM][XYZ]
  mode_set(MODE_FOLDED, indent+1); // do this after everything else is set up
} // end mode_setup


//========================================================
// mode_initialize_parts
//========================================================
void mode_initialize_parts(uint8_t indent){
  static const boolean local_debug = true;
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg mode_initialize_parts");
  }
  static const uint8_t xi = 0;
  static const uint8_t yi = 1;
  static const uint8_t zi = 2;
  // set all part coordinates to 0.0 initially
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      move_part_xyz[part][coor] = 0.0; 
    }     
  }
  // set the leg coordinates to the retracted position
  for(uint8_t leg=0; leg<NUM_LEGS; leg++){
    for(uint8_t coor=0; coor<XYZ; coor++){ 
      if((leg == 1) || (leg == 3)){
        if(coor == xi){
          move_part_xyz[leg][xi] = LEGS_FOOT_XYZ_SIGNS[leg][xi] * LEGS_XYZ_RETRACTED[yi];
        } else if (coor == yi){
          move_part_xyz[leg][yi] = LEGS_FOOT_XYZ_SIGNS[leg][yi] * LEGS_XYZ_RETRACTED[xi];
        } else {
          move_part_xyz[leg][coor] = LEGS_FOOT_XYZ_SIGNS[leg][coor] * LEGS_XYZ_RETRACTED[coor];
        }
      } else {
        move_part_xyz[leg][coor] = LEGS_FOOT_XYZ_SIGNS[leg][coor] * LEGS_XYZ_RETRACTED[coor];
      }
    }
  }  
  // for the body the coordinates are all zero except for z
  move_part_xyz[MODE_PART_BODY][zi] = -LEGS_XYZ_RETRACTED[zi];

  // now set all part beginning coordinates to match the initial values
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      move_part_beg_xyz[part][coor] = move_part_xyz[part][coor]; 
    }     
  }
  if (local_debug){
    mode_print_parts_xyz("move_part_xyz", move_part_xyz, indent+1);
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End mode_initialize_parts");
  }
} // end mode_initialize_parts


//========================================================
// mode_create_sequences()
//========================================================
void mode_create_sequences(uint8_t indent) {
  static const boolean local_debug = true;
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg mode_create_sequences");
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTLN("READY\tRETRACTED");
    for(uint8_t coor=0; coor<XYZ; coor++){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTF("%7.2f", LEGS_XYZ_READY[coor]);
      DEBUG_PRINT("\t");
      DEBUG_PRINTF("%7.2f\n", LEGS_XYZ_RETRACTED[coor]);
    }
  }
  float step_time;
  static const float F_LFT = 50.0; // ?? foot lift amount
  legs_compute_retracted_and_ready(indent+1); // make sure legs_xyz_retracted and legs_xyz_retracted values are available
  static const float B_ADJ = 20.0; // body adjustment during extention/retraction
  static const float F_RDY = LEGS_XYZ_READY[0]; // nominal ready position, x & y
  static const float F_FLD = LEGS_XYZ_RETRACTED[0]; // nominal folded (retracted) position, x & y
  static const float F_RET = F_FLD + B_ADJ; // folded but account for the body shift during leg retraction
  static const float F_EXT = F_RDY + B_ADJ; // ready but account for the body shift during the leg extension
  static const float BZNOM = -LEGS_XYZ_READY[2]; // nominal body height
  static const float BZLOW = -LEGS_XYZ_RETRACTED[2]; // lowered body height
  static const float BZHAF = 0.5 * (BZNOM + BZLOW); // halfway between nominal and lowered body height
  // call mode_calc_step_time() with the active leg step to see how long it takes to move a foot F_EXT-F_FLD and then b_vel = B_STP / TIME
  step_time = mode_calc_step_time(F_EXT-LEGS_XYZ_RETRACTED[0], F_EXT-LEGS_XYZ_RETRACTED[1], F_LFT, indent+1);
  static const float B_ARC = 0.5 * PI * B_ADJ; // the arc is 1/4 of the circumference = 1/4 * PI * 2 * radius = 0.5 * PI * radius
  static float badjv = B_ARC / step_time; // body adjust velocity during extention/retraction

  if (local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("BZNOM: %7.2f, BZLOW: %7.2f, BZHAF: %7.2f, \n", BZNOM, BZLOW, BZHAF);
  }

  // the maximum stride, M_STD = 2.0 * (F_RDY - F_RAD), the maximum half stride, MHSTD = F_RDY - F_RAD
  // the maximum body step, B_STP = 0.5 * MHSTD, and the maximum half body step, BHSTP = 0.25 * MHSTD
  // the leg movement for these strides is 3/4 of the strides since the body movement adds another 1/4 of the stride during the leg movement
  // we'll use the stride minus the body step to get the foot step
  // while walking the rear foot start position should be -F_STP = -(M_STD - B_STP) and the end position will be - F_RAD
  // while walking the front foot start position should be +F_RAD and the end position will be F_STP = M_STD - B_STP
  // when beginning to walk the rear foot start position should be -F_RDY and the end position will be - F_RAD
  // when beginning to walk the front foot start position should be F_RDY and the end position will be F_STP = M_STD - F_RAD
  static const float F_RAD = 10.0; // foot radius, can't put two feet in the same place so use this to make sure they're at separated locations
  static const float MHSTD = F_RDY - F_RAD; // max half stride = the foot ready position minus the foot radius
  static const float M_STD = 2.00 * MHSTD; // max stride = 2.0 * (the foot ready position minus the foot radius)
  static const float B_STP = 0.50 * MHSTD; // body displacement of 1 step, y. ??? Placeholder, will be scaled up later
  static const float F_STP = M_STD - B_STP; // foot step = stride minus the body stride to get the distance relative to the body center
  // call legs_coor_move_points() with the active leg step to see how long it takes to move a foot F_STP and then b_vel = B_STP / TIME
  step_time = mode_calc_step_time(0.0, F_STP, F_LFT, indent+1);
  static float b_vel = B_STP / step_time; // body velocity, ??? Placeholder, will be scaled up later

  // explanation of move_seq:
  //                            {descriptive string,                    {one part,       {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  //                                                                    {another part,   {     x,     y,     z}, {end_vx, end_vy, end_vz}}};
  // body positions are relative to the body position at the start of the move for x and y, but z is absolute
  // active leg positions x, y are relative to the body position at the start of the mode, but z is just used for the leg lift amount
  // the end velocities are all absolute
  static const boolean last_phase = true;
  static const boolean not_last_phase = false;

  // sequence for folded     {last_phase, descriptive string,                   {part_id,        {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  mode_seq[MODE_FOLDED][0] = {last_phase, "folded",                            {{MODE_PART_NONE, {   0.0,   0.0, BZNOM}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  // sequence for ready     {last_phase, descriptive string,                    {part_id,        {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  mode_seq[MODE_READY][0] = {last_phase, "ready",                              {{MODE_PART_NONE, {   0.0,   0.0, BZNOM}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  // sequence to start walking    {last_phase, descriptive string,              {part_id,        {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  mode_seq[MODE_WALKING_BEG][0] = {not_last_phase, "beg w 1/2, move leg1",     {{MODE_PART_BODY, {   0.0,   0.0, BZNOM}, {   0.0,   0.0,   0.0}},  // don't move body yet
                                                                                {MODE_PART_LEG1, { F_RDY,-F_RAD, F_LFT}, {   0.0,   0.0,   0.0}}}};// move rear leg to -F_RAD
  mode_seq[MODE_WALKING_BEG][1] = {last_phase, "beg w 2/2, mov bdy, mov leg0", {{MODE_PART_BODY, {   0.0, B_STP, BZNOM}, {   0.0, b_vel,   0.0}},  // move body forward
                                                                                {MODE_PART_LEG0, { F_RDY, F_STP, F_LFT}, {   0.0,   0.0,   0.0}}}};// move front leg to -F_RAD

  // sequence to stop walking     {last_phase, descriptive string,              {part_id,        {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  mode_seq[MODE_WALKING_END][0] = {not_last_phase, "end walking, move leg1",   {{MODE_PART_BODY, {   0.0, B_STP, BZNOM}, {   0.0,   0.0,   0.0}},  // move forward
                                                                                {MODE_PART_LEG1, { F_RDY,-F_RDY, F_LFT}, {   0.0,   0.0,   0.0}}}};// move rear leg to -F_RDY
  mode_seq[MODE_WALKING_END][1] = {last_phase, "end walking, move leg0",       {{MODE_PART_BODY, {   0.0,   0.0, BZNOM}, {   0.0,   0.0,   0.0}},  // move forward
                                                                                {MODE_PART_LEG0, { F_RDY, F_RDY, F_LFT}, {   0.0,   0.0,   0.0}}}};// move front leg to F_RDY

  // sequence for walking,    {last_phase, descriptive string,                  {part_id,        {     x,     y,     z}, {end_vx, end_vy, end_vz}},
  mode_seq[MODE_WALKING][0] = {not_last_phase, "walking, move leg1",           {{MODE_PART_BODY, {   0.0, B_STP, BZNOM}, {   0.0, b_vel,   0.0}},  // move forward
                                                                                {MODE_PART_LEG1, { F_RDY,-F_RAD, F_LFT}, {   0.0,   0.0,   0.0}}}};// move rear leg to -F_RAD
  mode_seq[MODE_WALKING][1] = {last_phase, "walking, move leg0",               {{MODE_PART_BODY, {   0.0, B_STP, BZNOM}, {   0.0, b_vel,   0.0}},  // move forward
                                                                                {MODE_PART_LEG0, { F_RDY, F_STP, F_LFT}, {   0.0,   0.0,   0.0}}}};// move front leg to F_STP

  // sequence for sidestepping     {last_phase, descriptive string,             {part_id,        {     x,     y,     z}, {end_vx, end_vy, end_vz}}, 
  mode_seq[MODE_SIDESTEPPING][0] = {last_phase, "walking",                     {{MODE_PART_NONE, {   0.0,   0.0, BZNOM}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  // sequence for rotating,    {last_phase, descriptive string,                 {part_id,        {     x,     y,     z}, {end_vx, end_vy, end_vz}},  
  mode_seq[MODE_ROTATING][0] = {last_phase, "walking",                         {{MODE_PART_NONE, {   0.0,   0.0, BZNOM}, {   0.0,   0.0,   0.0}},  // do nothing
                                                                                {MODE_PART_NONE, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  // sequence for unfolding,    {last_phase,     descriptive string,            {part_id,        {     x,     y,     z}, {end_vx, end_vy, end_vz}},  
  mode_seq[MODE_UNFOLDING][0] = {not_last_phase, "raise body 1/2",             {{MODE_PART_BODY, {   0.0,   0.0, BZHAF}, {   0.0,   0.0,  20.0}},  // body: raise 1/2 way up
                                                                                {MODE_PART_NONE, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_UNFOLDING][1] = {not_last_phase, "raise 2/2, ready ext leg0",  {{MODE_PART_BODY, {   0.0,-B_ADJ, BZNOM}, {-badjv,   0.0,   0.0}},  // body: finish raising, move back
                                                                                {MODE_PART_NONE, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_UNFOLDING][2] = {not_last_phase, "ext leg0, ready to ext leg1",{{MODE_PART_BODY, {-B_ADJ,   0.0, BZNOM}, {   0.0, badjv,   0.0}},  // body: move left
                                                                                {MODE_PART_LEG0, { F_EXT, F_RDY, F_LFT}, {   0.0,   0.0,   0.0}}}}; // leg0: extend
  mode_seq[MODE_UNFOLDING][3] = {not_last_phase, "ext leg1, ready to ext leg2",{{MODE_PART_BODY, {   0.0, B_ADJ, BZNOM}, { badjv,   0.0,   0.0}},  // body: move right
                                                                                {MODE_PART_LEG1, { F_RDY,-F_EXT, F_LFT}, {   0.0,   0.0,   0.0}}}}; // leg1: extend
  mode_seq[MODE_UNFOLDING][4] = {not_last_phase, "ext leg2, ready to ext leg3",{{MODE_PART_BODY, { B_ADJ,   0.0, BZNOM}, {   0.0,-badjv,   0.0}},  // body: move right
                                                                                {MODE_PART_LEG2, {-F_EXT,-F_RDY, F_LFT}, {   0.0,   0.0,   0.0}}}}; // leg2: extend
  mode_seq[MODE_UNFOLDING][5] = {not_last_phase, "ext leg3, ready to ext leg0",{{MODE_PART_BODY, {   0.0,-B_ADJ, BZNOM}, {-badjv,   0.0,   0.0}},  // body: move left
                                                                                {MODE_PART_LEG3, {-F_RDY, F_EXT, F_LFT}, {   0.0,   0.0,   0.0}}}}; // leg3: extend
  mode_seq[MODE_UNFOLDING][6] = {last_phase , "center body",                   {{MODE_PART_BODY, {   0.0,   0.0, BZNOM}, {   0.0,   0.0,   0.0}},  // body: center
                                                                                {MODE_PART_NONE, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  // sequence for folding,    {last_phase,     descriptive string,              {part_id,        {     x,     y,     z}, {end_vx, end_vy, end_vz}}, 
  mode_seq[MODE_FOLDING][0] = {not_last_phase, "shift body, ready to ret leg0",{{MODE_PART_BODY, {   0.0,-B_ADJ, BZNOM}, {-badjv,   0.0,   0.0}},  // body: move back
                                                                                {MODE_PART_NONE, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_FOLDING][1] = {not_last_phase, "ret leg0, ready to ret leg1",  {{MODE_PART_BODY, {-B_ADJ,   0.0, BZNOM}, {   0.0, badjv,   0.0}},  // body: move left 
                                                                                {MODE_PART_LEG0, { F_RET, F_FLD, F_LFT}, {   0.0,   0.0,   0.0}}}}; // leg0: retract
  mode_seq[MODE_FOLDING][2] = {not_last_phase, "ret leg1, ready to ret leg2",  {{MODE_PART_BODY, {   0.0, B_ADJ, BZNOM}, { badjv,   0.0,   0.0}},  // body: move fwd
                                                                                {MODE_PART_LEG1, { F_FLD,-F_RET, F_LFT}, {   0.0,   0.0,   0.0}}}}; // leg1: retract
  mode_seq[MODE_FOLDING][3] = {not_last_phase, "ret leg2, ready to ret leg3",  {{MODE_PART_BODY, { B_ADJ,   0.0, BZNOM}, {   0.0,-badjv,   0.0}},  // body: move right
                                                                                {MODE_PART_LEG2, {-F_RET,-F_FLD, F_LFT}, {   0.0,   0.0,   0.0}}}}; // leg2: retract
  mode_seq[MODE_FOLDING][4] = {not_last_phase, "ret leg3, ready to ret leg0",  {{MODE_PART_BODY, {   0.0,-B_ADJ, BZNOM}, {-badjv,   0.0,   0.0}},  // body: move back
                                                                                {MODE_PART_LEG3, {-F_FLD, F_RET, F_LFT}, {   0.0,   0.0,   0.0}}}}; // leg3: retract
  mode_seq[MODE_FOLDING][5] = {not_last_phase, "center body, start lowering",  {{MODE_PART_BODY, {   0.0,   0.0, BZHAF}, {   0.0,   0.0,   0.0}},  // body: center, start lowering
                                                                                {MODE_PART_NONE, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};
  mode_seq[MODE_FOLDING][6] = {last_phase , "finish lowering",                 {{MODE_PART_BODY, {   0.0,   0.0, BZLOW}, {   0.0,   0.0,   0.0}},  // body: finish lowering
                                                                                {MODE_PART_NONE, {   0.0,   0.0,   0.0}, {   0.0,   0.0,   0.0}}}};

  if (local_debug){
    mode_print_sequences(indent+1);
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End mode_create_sequences");
  }
} // end mode_create_sequences


//========================================================
// mode_print_sequences()
//========================================================
void mode_print_sequences(uint8_t indent){
  static const boolean local_debug = true;
  mode_seq_t this_mode_seq;
  mode_seq_part_t seq_part;
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg mode_print_sequences");
    for(uint8_t mode=0; mode<MODE_NUM; mode++){
      for(uint8_t phase=0; phase<MODE_PHASE_MAX_NUM; phase++){
        this_mode_seq = mode_seq[mode][phase];
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTF("mode_seq[%s][%u]: ", MODE_NAME[mode].c_str(), phase);
        if(this_mode_seq.last_phase) {
          DEBUG_PRINT("last_phase");
        } else {
          DEBUG_PRINT("not_last_phase");
        }
        DEBUG_PRINTF(", %s\n", this_mode_seq.phase_name.c_str());
        for(uint8_t seq_part_i=0; seq_part_i<MODE_SEQ_PART_NUM; seq_part_i++){
          seq_part = this_mode_seq.part[seq_part_i];
          DEBUG_INDENT(indent+1);
          DEBUG_PRINT("-\t\t\t\t");
          DEBUG_PRINTF("%s D:", MODE_PART_NAME[seq_part.part_id].c_str());
          for(uint8_t coor=0; coor<XYZ; coor++){
            DEBUG_PRINTF(" %7.2f", seq_part.d[coor]);
          }
          DEBUG_PRINTF(", V:", MODE_PART_NAME[seq_part.part_id].c_str());
          for(uint8_t coor=0; coor<XYZ; coor++){
            DEBUG_PRINTF(" %7.2f", seq_part.v[coor]);
          }
          DEBUG_PRINTLN();
        }
        if(this_mode_seq.last_phase) break;
      }
    }
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End mode_print_sequences");
  }
} // end mode_print_sequences


//========================================================
// mode_calc_step_time()
// use the step_size and lift height to calculate the step_time
//========================================================
float mode_calc_step_time(float step_x, float step_y, float lift_height, uint8_t indent){
  static const boolean local_debug = true;
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg mode_calc_step_time");
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("step_x: %7.2f", step_x);
    DEBUG_PRINTF(", step_y: %7.2f", step_y);
    DEBUG_PRINTF(", lift_height: %7.2f\n", lift_height);
  }
  float step_time;
  //LEGS_XYZ_VA_MAX[XYZ][2]
  //     LEGS_PARAM_NAME[LEGS_PARAM_NUM] = {     "DIST", "DIR", "UPDN",               "V_MAX",               "A_MAX", "V_BEG", "V_END"};
  float parameters[XYZ][LEGS_PARAM_NUM] = {{     step_x,   1.0,   -1.0, LEGS_XYZ_VA_MAX[0][0], LEGS_XYZ_VA_MAX[0][1],     0.0,     0.0},
                                           {     step_y,   1.0,   -1.0, LEGS_XYZ_VA_MAX[1][0], LEGS_XYZ_VA_MAX[1][1],     0.0,     0.0},
                                           {lift_height,   1.0,    1.0, LEGS_XYZ_VA_MAX[2][0], LEGS_XYZ_VA_MAX[2][1],     0.0,     0.0}};
  float move_points[XYZ][LEGS_MOVE_POINT_NUM][2];
  legs_coor_move_points(parameters, move_points, indent+1);
  step_time = move_points[0][0][0];  
  if (local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINT("step_time: ");
    DEBUG_PRINTLN(step_time);
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End mode_calc_step_time");
  }
  return step_time;
} // end mode_calc_step_time


//========================================================
// mode_update()
// check to see if mode chould be changed based on current mode and inputs
// execute current mode sequence (new or previously started)
//========================================================
void mode_update(uint8_t indent) {
  static const boolean local_debug = true;
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg mode_update");
  }
  mode_check_new_mode(indent+1);
  mode_execute_seq(indent+1);
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End mode_update");
  }
} // end mode_update


//========================================================
// mode_check_new_mode()
// check to see if mode chould be changed based on current mode and inputs
//========================================================
void mode_check_new_mode(uint8_t indent) {
  uint8_t mode = mode_phase.mode; // get the mode
  static const boolean local_debug = true;
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINT("Beg mode_check_new_mode, current_mode: ");
    DEBUG_PRINTLN(MODE_NAME[mode]);
  }
  mode_values_update();
  switch (mode) {
    case MODE_FOLDED:
      // currently powered down, check if radio, power, orientation are OK
      if (!mode_values.fold) {
        mode_set(MODE_UNFOLDING, indent+1); // if !mode_value.fold, change mode to unfolding
//        if (local_debug){
//          DEBUG_PRINT("  new mode: ");
//          DEBUG_PRINTF("%7.2f\n", MODE_NAME[mode]);
//        }
//        body_start_unfolding(); // start the unfolding sequence
      } else {
        // continue in current mode, doing nothing
        //if (local_debug) DEBUG_PRINTLN();
      }
      break;
    case MODE_UNFOLDING:
      // don't do anything
      //if (local_debug) DEBUG_PRINTLN();
      break;
    case MODE_READY:
      float vy, vx, vt;
      vy= mode_value_vy();
      vx = mode_value_vx();
      vt = mode_value_vt();
      if (mode_value_fold()) {
        mode_set(MODE_FOLDING, indent+1);
//        if (local_debug){
//          DEBUG_PRINT("  new mode: ");
//          DEBUG_PRINTF("%7.2f\n", MODE_NAME[mode]);
//        }
//        body_start_folding();
      } else if(vy != 0.0) {
        // We're going to walk! Which direction though?
        if(vy > 0.0) {
          mode_set_dir(MODE_DIR_PLUS, indent+1);
        } else {
          mode_set_dir(MODE_DIR_MINUS, indent+1);
        }
        mode_set(MODE_WALKING_BEG, indent+1);
      } else if(vx != 0.0) {
        // We're going to sidestep! Which direction though?
        if(vx > 0.0) {
          mode_set_dir(MODE_DIR_PLUS, indent+1);
        } else {
          mode_set_dir(MODE_DIR_MINUS, indent+1);
        }
        mode_set(MODE_SIDESTEPPING_BEG, indent+1);
      } else if(vt != 0.0) {
        // We're going to rotate! Which direction though?
        if(vt > 0.0) {
          mode_set_dir(MODE_DIR_PLUS, indent+1);
        } else {
          mode_set_dir(MODE_DIR_MINUS, indent+1);
        }
        mode_set(MODE_ROTATING_BEG, indent+1);
      } else {
        // don't do anything
        //if (local_debug) DEBUG_PRINTLN();
      }
      break;
    case MODE_FOLDING:
      // don't do anything
      //if (local_debug) DEBUG_PRINTLN();
      break;
    default:
      //if (local_debug) DEBUG_PRINTLN();
      break;
  }
  if (local_debug){       
    DEBUG_INDENT(indent+1);
    DEBUG_PRINT("mode: ");
    DEBUG_PRINT(MODE_NAME[mode_mode_phase_get().mode]);
    DEBUG_PRINT(", direction: ");
    DEBUG_PRINTLN(MODE_DIR_NAME[mode_mode_phase_get().direction]);
    mode_print_parts_xyz("move_part_xyz", move_part_xyz, indent+1);
    mode_print_parts_xyz("move_part_beg_xyz", move_part_beg_xyz, indent+1);
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End mode_check_new_mode");
  }
} // end mode_check_new_mode


//========================================================
// mode_print_current_time
//========================================================
void mode_print_current_time(uint8_t indent){
  uint32_t start_time = mode_phase.start_time;
  float current_time = float(millis()-start_time)/1000.0; // get float current_time = time since the start of this phase in seconds
  com_indent(indent);
  DEBUG_PRINTF("current_time: %1.2f\n", current_time);
} // end mode_print_current_time


//========================================================
// mode_execute_seq()
// execute current mode sequence (new or previously started)
// move body and legs as required
//========================================================
void mode_execute_seq(uint8_t indent){
  uint8_t current_mode = mode_phase.mode;
  uint8_t phase = mode_phase.phase;
  uint32_t start_time = mode_phase.start_time;
  // get the current mode sequence phase and parts
  mode_seq_t mode_seq_phase = mode_seq[current_mode][phase];
  mode_seq_part_t part[2];
  part[0] = mode_seq_phase.part[0];
  part[1] = mode_seq_phase.part[1];
  uint8_t part0_id = part[0].part_id;
  uint8_t part1_id = part[1].part_id;

  static const boolean local_debug = true;
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINT("Beg mode_execute_seq, alternate: ");
    DEBUG_PRINT(mode_phase.alternate);
    DEBUG_PRINT(", mode: ");
    DEBUG_PRINT(MODE_NAME[current_mode]);
    DEBUG_PRINT(", phase: ");
    DEBUG_PRINT(mode_seq_phase.phase_name);
    DEBUG_PRINT(", part0: ");
    DEBUG_PRINT(MODE_PART_NAME[part0_id]);
    DEBUG_PRINT(", part1: ");
    DEBUG_PRINTLN(MODE_PART_NAME[part1_id]);
    mode_print_current_time(indent+1);
    mode_print_parts_xyz("move_part_xyz", move_part_xyz, indent+1);
  }
  // update parts
  float current_time = float(millis()-start_time)/1000.0; // get float current_time = time since the start of this phase in seconds
  boolean move_done = false;
  boolean zero_vel = false;
//  uint8_t part_index = 0;
  mode_execute_move(current_time, mode_seq_phase, &move_done, &zero_vel, indent+1);
  if(move_done){
    // the move for all parts has completed, move to the next phase if there is one
    if (local_debug){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTLN("move_done == true");
    }
    if(mode_seq_phase.last_phase){
      // depends on the just completed mode
      // if the mode was FOLDING, we are FOLDED
      // if the mode was UNFOLDING, wer are READY
      // if the mode was WALKING, SIDESTEPPING, or ROTATING and ends with zero velocity, we are READY
      // if the mode was WALKING, SIDESTEPPING, or ROTATING and ends with a NON-zero velocity, we are still WALKING, SIDESTEPPING, or ROTATING 
//      if(local_debug) DEBUG_PRINTLN("Beg mode_execute_seq, finished seqence");
      if (local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTLN("mode_seq_phase.last_phase == true");
      }
      switch(current_mode){
        case MODE_FOLDED:
          // stay folded, do nothing
          break;
        case MODE_FOLDING:
          mode_set(MODE_FOLDED, indent+1); // it's now folded
          break;
        case MODE_UNFOLDING:
          mode_set(MODE_READY, indent+1); // now it's ready
          break;
        case MODE_READY:
          // stay ready, do nothing
          break;
        case MODE_WALKING_BEG:
          mode_set(MODE_WALKING, indent+1); // now start walking
          break;
        case MODE_SIDESTEPPING_BEG:
          mode_set(MODE_SIDESTEPPING, indent+1);
          break;
        case MODE_ROTATING_BEG:
          mode_set(MODE_ROTATING, indent+1);
          break;
        case MODE_WALKING:
          if(zero_vel){
            mode_set(MODE_WALKING_END, indent+1);
          } else {
            mode_set(current_mode, indent+1); // continue with current mode but reset phase and start_time
          }
          break;
        case MODE_SIDESTEPPING:
          if(zero_vel){
            mode_set(MODE_SIDESTEPPING_END, indent+1);
          } else {
            mode_set(current_mode, indent+1); // continue with current mode but reset phase and start_time
          }
          break;
        case MODE_ROTATING:
          if(zero_vel){
            mode_set(MODE_ROTATING_END, indent+1);
          } else {
            mode_set(current_mode, indent+1); // continue with current mode but reset phase and start_time
          }
          break;
        case MODE_WALKING_END:
        case MODE_SIDESTEPPING_END:
        case MODE_ROTATING_END:
          mode_set(MODE_READY, indent+1);
          break;
      }
    } else {
      // not the last phase, go to the next phase, adjust start_time
//      if(local_debug) DEBUG_PRINTLN("Beg mode_execute_seq, finished phase but not seqence");
      if (local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTLN("mode_seq_phase.last_phase != true");
      }
      mode_phase.phase++;
      mode_phase.start_time = millis();
      mode_update_move_part_data(mode_phase.mode, mode_phase.phase, indent+1); // update the data for the next move seq
    }
  } else {
    if (local_debug){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINTLN("move_done != true");
    }    
  }
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End mode_execute_seq");
  }
} // end mode_execute_seq


//========================================================
// mode_print_move_part_points
//========================================================
void mode_print_move_part_points(uint8_t indent){
  String coor_name[XYZ] = {"X", "Y", "Z"};
  DEBUG_INDENT(indent);
  DEBUG_PRINTLN("Beg mode_print_move_part_points");
  DEBUG_INDENT(indent+1);
  DEBUG_PRINT("    \t\t");
  for(uint8_t point=0; point<LEGS_MOVE_POINT_NUM; point++){
    DEBUG_PRINT("\t");
    DEBUG_PRINT(LEGS_MOVE_NAME[point]);
  }
  DEBUG_PRINTLN();
  for(uint8_t part=0; part<MODE_SEQ_PART_NUM; part++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      for(uint8_t td=0; td<LEGS_MOVE_TD_NUM; td++){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINT("seq part ");
        DEBUG_PRINT(part);
        DEBUG_PRINT(", ");
        DEBUG_PRINT(coor_name[coor]);
        DEBUG_PRINT(", ");
        DEBUG_PRINT(LEGS_MOVE_TD_NAME[td]);
        DEBUG_PRINT(":");
        for(uint8_t point=0; point<LEGS_MOVE_POINT_NUM; point++){
          DEBUG_PRINT("\t");
          DEBUG_PRINTF("%7.2f", move_part_points[part][coor][point][td]);
        }
        DEBUG_PRINTLN();
      }
    }
  }
  DEBUG_INDENT(indent);
  DEBUG_PRINTLN("End mode_print_move_part_points");
}
// end mode_print_move_part_points


//========================================================
// mode_print_part_parameters
//========================================================
void mode_print_part_parameters(const char *text, float part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], uint8_t indent){
  const static char routine[] = "mode_print_part_parameters";
  DEBUG_PRINT_BEG(routine, indent);
  String coor_name[XYZ] = {"X", "Y", "Z"};
  DEBUG_INDENT(indent+1);
  DEBUG_PRINTLN(text);
  DEBUG_INDENT(indent+1);
  DEBUG_PRINT("\t\t");
  for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
    DEBUG_PRINT("\t");
    DEBUG_PRINT(LEGS_PARAM_NAME[param]);
  }
  DEBUG_PRINTLN();
  for(uint8_t part=0; part<MODE_SEQ_PART_NUM; part++){
    for(uint8_t coor=0; coor<XYZ; coor++){
      DEBUG_INDENT(indent+1);
      DEBUG_PRINT("seq part ");
      DEBUG_PRINT(part);
      DEBUG_PRINT(", ");
      DEBUG_PRINT(coor_name[coor]);
      DEBUG_PRINT(":");
      for(uint8_t param=0; param<LEGS_PARAM_NUM; param++){
        DEBUG_PRINT("\t");
        DEBUG_PRINTF("%7.2f", part_parameters[part][coor][param]);
      }
      DEBUG_PRINTLN();
    }
  }
  DEBUG_PRINT_END(routine, indent);
}
// end mode_print_part_parameters


//========================================================
// mode_print_parts_xyz
//========================================================
void mode_print_parts_xyz(const char *text, float part_xyz[MODE_PART_NUM][XYZ], uint8_t indent){
  const static char routine[] = "mode_print_parts_xyz";
  DEBUG_PRINT_BEG(routine, indent);
  DEBUG_INDENT(indent+1);
  DEBUG_PRINTLN(text);
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
    DEBUG_INDENT(indent+1);
    for(uint8_t coor=0; coor<XYZ; coor++){
      DEBUG_PRINTF("%7.2f", part_xyz[part][coor]);
      DEBUG_PRINT("\t");
    }
    DEBUG_PRINTLN();
  }
  DEBUG_PRINT_END(routine, indent);
} // end mode_print_parts_xyz


//========================================================
// mode_print_parts_activity_seq_part
//========================================================
void mode_print_parts_activity_seq_part(const char *text, uint8_t part_activity[MODE_PART_NUM], uint8_t part_seq_part[MODE_PART_NUM], uint8_t indent){
  const static char routine[] = "mode_print_parts_activity_seq_part";
  DEBUG_PRINT_BEG(routine, indent);
  DEBUG_INDENT(indent+1);
  DEBUG_PRINTLN(text);
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("%s", MODE_PART_NAME[part].c_str());
//    DEBUG_PRINTF("\t%s", MODE_PART_ACTIVITY_NAME[part_activity[part]].c_str());
//    DEBUG_PRINTF("\t%s\n", MODE_SEQ_PART_NAME[part_seq_part[part]].c_str());
//    DEBUG_PRINTF("%s\t%s\t%s\n", MODE_PART_NAME[part], MODE_PART_ACTIVITY_NAME[part_activity[part]], MODE_SEQ_PART_NAME[part_seq_part[part]]);
  }
  DEBUG_PRINT_END(routine, indent);
} // end mode_print_parts_activity_seq_part


//========================================================
// mode_update_part_move_data()
// gets old move_points data and updates move_points for part to prepare for next phase
// called when there is a mode or phase change
//========================================================
void mode_update_move_part_data(uint8_t new_mode, uint8_t new_phase, uint8_t indent){
  // use the move_seq to calculate to move parameters and move_points for the active parts in this phase of the move_sequence
  // need to create the move parameters (distance, direction, up_down, v_max, a_max, v_beg, v_end) for each part
  // mode_seq[mode][phase].part[i] contains part_id, and d[XYZ] and v[XYZ] for each of the two moving parts
  // float move_part_points[MODE_SEQ_PART_NUM][XYZ][LEGS_MOVE_POINT_NUM][LEGS_MOVE_TD_NUM]; // move points for parts
  //     LEGS_PARAM_NAME[LEGS_PARAM_NUM] = {"DIST", "DIR", "UPDN", "V_MAX", "A_MAX", "V_BEG", "V_END"};
  // need to update the move_part_beg_xyz values
  const static char routine[] = "mode_update_move_part_data";
  const boolean local_debug = true;
  if (local_debug){
    DEBUG_PRINT_BEG(routine, indent);
    DEBUG_INDENT(indent+1);
    DEBUG_PRINTF("new_mode: %s", MODE_NAME[new_mode].c_str());
    DEBUG_PRINTF(", new_phase: %u\n", new_phase);
    mode_print_current_time(indent+1);
  }

//  static const uint8_t zi = 2; // z index to make code more readable

  // create a variable to hold the next move_seq_phase
  mode_seq_t next_seq_phase = mode_seq[new_mode][new_phase]; // next_move_seq_phase for this move seq

  // create temporary versions of next move_part_parameters, move_part_beg_xyz, move_part_activity and move_part_seq_part
  float next_part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM]; // parameters for each of the part move
  float next_part_beg_xyz[MODE_PART_NUM][XYZ]; // xyz coordinates of each part at the beginning of the current move seq (each leg and the body)
  uint8_t next_part_activity[MODE_PART_NUM]; // updated between move sequences, indicates the activity and type of each part (the part type: MODE_PART_TYPE_ & {"SUPPORT_LEG", "ACTIVE_LEG", "STATIC_BODY", "ACTIVE_BODY"})
  uint8_t next_part_seq_part[MODE_PART_NUM]; // updated between move sequences, indicates which seq_part (if any) is associated with the part
  mode_create_next_part_beg_xyz(next_seq_phase, move_part_parameters, move_part_activity, move_part_seq_part, move_part_beg_xyz, next_part_beg_xyz, indent+1);
  mode_create_next_part_parameters(next_seq_phase, next_part_beg_xyz, move_part_parameters, move_part_activity, move_part_seq_part, next_part_parameters, indent+1);
  mode_create_next_part_activity_seq_part(next_seq_phase, next_part_activity, next_part_seq_part, indent+1);

  if(local_debug){
    mode_print_parts_xyz("next_part_beg_xyz", next_part_beg_xyz, indent+1);
    mode_print_part_parameters("next_part_parameters", next_part_parameters, indent+1);
    mode_print_parts_activity_seq_part("next_part_activity, next_part_seq_part", next_part_activity, next_part_seq_part, indent+1);
  }

  if (local_debug){
    mode_print_parts_xyz("move_part_xyz", move_part_xyz, indent+1);
    mode_print_parts_xyz("next_part_beg_xyz", next_part_beg_xyz, indent+1);
    mode_print_part_parameters("move_part_parameters", move_part_parameters, indent+1);
    mode_print_move_part_points(indent+1);
    DEBUG_PRINT_END(routine, indent);
  }
} // end mode_update_part_move_data


//========================================================
// mode_execute_move()
// use current time and move_points for all parts to calculate its position and move it there
// move_done will be set true when time >= last time in move, the zero_vel variable will be set to true if velocity is zero
//========================================================
void mode_execute_move(float current_time, mode_seq_t mode_seq_phase, boolean *move_done, boolean *zero_vel, uint8_t indent){
  const boolean local_debug = true;
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINT("Beg mode_execute_move, current_time: ");
    DEBUG_PRINTF("%7.2f", current_time);
    DEBUG_PRINT(", mode_seq_phase description: ");
    DEBUG_PRINTLN(mode_seq_phase.phase_name);
    mode_print_current_time(indent+1);
  }
  
//  static const uint8_t xi = 0;
//  static const uint8_t yi = 1;
  static const uint8_t zi = 2;
  boolean the_body_moves = (move_part_activity[MODE_PART_BODY] == MODE_PART_ACTIVITY_ACTIVE_BODY);
  uint8_t body_seq_part_i = move_part_seq_part[MODE_PART_BODY];

  if(local_debug && false){
    mode_print_part_parameters("move_part_parameters", move_part_parameters, indent+1);
    mode_print_move_part_points(indent+1);
  }
  if(local_debug && true){
    mode_print_parts_xyz("move_part_xyz", move_part_xyz, indent+1);
    mode_print_parts_xyz("move_part_beg_xyz", move_part_beg_xyz, indent+1);
  }

  float body_move_position[XYZ];
  float leg_move_position[XYZ];
  // first let's update the body part if there is one
  if(the_body_moves){
    // yes, there is a body in this move seq
    // this needs access to the move_points for the part, the start coordinates of the part
    legs_position(current_time, move_part_parameters[body_seq_part_i], move_part_points[body_seq_part_i], body_move_position, indent+1);
    // update the coordinates of the body
    for(uint8_t coor=0; coor<XYZ; coor++){
      move_part_xyz[MODE_PART_BODY][coor] = move_part_beg_xyz[MODE_PART_BODY][coor] + body_move_position[coor];
    }
  } else {
    // no, there is not a body in this move seq
    // don't have to update body_xyz
  }
  
  if (local_debug){
    DEBUG_INDENT(indent+1);
    DEBUG_PRINT("move_part_beg_zyx[MODE_PART_BODY]:\t");
    for(uint8_t coor=0; coor<XYZ; coor++){
      //Serial.printf("%7.2f", move_part_beg_xyz[MODE_PART_BODY][coor]);
      DEBUG_PRINTF("%7.2f", move_part_beg_xyz[MODE_PART_BODY][coor]);
      DEBUG_PRINT("\t");
    }
    DEBUG_PRINTLN();
    DEBUG_INDENT(indent+1);
    DEBUG_PRINT("body_move_position:\t\t\t");
    for(uint8_t coor=0; coor<XYZ; coor++){
      DEBUG_PRINTF("%7.2f", body_move_position[coor]);
      DEBUG_PRINT("\t");
    }
    DEBUG_PRINTLN();
    DEBUG_INDENT(indent+1);
    DEBUG_PRINT("move_part_xyz[MODE_PART_BODY]:\t");
    for(uint8_t coor=0; coor<XYZ; coor++){
      DEBUG_PRINTF("%7.2f", move_part_xyz[MODE_PART_BODY][coor]);
      DEBUG_PRINT("\t");
    }
    DEBUG_PRINTLN();
  }

  // now let's update the legs in the list of all possible parts
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
//    DEBUG_PRINT("part: ");
//    DEBUG_PRINT(part);
//    DEBUG_PRINT(", activity type: ");
//    DEBUG_PRINT(move_part_activity[part]);
//    DEBUG_PRINT(", activity name: ");
//    DEBUG_PRINTLN(MODE_PART_ACTIVITY_TYPE_NAME[move_part_activity[part]]);
    switch(move_part_activity[part]){
      case MODE_PART_ACTIVITY_ACTIVE_LEG:
        // if the part is an ACTIVE_LEG, update the coordinates
        legs_position(current_time, move_part_parameters[move_part_seq_part[part]], move_part_points[move_part_seq_part[part]], leg_move_position, indent+1);
        for(uint8_t coor=0; coor<XYZ; coor++){
          if(coor == zi){
            move_part_xyz[part][coor] = move_part_beg_xyz[part][coor] + leg_move_position[coor] - body_move_position[zi];
          } else {
            move_part_xyz[part][coor] = move_part_beg_xyz[part][coor] + leg_move_position[coor];
          }
        }
        break;
      case MODE_PART_ACTIVITY_SUPPORT_LEG:
        // if the part is a SUPPORT_LEG, update the coordinates
        for(uint8_t coor=0; coor<XYZ; coor++){
          move_part_xyz[part][coor] = move_part_beg_xyz[part][coor] - body_move_position[coor];
          //com_free_mem();
//          DEBUG_PRINT("move_part_beg_xyz[part][coor] (");
//          DEBUG_PRINTF("%7.2f", move_part_beg_xyz[part][coor]);
//          DEBUG_PRINT(") - body_move_position[coor] (");
//          DEBUG_PRINTF("%7.2f", body_move_position[coor]);
//          DEBUG_PRINT(") = move_part_xyz[part][coor] (");
//          DEBUG_PRINTF("%7.2f)\n", move_part_xyz[part][coor]);
        }
        break;
      case MODE_PART_ACTIVITY_STATIC_BODY:
      case MODE_PART_ACTIVITY_ACTIVE_BODY:
        // don't do anything, we've already handled the body
        break;
    }
//    mode_print_parts_xyz("move_part_xyz", move_part_xyz, indent+1);
  }

  float angle_phk[NUM_LEGS][NUM_JOINTS_LEG];
  legs_angles(move_part_xyz, angle_phk);
  servo_set_angles(angle_phk);
  *move_done = (current_time >= move_part_points[0][0][0][0]) && (current_time >= move_part_points[1][0][0][0]);

  *zero_vel = true; // place holder!!!
  if (local_debug){
    mode_print_parts_xyz("move_part_xyz", move_part_xyz, indent+1);
    //mode_print_move_part_points(indent+1);
    //mode_print_parts_beg_xyz(indent+1);
    DEBUG_INDENT(indent+1);
    if(current_time >= move_part_points[0][0][0][0]){
      DEBUG_PRINTF("current_time: %7.2f >= move_part_points[0][0][0][0]: %7.2f", current_time, move_part_points[0][0][0][0]);
    } else {
      DEBUG_PRINTF("current_time: %7.2f < move_part_points[0][0][0][0]: %7.2f", current_time, move_part_points[0][0][0][0]);      
    }
    if(current_time >= move_part_points[1][0][0][0]){
      DEBUG_PRINTF(", current_time: %7.2f >= move_part_points[1][0][0][0]: %7.2f", current_time, move_part_points[1][0][0][0]);
    } else {
      DEBUG_PRINTF(", current_time: %7.2f < move_part_points[1][0][0][0]: %7.2f", current_time, move_part_points[1][0][0][0]);
    }
    DEBUG_PRINT(", move_done: ");
    DEBUG_PRINTLN(*move_done);
    
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End mode_execute_move");
  }
}
// end mode_execute_move


////========================================================
//// mode_update_part_activity_seq_part
//// update the move_part_activity[MODE_PART_NUM] and move_part_seq_part[MODE_PART_NUM]
////========================================================
//void mode_update_part_activity_seq_part(mode_seq_t next_move_seq_phase, uint8_t indent){
//  const boolean local_debug = true;
//  if (local_debug){
//    DEBUG_INDENT(indent);
//    DEBUG_PRINTLN("Beg mode_update_part_activity_seq_part");
//    mode_print_current_time(indent+1);
//  }
//  // set all parts to their activity type for this new phase, first assume they're not active
//  for(uint8_t part_i=0; part_i<MODE_PART_NUM; part_i++){
//    // all parts are either legs or body
//    if(part_i == MODE_PART_BODY){
//      // this is the body, assume it's not active
//      move_part_activity[part_i] = MODE_PART_ACTIVITY_STATIC_BODY;
//    } else {
//      // this is a leg, assume it's not active
//      move_part_activity[part_i] = MODE_PART_ACTIVITY_SUPPORT_LEG;
//    }
//    move_part_seq_part[part_i] = MODE_SEQ_NONE;
//  }
//  // now see if any ARE active
//  for(uint8_t seq_part_i=0; seq_part_i<MODE_SEQ_PART_NUM; seq_part_i++){
//    uint8_t part_i = next_move_seq_phase.part[seq_part_i].part_id;
//    switch (part_i){
//      case MODE_PART_BODY:
//        move_part_activity[part_i] = MODE_PART_ACTIVITY_ACTIVE_BODY; // the body is one of the move_seq_parts, set the body part to active
//        move_part_seq_part[part_i] = seq_part_i; // the body is one of the move_seq_parts, set the body's move_part_seq_part to the move_phase_seq_part_i
//        break;
//      case MODE_PART_LEG0:
//      case MODE_PART_LEG1:
//      case MODE_PART_LEG2:
//      case MODE_PART_LEG3:
//        move_part_activity[part_i] = MODE_PART_ACTIVITY_ACTIVE_LEG; // this leg is one of the move_seq_parts, set the leg part to active
//        move_part_seq_part[part_i] = seq_part_i; // this leg is one of the move_seq_parts, set the leg's move_part_seq_part to the move_phase_seq_part_i
//        break;
//      case MODE_PART_NONE:
//        // do nothing
//        break;
//   }
//  }
//  
//  // for each part show the activity type and which seq_part it is (if any)
//  if(local_debug){
//    for(uint8_t part_i=0; part_i<MODE_PART_NUM; part_i++){
//      DEBUG_INDENT(indent+1);
//      DEBUG_PRINT(MODE_PART_NAME[part_i]);
//      DEBUG_PRINT(" new activity_type: ");
//      DEBUG_PRINT(MODE_PART_ACTIVITY_TYPE_NAME[move_part_activity[part_i]]);
//      DEBUG_PRINT(", seq_part_i: ");
//      DEBUG_PRINTLN(MODE_SEQ_PART_NAME[move_part_seq_part[part_i]]);
//    }
//  }  
//  if (local_debug){
//    DEBUG_INDENT(indent);
//    DEBUG_PRINTLN("End mode_update_part_activity_seq_part");
//  }
//} // end mode_update_part_activity_seq_part
//

//========================================================
//  mode_create_next_part_parameters
//    create the next move_part_parameters for each new active seq_part
//    note: the leg move_parameters are relative to the body center exept for z which will be an amout of up/down motion
//    while the body move parameters are relative to the beginning body center at ground level
//========================================================
void mode_create_next_part_parameters(mode_seq_t next_seq_phase, float next_part_beg_xyz[MODE_PART_NUM][XYZ], float part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], uint8_t part_activity_type[MODE_PART_NUM], uint8_t part_seq_part[MODE_PART_NUM], float next_part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], uint8_t indent){
  const static char routine[] = "mode_create_next_part_parameters";
  const boolean local_debug = true;
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);

  static const uint8_t zi = 2; // z index to make code more readable

  for(uint8_t next_seq_part=0; next_seq_part<MODE_SEQ_PART_NUM; next_seq_part++){
    // check each of the seq_parts, don't bother with this part if it's not moving
    uint8_t next_part = next_seq_phase.part[next_seq_part].part_id; // this is the actual part index for this seq_part
    if(next_part != MODE_PART_NONE){
      // an actual part is specified in the move sequence
      if(local_debug){
        DEBUG_INDENT(indent+1);
        DEBUG_PRINTF("next_seq_part: %u", next_seq_part);
        DEBUG_PRINTF(",  part: %s\n", MODE_PART_NAME[next_part].c_str());
      }
      for(uint8_t coor=0; coor<XYZ; coor++){
        // for each coordinate store the move parameters for this part
        if(local_debug){
          DEBUG_INDENT(indent+1);
          DEBUG_PRINTF("coor: %u", coor);
          DEBUG_PRINTF(", next_seq_phase.part[next_seq_part].d[coor]: %7.2f\n", next_seq_phase.part[next_seq_part].d[coor]); // amount to move this part, either absolute or relative
//          DEBUG_PRINTF(", move_part_xyz[next_part][coor]: %7.2f\n", move_part_xyz[next_part][coor]); // current coordinate of part
        }
        if(next_part != MODE_PART_BODY){
          // this moving part isn't the body so it must be a leg
          // active leg positions x, y are relative to the body position at the start of the mode, but z is just used for the leg lift amount
          if(coor == zi){
            // moving part is a leg and Z coor
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_DIST] = next_seq_phase.part[next_seq_part].d[coor]; // this is just the lift amount for z
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_UPDN] = 1.0; // set to > 0.0 to signify that this is an up/down move for z
          } else {
            // moving part is a leg and X or Y coor
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_DIST] = next_seq_phase.part[next_seq_part].d[coor] - next_part_beg_xyz[next_part][coor]; // convert the absolute position of d[XY] to a distance = absolute - current
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_UPDN] = -1.0; // set to < 0.0 to signify that this is NOT an up/down move for z
          }
        } else {
          // else (the part is a body)
          // body positions are relative to the body position at the start of the move for x and y, but z is absolute
          if(coor == zi){
            // coor is z so this is absolute position, dist = next position - current position
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_DIST] = next_seq_phase.part[next_seq_part].d[coor] - next_part_beg_xyz[next_part][coor]; // convert the absolute position of d[XY] to a distance = absolute - current
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_UPDN] = -1.0; // set to < 0.0 to signify that this is NOT an up/down move for z
          } else {
            // coor is x or y so this is the distance to move
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_DIST] = next_seq_phase.part[next_seq_part].d[coor]; // this is just the amount to move
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_UPDN] = -1.0; // set to < 0.0 to signify that this is NOT an up/down move
          }
//          if(local_debug){
//            DEBUG_INDENT(indent+1);
//            DEBUG_PRINTF("next_part_parameters[next_part][coor]: %7.2f\n", move_part_xyz[next_part][coor]); // the current coordinate of the part
//            DEBUG_INDENT(indent+1);
//            DEBUG_PRINTF("next_part_parameters[next_seq_part][coor][LEGS_PARAM_DIST]: %7.2f\n", next_part_parameters[next_seq_part][coor][LEGS_PARAM_DIST]); // verify that we've set the DIST correctly
//          }
        }
        // now for each part, coor and parameter set the direction
        next_part_parameters[next_seq_part][coor][LEGS_PARAM_DIR] = 1.0; // will be changed by legs_coor_move_points() if distance is negative
        
        // now for each part, coor and parameter set the v_max and a_max
        next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_MAX] = LEGS_XYZ_VA_MAX[coor][0]; // initialize with the v_max value from LEGS_XYZ_VA_MAX
        next_part_parameters[next_seq_part][coor][LEGS_PARAM_A_MAX] = LEGS_XYZ_VA_MAX[coor][1]; // initialize with the a_max value from LEGS_XYZ_VA_MAX

        // now set the beginning velocities for the next move for each ACTIVE part
        // if the body will be MODE_PART_ACTIVITY_ACTIVE_BODY and it was MODE_PART_ACTIVITY_ACTIVE_BODY: v_beg velocity is the body's previous v_end velocity
        // if the body will be MODE_PART_ACTIVITY_ACTIVE_BODY and it was MODE_PART_ACTIVITY_STATIC_BODY: v_beg velocity is 0.0
        // if a leg will be MODE_PART_ACTIVITY_ACTIVE_LEG and it was MODE_PART_ACTIVITY_SUPPORT_LEG: v_beg = - body's previous v_end velocity
        // if a leg will be MODE_PART_ACTIVITY_ACTIVE_LEG and it was MODE_PART_ACTIVITY_ACTIVE_LEG: v_beg = the feet's previous v_end velocity
        if(next_part == MODE_PART_BODY){
          // the body will be MODE_PART_ACTIVITY_ACTIVE_BODY
          if(part_activity_type[MODE_PART_BODY] == MODE_PART_ACTIVITY_ACTIVE_BODY){
            // and it was MODE_PART_ACTIVITY_ACTIVE_BODY: v_beg velocity is the body's previous v_end velocity
            uint8_t seq_part_body = part_seq_part[MODE_PART_BODY]; // this is the seq_part the body had in the previous move
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_BEG] = part_parameters[seq_part_body][coor][LEGS_PARAM_V_END]; 
          } else {
            // and it was MODE_PART_ACTIVITY_STATIC_BODY: v_beg velocity is 0.0
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_BEG] = 0.0; 
          }
        } else {
          // it's not the body, it must be a leg, and it will be active in the next move
          // the leg will be MODE_PART_ACTIVITY_ACTIVE_LEG
          if(part_activity_type[next_part] == MODE_PART_ACTIVITY_SUPPORT_LEG){
            // and it was previously MODE_PART_ACTIVITY_SUPPORT_LEG
            if(part_activity_type[MODE_PART_BODY] == MODE_PART_ACTIVITY_ACTIVE_BODY){
              // the body moved previously, v_beg = - body's previous v_end velocity (if it had any v_end velocity)
              uint8_t seq_part_body = part_seq_part[MODE_PART_BODY]; // this is the seq_part the body had in the previous move
              next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_BEG] = - part_parameters[seq_part_body][coor][LEGS_PARAM_V_END]; 
            } else {
              // the body wasn't moving before so the support leg's v_beg = 0.0
              next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_BEG] = 0.0; 
            }
          } else {
            // it was previously MODE_PART_ACTIVITY_ACTIVE_LEG: v_beg = the feet's previous v_end velocity
            uint8_t seq_part_leg = part_seq_part[next_part]; // this is the seq_part this leg had in the previous move
            next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_BEG] = part_parameters[seq_part_leg][coor][LEGS_PARAM_V_END]; 
          }
        }

        // now write the ending velocities for the next active parts
        //DEBUG_PRINTF("about to write V_END of :%7.2f\n", next_move_seq_phase.part[next_seq_part].v[coor]);
        next_part_parameters[next_seq_part][coor][LEGS_PARAM_V_END] = next_seq_phase.part[next_seq_part].v[coor]; // initialize with the move_seq values
      }
      
    } else {
      // part_i == MODE_PART_NONE
      // do nothing
    }
  }
  
  if (local_debug) DEBUG_PRINT_END(routine, indent);
} // end mode_create_next_part_parameters


//========================================================
//  mode_create_next_part_activity_seq_part
//========================================================
void mode_create_next_part_activity_seq_part(mode_seq_t next_seq_phase, uint8_t next_part_activity[MODE_PART_NUM], uint8_t next_part_seq_part[MODE_PART_NUM], uint8_t indent){
  const static char routine[] = "mode_create_next_part_activity_seq_part";
  const boolean local_debug = true;
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);

  // set all part activity to inactive initially
  // set all part seq part to none
  for(uint8_t part=0; part<MODE_PART_NUM; part++){
    if(part == MODE_PART_BODY){
      next_part_activity[part] = MODE_PART_ACTIVITY_STATIC_BODY;
    } else {
      next_part_activity[part] = MODE_PART_ACTIVITY_SUPPORT_LEG;      
    }
    next_part_seq_part[part] = MODE_SEQ_NONE;
  }

  // now see the active ones to active
  for(uint8_t seq_part=0; seq_part<MODE_SEQ_PART_NUM; seq_part++){
    uint8_t part = next_seq_phase.part[seq_part].part_id;
    if(part == MODE_PART_NONE){
      // do nothing
    } else if (part == MODE_PART_BODY){
      next_part_activity[part] = MODE_PART_ACTIVITY_ACTIVE_BODY;
      next_part_seq_part[part] = seq_part;
    } else {
      next_part_activity[part] = MODE_PART_ACTIVITY_ACTIVE_LEG;
      next_part_seq_part[part] = seq_part;
    }
  }
  
  if (local_debug) DEBUG_PRINT_END(routine, indent);
} // end mode_create_next_part_activity_seq_part


//========================================================
//  mode_create_next_part_beg_xyz
//========================================================
void mode_create_next_part_beg_xyz(mode_seq_t next_seq_phase, float part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], uint8_t part_activity_type[MODE_PART_NUM], uint8_t part_seq_part[MODE_PART_NUM], float part_beg_xyz[MODE_PART_NUM][XYZ], float next_part_beg_xyz[MODE_PART_NUM][XYZ], uint8_t indent){
  // create next_move_part_beg_xyz[MODE_PART_NUM][XYZ] with where they should be at at the end of the move that just finished
  // if the body moved, that will impact the body and all SUPPORT legs (they moved realtive to the body because they are at fixed positions on the ground)
  // if any legs were ACTIVE, just use the info from the previous move_points for the ACTIVE legs as that motion is relative to the body
  // what about first time, when there is no previous move? We will initialize the move_points to the correct locations in mode_setup
  // Potential drift:
  // - Body: the body may accumulate some potential drift due to accumulating error from adding incremental changes, for XY this is not critical at this time
  //         for Z this could become an issue but due to the fact that Z is relative to ground level and that ACTIVE legs will be set to absolute positions relative to the body,
  //         this is not expected to be an issue
  // - Legs: the Support legs may accumulate error due to the body error described above but once a leg moves, is ACTIVE, then is is set to an absolute position relative to the body

  //  uses the following passed in globals:
  //    part_activity_type[MODE_PART_NUM]    to see if body or leg was moving
  //    part_seq_part[MODE_PART_NUM]         to find the index of the previously moving body part
  //    move_part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM]               to get DIST & DIR for the distance moved
  
  const static char routine[] = "mode_create_next_part_beg_xyz";
  const boolean local_debug = true;
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);
  
  // first, let's see if the body moved, that will impact whether we do anything for the SUPPORT legs
  if(part_activity_type[MODE_PART_BODY] == MODE_PART_ACTIVITY_ACTIVE_BODY){
    // the body moved in the last move_seq, update the ACTIVE leg(s), the body and all SUPPORT legs
    uint8_t seq_part_body = part_seq_part[MODE_PART_BODY]; // since we know the body moved, this may be MODE_SEQ_PART0 or MODE_SEQ_PART1
    float body_displacement[XYZ];
    for(uint8_t coor=0; coor<XYZ; coor++){ // for each coor
      body_displacement[coor] = part_parameters[seq_part_body][coor][LEGS_PARAM_DIST] * part_parameters[seq_part_body][coor][LEGS_PARAM_DIR];
    }
    for(uint8_t part=0; part<MODE_PART_NUM; part++){ // for every part
      for(uint8_t coor=0; coor<XYZ; coor++){ // for each coor
        switch(part_activity_type[part]){
          case MODE_PART_ACTIVITY_ACTIVE_BODY:
            // update the body begin_XYZ by ADDING the amount of the last body move to each coor
            next_part_beg_xyz[part][coor] = part_beg_xyz[part][coor] + body_displacement[coor];
            break;
          case MODE_PART_ACTIVITY_SUPPORT_LEG:
            // update the SUPPORT leg(s) begin_XYZ by SUBTRACTING the amount of the last body move
            next_part_beg_xyz[part][coor] = part_beg_xyz[part][coor] - body_displacement[coor];
            break;
          case MODE_PART_ACTIVITY_ACTIVE_LEG:{
              // update the ACTIVE leg(s) begin_XYZ by ADDING the amount of this leg's last move
              uint8_t seq_part_leg = part_seq_part[part];
              float leg_displacement =  part_parameters[seq_part_leg][coor][LEGS_PARAM_DIST] * part_parameters[seq_part_leg][coor][LEGS_PARAM_DIR];
              next_part_beg_xyz[part][coor] = part_beg_xyz[part][coor] + leg_displacement;
            }
            break;
          default:
            const static char err_msg[] = "shouldn't see this STATIC_BODY here";
            com_err_msg_int(routine, err_msg);
            break;
        }
      }
    }
  } else {
    // the body didn't move in the previous move_seq, update any ACTIVE leg(s)
    for(uint8_t part=0; part<MODE_PART_NUM; part++){
      for(uint8_t coor=0; coor<XYZ; coor++){
        switch(part_activity_type[part]){
          case MODE_PART_ACTIVITY_ACTIVE_LEG:{
              // update the ACTIVE leg(s) begin_XYZ by ADDING the amount of this leg's last move
              uint8_t seq_part_leg = part_seq_part[part];
              float leg_displacement =  part_parameters[seq_part_leg][coor][LEGS_PARAM_DIST] * part_parameters[seq_part_leg][coor][LEGS_PARAM_DIR];
              next_part_beg_xyz[part][coor] = part_beg_xyz[part][coor] + leg_displacement;
            }
            break;
          case MODE_PART_ACTIVITY_STATIC_BODY:
          case MODE_PART_ACTIVITY_SUPPORT_LEG:
            // don't need to update the body or SUPPORT leg(s)
            break;
          default:{
              const static char err_msg[] = "shouldn't see this ACTIVE_BODY here";
              com_err_msg_int(routine, err_msg);
            }
            break;
        }
      }
    }
  }
  if (local_debug) DEBUG_PRINT_END(routine, indent);
} // end mode_create_next_part_beg_xyz


//========================================================
// mode_set_folded
//========================================================
void mode_set_folded(uint8_t indent){
  mode_set(MODE_FOLDED, indent+1);
} // end mode_set_folded


//========================================================
// mode_set_ready
//========================================================
void mode_set_ready(uint8_t indent){
  mode_set(MODE_READY, indent+1);
} // end mode_set_ready


//========================================================
// mode_set_dir
//========================================================
void mode_set_dir(uint8_t newdir, uint8_t indent){
  mode_phase.direction = newdir;
} // end mode_set_dir


//========================================================
// mode_set
// change to new mode, reset phase and start_time
// for each part in last sequence get v_end
// lookup dist and v_end for each part in new sequence
// update mode_move_part_points for new sequence
//========================================================
void mode_set(uint8_t new_mode, uint8_t indent){
  const boolean local_debug = true;
  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINT("Beg mode_set, new_mode: ");
    DEBUG_PRINTLN(MODE_NAME[new_mode]);
    mode_print_current_time(indent+1);
  }

  // update the start_time
  mode_phase.start_time = millis();
  
  // update mode_phase.alternate
  if(mode_phase.alternate == 0){
    mode_phase.alternate = 1;
  } else {
    mode_phase.alternate = 0;
  }
  
  uint8_t part_index, coor;
  
  //uint8_t part_index, coor, seq_part_index;
  // get old sequence data
  uint8_t old_mode = mode_phase.mode;
  uint8_t old_phase = mode_phase.phase; 
  mode_seq_t old_mode_seq = mode_seq[old_mode][old_phase];
  
  // get new sequence data
  mode_phase.mode = new_mode;
  uint8_t new_phase = 0;
  mode_phase.phase = new_phase;
  mode_seq_t new_mode_seq = mode_seq[new_mode][new_phase];
  
  // get old sequence v_end data
  // first initialize all parts, all coor v_end to 0.0
  float old_part_v_end[MODE_PART_NUM][XYZ];
  for(part_index =0; part_index<MODE_PART_NUM; part_index++){
    for(coor =0; coor<XYZ; coor++){
      old_part_v_end[part_index][coor] = 0.0;
    }
  }
  
  // now set move sequence parts v_end to v_end from old sequence
  for(uint8_t part_i=0; part_i<MODE_SEQ_PART_NUM; part_i++){
    part_index = old_mode_seq.part[part_i].part_id;
    for(uint8_t coor =0; coor<XYZ; coor++){
      old_part_v_end[part_index][coor] = old_mode_seq.part[part_i].v[coor];
    }
  }
  if(local_debug && false){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("  old_part_v_end");
    for(part_index =0; part_index<MODE_PART_NUM; part_index++){
      DEBUG_INDENT(indent);
      for(uint8_t coor =0; coor<XYZ; coor++){
        DEBUG_PRINTF("\t%7.2f", old_part_v_end[part_index][coor]);
      }
      DEBUG_PRINTLN();
    }
  }
  
  mode_update_move_part_data(new_mode, new_phase, indent+1);

  if (local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End mode_set");
  }
} // end mode_set


//========================================================
// mode_mode_phase_get
//========================================================
mode_phase_t mode_mode_phase_get(void){
  return mode_phase;
} // end mode_mode_phase_get


//========================================================
// mode_mode_get
//========================================================
uint8_t mode__mode_get(void){
//  return mode_phase[0];
  return mode_phase.mode;
} // end mode_mode_get


//========================================================
// mode_phase_get
//========================================================
uint8_t mode_phase_get(void){
  return mode_phase.phase;
} // end mode_phase_get


//========================================================
// mode_value_fold()
//========================================================
boolean mode_value_fold(void){
  return mode_values.fold;
} // end mode_value_fold


//========================================================
// mode_value_vy()
//========================================================
float mode_value_vy(void){
  return mode_values.vy;
} // end mode_value_vy


//========================================================
// mode_value_vx()
//========================================================
float mode_value_vx(void){
  return mode_values.vx;
} // end mode_value_vx


//========================================================
// mode_value_vt()
//========================================================
float mode_value_vt(void){
  return mode_values.vt;
} // end mode_value_throttle


//========================================================
// mode_value_height()
//========================================================
float mode_value_height(void){
  return mode_values.height;
} // end mode_value_height


//========================================================
// mode_value_angle()
//========================================================
float mode_value_angle(void){
  return mode_values.angle;
} // end mode_value_angle


//========================================================
// mode_values_update
// gets the sbus values and converts them to mode values
// based on center, dead-zone and gain
// typical sbus values range from 
//========================================================
void mode_values_update(void){
  const boolean local_debug = false;
  if (local_debug){
//    DEBUG_PRINT("Beg mode_values_update; DX8: F, VY, VX, VT, H, A: ");
    DEBUG_PRINT("Beg mode_values_update; DXe: F, VY, VX, VT, H, A: ");
  }
  int16_t temp; // will have +/- values

  const int16_t GEAR_CENTER = 1024; // should be the center reading of the gear switch
  if(sbus_channel(SBUS_GEAR) < GEAR_CENTER) {
    mode_values.fold = true;
  } else {
    mode_values.fold = false;
  }

  const int16_t THROTTLE_CENTER = 1024; // should be the center reading of the throttle
  const int16_t THROTTLE_DEAD_ZONE = 16; // amount we'll allow the reading to go above or below center while keeping 0.0 value
  const float VY_POS_GAIN = 0.01; // move forward at 0.01 mm/sec for 1 usec pulse, minus sign reverses direction
  const float VY_NEG_GAIN = 0.01; // move backward at 0.01 mm/sec for 1 usec pulse, minus sign reverses direction
  temp = sbus_channel(SBUS_THROTTLE) - THROTTLE_CENTER;
  if(temp > THROTTLE_DEAD_ZONE){
    mode_values.vy = VY_POS_GAIN * float(temp - THROTTLE_DEAD_ZONE);
  } else if(temp < -THROTTLE_DEAD_ZONE) {
    mode_values.vy = VY_NEG_GAIN * float(temp + THROTTLE_DEAD_ZONE);
  } else {
    mode_values.vy = 0.0;
  }
  const int16_t RUDDER_CENTER = 1024; // should be the center reading of the throttle
  const int16_t RUDDER_DEAD_ZONE = 16; // amount we'll allow the reading to go above or below center while keeping 0.0 value
  const float VX_POS_GAIN = -0.01; // move right at 0.01 mm/sec for 1 usec pulse, minus sign reverses direction
  const float VX_NEG_GAIN = -0.01; // move left at 0.01 mm/sec for 1 usec pulse, minus sign reverses direction
  temp = sbus_channel(SBUS_RUDDER) - RUDDER_CENTER;
  if(temp > RUDDER_DEAD_ZONE){
    mode_values.vx = VX_POS_GAIN * float(temp - RUDDER_DEAD_ZONE);
  } else if(temp < -RUDDER_DEAD_ZONE) {
    mode_values.vx = VX_NEG_GAIN * float(temp + RUDDER_DEAD_ZONE);
  } else {
    mode_values.vx = 0.0;
  }
  const int16_t AILERON_CENTER = 1024; // should be the center reading of the throttle
  const int16_t AILERON_DEAD_ZONE = 16; // amount we'll allow the reading to go above or below center while keeping 0.0 value
  const float VT_POS_GAIN = 0.01; // counter clockwise 0.01 radian/sec for 1 usec pulse, minus sign reverses direction 
  const float VT_NEG_GAIN = 0.01; // clockwise 0.01 radian/sec for 1 usec pulse, minus sign reverses direction
  temp = sbus_channel(SBUS_AILERON) - AILERON_CENTER;
  if(temp > AILERON_DEAD_ZONE){
    mode_values.vt = VT_POS_GAIN * float(temp - AILERON_DEAD_ZONE);
  } else if(temp < -AILERON_DEAD_ZONE) {
    mode_values.vt = VT_NEG_GAIN * float(temp + AILERON_DEAD_ZONE);
  } else {
    mode_values.vt = 0.0;
  }
  const int16_t AUX3_CENTER = 1024; // should be the center reading of the throttle
  const int16_t AUX3_DEAD_ZONE = 32; // amount we'll allow the reading to go above or below center while keeping 0.0 value
  const float HEIGHT_POS_GAIN = -0.01; // plus 0.01 mm for 1 usec pulse, minus sign reverses direction
  const float HEIGHT_NEG_GAIN = -0.01; // minus 0.01 mm for 1 usec pulse, minus sign reverses direction
  temp = sbus_channel(SBUS_AUX3) - AUX3_CENTER;
  if(temp > AUX3_DEAD_ZONE){
    mode_values.height = HEIGHT_POS_GAIN * float(temp - AUX3_DEAD_ZONE);
  } else if(temp < -AUX3_DEAD_ZONE) {
    mode_values.height = HEIGHT_NEG_GAIN * float(temp + AUX3_DEAD_ZONE);
  } else {
    mode_values.height = 0.0;
  }
  const int16_t ELEVATOR_CENTER = 1024; // should be the center reading of the throttle
  const int16_t ELEVATOR_DEAD_ZONE = 16; // amount we'll allow the reading to go above or below center while keeping 0.0 value
  const float ANGLE_POS_GAIN = -0.01; // plus 0.01 radian for 1 usec pulse, minus sign reverses direction
  const float ANGLE_NEG_GAIN = -0.01; // minus 0.01 radian for 1 usec pulse, minus sign reverses direction
  temp = sbus_channel(SBUS_ELEVATOR) - ELEVATOR_CENTER;
  if(temp > ELEVATOR_DEAD_ZONE){
    mode_values.angle = ANGLE_POS_GAIN * float(temp - ELEVATOR_DEAD_ZONE);
  } else if(temp < -ELEVATOR_DEAD_ZONE) {
    mode_values.angle = ANGLE_NEG_GAIN * float(temp + ELEVATOR_DEAD_ZONE);
  } else {
    mode_values.angle = 0.0;
  }

  if(local_debug){
    DEBUG_PRINT("\t");
    DEBUG_PRINT(mode_values.fold);
    DEBUG_PRINT("\t");
    DEBUG_PRINTF("%7.2f", mode_values.vy);
    DEBUG_PRINT("\t");
    DEBUG_PRINTF("%7.2f", mode_values.vx);
    DEBUG_PRINT("\t");
    DEBUG_PRINTF("%7.2f", mode_values.vt);
    DEBUG_PRINT("\t");
    DEBUG_PRINTF("%7.2f", mode_values.height);
    DEBUG_PRINT("\t");
    DEBUG_PRINTF("%7.2f\n", mode_values.angle);
  }
  
  return;
} // end mode_values_update
