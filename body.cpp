////====================================================
//// body.cpp
//// functions for updating walker body
////====================================================
//
//#include "Arduino.h"
//#include "body.h"
//#include "com.h"
//#include "mode.h"
//#include "legs.h"
//
//static float body_xyz[XYZ]; // position in mm relative to start 
//static float body_v_xyz[XYZ]; // velocity in mm/sec 
//static float body_Axyz[XYZ]; // orientation in radians
//static float body_v_Axyz[XYZ]; // angular velocity in radians/sec
////static uint8_t body_seq;
////static uint8_t body_seq_leg; // for multi leg sequences
//static unsigned long body_seq_timer;
//
////const static uint8_t BODY_SEQ_NONE = 0;
////const static uint8_t BODY_SEQ_WALK_2 = 1;
////const static uint8_t BODY_SEQ_SIDESTEP_2 = 2;
////const static uint8_t BODY_SEQ_RETRACT_4 = 3;
////const static uint8_t BODY_SEQ_LOWER = 4;
////const static uint8_t BODY_SEQ_RAISE = 5;
////const static uint8_t BODY_SEQ_EXTEND_4 = 6;
////const static uint8_t BODY_SEQ_NUM = 7;
////const String BODY_SEQ_NAME[BODY_SEQ_NUM] = {"NONE", "WALK_2", "SIDE_2", "RETRACT", "LOWER", "RAISE", "EXTEND"};
////
//
//// steps sequences may be fixed or variable based on velocity
//// leg_step_seq = {xyz_delta[leg], time} (fixed, for raise, lower)
//// legs_parameters[leg] = {parameters[coor][param_num], parameters[coor][param_num], parameters[coor][param_num], parameters[coor][param_num]};
//// leg_step_seq = {legs_parameters, legs_parameters} (variable; dist, v_max, a_max, v_beg, v_end all to be scaled based on velocity for walk_2, sidestep_2)
//// leg_step_seq = {legs_parameters, legs_parameters, legs_parmeters, legs_parameters} (4 steps fixed, for fold, unfold)
//
//// data to describe retracting 1 leg to be used, with modification, by all 4 legs
//// contains move parameters and move_points: parameters[coor][LEGS_PARAM_NUM], dir[coor], move_points[coor][point][td]
//// parameters[coor][LEGS_PARAM_NUM] include for each coor: {"dist", "v_max", "a_max", "v_beg", "v_end"}
//// dir[coor] is the int8_t direction (+1 or -1) for each coordinate
//// move_points[coor][point][td] includes for each coordinate 3 or 7 points (depending if unidirectional move or back and forth move) with time and distance
//
//const static com_move_param_t BODY_SEQ_RETRACT[3] = {
//    {0.0, 0.0, 0.0, 0.0, 0.0}, // x {"dist", "v_max", "a_max", "v_beg", "v_end"}
//    {0.0, 0.0, 0.0, 0.0, 0.0}, // y {"dist", "v_max", "a_max", "v_beg", "v_end"}
//    {0.0, 0.0, 0.0, 0.0, 0.0}  // z {"dist", "v_max", "a_max", "v_beg", "v_end"}
//  }; // place holder!!!
//
//
////====================================================
//// setup body
////====================================================
//void body_setup(){
//  for(uint8_t i=0; i<XYZ; i++){
//    body_xyz[i] = 0.0; // set initial body_xyz to 0, 0, 0
//    body_v_xyz[i] = 0.0; // set initial body_v_xyz to 0, 0, 0
//    body_Axyz[i] = 0.0; // set initial body_Axyz to 0, 0, 0
//    body_v_Axyz[i] = 0.0; // set initial body_v_Axyz to 0, 0, 0
////    body_seq = BODY_SEQ_NONE;
//  }
//}
//
//
////====================================================
//// body update
////====================================================
//void body_update(){
//  const boolean local_debug = true;
//  mode_phase_t mode_phase = mode_mode_phase_get();
//  if(local_debug){
//    DEBUG_PRINT("in body_update, mode_seq: ");
//    DEBUG_PRINTLN(MODE_SEQ_NAME[mode_phase.mode]);
//  }
//  // check to see if we are in a new mode
//  
//  switch (mode_phase.mode){
//    case MODE_WALKING:
//      // see if done
//      if(millis() >= (body_seq_timer+1000)){
//        // pretend step is done
//        if(mode_value_vy() == 0.0){
////          mode_phase.mode = MODE_READY;
//          mode_set_ready();
//        } else {
//          body_seq_timer = millis();
//        }
//      }
//      break;
//    case MODE_SIDESTEPPING:
//      // see if done
//      break;
//    case MODE_FOLDING:
//      // see if done
//      if(millis() >= (body_seq_timer+1000)){
//        // pretend it's done
////        body_seq = BODY_SEQ_NONE;
//        mode = FOLDED;
//        mode_set_folded();
//      }
//      break;
//    case MODE_UNFOLDING:
//      // update unfolding
//      // use phase to 
//      // see if done
//      if(millis() >= (body_seq_timer+1000)){
//        // pretend it's done
///        mode = MODE_READY;
//        mode_set_ready();
//      }
//      break;
//  }
//} // end body_update
//
//
////====================================================
//// body start folding
////====================================================
//void body_start_folding(void){
//  const boolean local_debug = true;
//  if(local_debug) DEBUG_PRINTLN("in body_start_folding");
//  // intialize folding
//  body_seq = BODY_SEQ_RETRACT_4;
//  body_seq_leg = 0;
//  body_seq_timer = millis();
//} // end body_start_folding
//
//
////====================================================
//// body start unfolding
////====================================================
//void body_start_unfolding(void){
//  const boolean local_debug = true;
//  if(local_debug) DEBUG_PRINTLN("in body_start_unfolding");
//  // intialize unfolding
//  body_seq = BODY_SEQ_EXTEND_4;
//  body_seq_leg = 0;
//  body_seq_timer = millis();
//} // end body_start_unfolding
//
//
////====================================================
//// body start walking
////====================================================
//void body_start_walking(void){
//  const boolean local_debug = true;
//  if(local_debug) DEBUG_PRINTLN("in body_start_walking");
//  // intialize walking
//  body_seq = BODY_SEQ_WALK_2;
//  body_seq_leg = 0;
//  body_seq_timer = millis();
//} // end body_start_walking
