//========================================================
// mode.h
// definitions for mode functions
//========================================================

#ifndef mode_h
  #define mode_h
  #include "Arduino.h"
  #include "legs.h"

  struct mode_phase_s {
    uint8_t alternate; // above a mode, for example am I moving the right or left side? this will alternate with each succesive mode
    uint8_t mode; // the mode of action
    uint8_t direction; // plus or minus (forward/backward, right/left, counterclockwise/clockwise, unfolding/folding?)
    uint8_t phase; // the phase within a mode, for example a walking phase consists of moving a rear foot, then a front foot on the same side
    uint32_t start_time; // millis() at the start of the phase, unsigned long;
    uint32_t end_time; // what millis() should be at the end, calculated when calculating move_points, unsigned long;
  };
  typedef struct mode_phase_s mode_phase_t;

  struct mode_values_s {
    boolean fold; // fold (vs. unfold)
    float vy; // y velocity (forward/backward)
    float vx; // x velocity (side to side)
    float vt; // angular velocity (rotation)
    float height; // height (future)
    float angle; // camera angle (future)
  };
  typedef struct mode_values_s mode_values_t;

  struct mode_seq_part_s {
    uint8_t part_id; // LEG0, LEG1, LEG2, LEG3, BODY, NONE
    float d[XYZ]; // xyz displacement/distance 
    // for both the body and leg cases d is relative to the center of the body position at ground level at the start of the move
    // so for the body it is a relative move from where it is at the start of the move for x and y, starting at zero and moving to d[XY]
    // for the body d[Z] is the absolute height of the body center above the nominal ground level
    // all SUPPORT legs will have the body position subtracted from their position, so for example if the body moves forward during a move, all the SUPPORT legs move back during the move
    // if the body raises to or is at d[Z], all SUPPORT leg feet will be at -Z relative to the body center
    // for an ACTIVE leg d specifies the end position relative to the body center, so if the body moves during the move the active leg will end up moving body_d + leg_d - leg_start
    // for ACTIVE legs d[Z] specifies the leg lift amount and the leg will finish at z = 0.0
    // in other words for ACTIVE legs, x and y will move to d[X] and d [Y] relative to the body reference frame at the end of the move, z will always end at 0.0 but be adjuste by the body z
    // when computing move-points for the body:
    //   for x,y we will ignore the absolute body_x and body_y, assume they are 0.0 at the start so the displacement is merely the body d[XY]
    //   for z, d[Z] is the absolute hieght above nominal ground level, the displacement will be body d[Z] - body_z
    // when computing move_points for an ACTIVE leg:
    //   for x,y we will start with the leg_x, leg_y and the displacements will be disp_x = d[X] - leg_x and disp_y = d[Y] - leg_xy
    //   for z, d[Z] is only the lift hieght and the we will use that as the displacement when generating move_points
    // when computing where the legs should be moved to by calling legs_angles(legs_xyz[legs][XYZ], angles we need to combine the body coordinates with the individual leg coordinates
    //   for SUPPORT legs, legs_xyz[support_leg][XYZ] = legs_xyz_start[support_leg][XYZ] - body_move[XYZ] (if only translation!!!, otherwise we need to map the start coor into the new body ref frame)
    //   - the legs_xyz_start[support_leg][XYZ] and body_xyz_start[XYZ] values must be updated at the end of the move with the calculated final results of the body move displacements
    //   for ACTIVE legs, 
    //   - legs_xyz[active_leg][XY] = legs_xyz_start[active_leg][XY] + active_leg_move[XY]
    //   - legs_xyz[active_leg][Z] = legs_xyz_start[active_leg][Z] + active_leg_move[Z] - body_move[Z]
    //   for the body, body_xyz[XYZ] = body_xyz_start[XYZ] + body_move[XYZ]
    float v[XYZ]; // ending velocity, if the part is the body it is an absolute velocity, if the part is a leg the velocity is relative to the body reference frame
    // we need a good way to calculate what these ending velocities should be. 
    //   Could they be computed by the move_points calculation if we say the end velocity is either 0.0 or the max possible at the end of the move?
  };
  typedef struct mode_seq_part_s mode_seq_part_t;

  const static uint8_t MODE_SEQ_PART_NUM = 2;
  
  const static uint8_t MODE_SEQ_PART0 = 0;
  const static uint8_t MODE_SEQ_PART1 = 1;
  const static uint8_t MODE_SEQ_NONE = 2;
  //const static uint8_t MODE_SEQ_PART_NUM = 3;
  const String MODE_SEQ_PART_NAME[MODE_SEQ_PART_NUM+1] = {"SEQ_PART_0", "SEQ_PART_1", "SEQ_PART_X"};
  
  struct mode_seq_s {
    boolean last_phase; // true if this is the last phase of a move sequence
    String phase_name; // this is descriptive but the last phase name MUST begin with 
    mode_seq_part_t part[MODE_SEQ_PART_NUM];
  };
  typedef struct mode_seq_s mode_seq_t;


  const static uint8_t MODE_FOLDED = 0;
  const static uint8_t MODE_UNFOLDING = 1;
  const static uint8_t MODE_READY = 2;
  const static uint8_t MODE_FOLDING = 3;
  const static uint8_t MODE_WALKING = 4;
  const static uint8_t MODE_WALKING_BEG = 5;
  const static uint8_t MODE_WALKING_END = 6;
  const static uint8_t MODE_SIDESTEPPING = 7;
  const static uint8_t MODE_SIDESTEPPING_BEG = 8;
  const static uint8_t MODE_SIDESTEPPING_END = 9;
  const static uint8_t MODE_ROTATING = 10;
  const static uint8_t MODE_ROTATING_BEG = 11;
  const static uint8_t MODE_ROTATING_END = 12;
  const static uint8_t MODE_NUM = 13;
  const String MODE_NAME[MODE_NUM] = {"FOLDED", "UNFOLDING", "READY", "FOLDING", "WALKING", "WALKING_BEG", "WALKING_END", "SIDESTEPPING", "SIDESTEPPING_BEG", "SIDESTEPPING_END", "ROTATING", "ROTATING_BEG", "ROTATING_END"};
  
  const static uint8_t MODE_DIR_PLUS = 0;
  const static uint8_t MODE_DIR_MINUS = 1;
  const static uint8_t MODE_DIR_NUM = 2;
  const String MODE_DIR_NAME[MODE_DIR_NUM] = {"PLUS", "MINUS"};
  
  const static uint8_t MODE_PART_ACTIVITY_SUPPORT_LEG = 0;
  const static uint8_t MODE_PART_ACTIVITY_ACTIVE_LEG = 1;
  const static uint8_t MODE_PART_ACTIVITY_STATIC_BODY = 2;
  const static uint8_t MODE_PART_ACTIVITY_ACTIVE_BODY = 3;
  const static uint8_t MODE_PART_ACTIVITY_NUM = 4;
  const String MODE_PART_ACTIVITY_NAME[MODE_PART_ACTIVITY_NUM] = {"SUPPORT_LEG", "ACTIVE_LEG", "STATIC_BODY", "ACTIVE_BODY"};

  const static uint8_t MODE_PART_LEG0 = 0;
  const static uint8_t MODE_PART_LEG1 = 1;
  const static uint8_t MODE_PART_LEG2 = 2;
  const static uint8_t MODE_PART_LEG3 = 3;
  const static uint8_t MODE_PART_BODY = 4;
  const static uint8_t MODE_PART_NONE = 5;
  const static uint8_t MODE_PART_NUM = 5;
  const String MODE_PART_NAME[MODE_PART_NUM+1] = {"LEG0", "LEG1", "LEG2", "LEG3", "BODY", "NONE"};

//  const static uint8_t MODE_ACTIVE_PART_NUM = 2; // only allowing 2 parts to be active at a time
  const static uint8_t MODE_PHASE_MAX_NUM = 7; // only allowing 7 phases in a sequence
  
  void mode_setup(uint8_t indent);
  void mode_initialize_parts(uint8_t indent);
  void mode_create_sequences(uint8_t indent);
  void mode_print_sequences(uint8_t indent);
  float mode_calc_step_time(float step_x, float step_y, float lift_height, uint8_t indent);
  void mode_update(uint8_t indent);
  void mode_check_new_mode(uint8_t indent);
  void mode_print_current_time(uint8_t indent);
  void mode_execute_seq(uint8_t indent);
  void mode_print_move_part_points(uint8_t indent);
  void mode_print_part_parameters(const char *text, float part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], uint8_t indent);
  void mode_print_parts_xyz(const char *text, float part_xyz[MODE_PART_NUM][XYZ], uint8_t indent);
  void mode_print_parts_activity_seq_part(const char *text, uint8_t part_activity[MODE_PART_NUM], uint8_t part_seq_part[MODE_PART_NUM], uint8_t indent);
  void mode_update_move_part_data(uint8_t new_mode, uint8_t new_phase, uint8_t indent);
  void mode_execute_move(float current_time, mode_seq_t mode_seq_phase, boolean *move_done, boolean *zero_vel, uint8_t indent);
  void mode_create_next_part_parameters(mode_seq_t next_seq_phase, float next_part_beg_xyz[MODE_PART_NUM][XYZ], float part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], uint8_t part_activity_type[MODE_PART_NUM], uint8_t part_seq_part[MODE_PART_NUM], float next_part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], uint8_t indent);
  void mode_create_next_part_activity_seq_part(mode_seq_t next_seq_phase, uint8_t next_part_activity[MODE_PART_NUM], uint8_t next_part_seq_part[MODE_PART_NUM], uint8_t indent);
  void mode_create_next_part_beg_xyz(mode_seq_t next_seq_phase, float part_parameters[MODE_SEQ_PART_NUM][XYZ][LEGS_PARAM_NUM], uint8_t part_activity_type[MODE_PART_NUM], uint8_t part_seq_part[MODE_PART_NUM], float part_beg_xyz[MODE_PART_NUM][XYZ], float next_part_beg_xyz[MODE_PART_NUM][XYZ], uint8_t indent);
  void mode_set_folded(uint8_t indent);
  void mode_set_ready(uint8_t indent);
  void mode_set_dir(uint8_t newdir, uint8_t indent);
  void mode_set(uint8_t new_mode, uint8_t indent);
  mode_phase_t mode_mode_phase_get(void);
  uint8_t mode_mode_get(void);
  uint8_t mode_phase_get(void);
  mode_values_t mode_values_get(void);
  boolean mode_value_fold(void);
  float mode_value_vy(void);
  float mode_value_vx(void);
  float mode_value_vt(void);
  float mode_value_height(void);
  float mode_value_angle(void);
  void mode_values_update(void);
  
#endif
