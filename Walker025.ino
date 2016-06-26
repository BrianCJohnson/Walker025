//====================================================
// Walker main program
// Walker024 add initial folded to ready (unfolded) sequence
// Walker023 moved some variable declarations out of legs.h into legs.cpp
// Walker021 replaced array sizes with defined constants
// Walker020 changed some variables to structures, made constants CAPITALS
// Walker019 checkpoint
// Walker018 added body sequence logic
// Walker017 added start initial and final velocities in leg calculations
// Walker016 improved listing and corrections to leg calculations
// Walker015 added listing of leg path and corrections to leg calculations
// Walker014 added leg position tests
// Walker013 added leg position calculations
// Walker012 changed distance and radians to float
//====================================================

#include "debug.h"
#include "com.h"

// apparently need the following include here and in sbus.cpp
#include <FUTABA_SBUS.h> //note, FUTABA_SBUS.h was modified to "#define port Serial2"
#include "sbus.h"

// apparently need the following here and in servo.cpp
#include <PololuMaestro.h> // apparently need this here and in servo.cpp
#include "servo.h"

#include "legs.h"
#include "mode.h"

unsigned long last_time;

//====================================================
// Walker setup
//========================================================
void setup(){
  int8_t indent = 0;
  Serial.begin(115200);
  delay(800);
  debug_clr_new_mode();
  const char *routine = "setup";
//  static const boolean local_debug = true;
//  if(!local_debug) indent = -1;
  LOCAL_DEBUG_ENABLED
  if(local_debug) DEBUG_PRINT_BEG(routine, indent);
  sbus_setup(indent+1);
  servo_setup();
//  body_setup();
  legs_setup(indent+1);
//  legs_position_tests();
  mode_setup(indent+1);
  last_time = millis();
  if(local_debug) DEBUG_PRINT_END(routine, indent);
}
// end setup


//========================================================
// Walker loop
//========================================================
void loop(){
  //static unsigned long last_time = 0;
  const static int8_t indent = 0; // for debug prints
  unsigned long this_time = millis();
  boolean show_time = true;
  if(show_time){
    Serial.print("In loop, cycle time: ");
    Serial.print(this_time - last_time);
    Serial.print(", mode: ");
    Serial.print(MODE_NAME[mode_mode_phase_get().mode]);
    Serial.print(", phase: ");
    Serial.println(mode_mode_phase_get().phase);
//    Serial.printf("%s, %s, %d\n", __FILE__, __func__, __LINE__);
  }
  last_time = this_time;
//  servo_math_test();
//  servo_test();
  sbus_update(indent+1);
//  sbus_print_channels();
  mode_update(indent+1);
//  body_update();
//  position_update();
//  legs_update();
//  mode_display_position();
//  servo_print_angle();
//  servo_print_target();
  debug_clr_new_mode(); // clear new_mode
  delay(20); // note, if this is larger than our minumum move time (100 msec) then the system will continually fall behind the current time
} // end loop



