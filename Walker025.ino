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

#include "body.h"
#include "legs.h"
#include "mode.h"

unsigned long last_time;

//====================================================
// Walker setup
//========================================================
void setup(){
  const static uint8_t indent = 0;
  Serial.begin(115200);
  delay(800);
  static const boolean local_debug = true;
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("Beg setup");
  }
  //delay(1000);
  sbus_setup();
  servo_setup();
//  body_setup();
  legs_setup(indent+1);
//  legs_position_tests();
  mode_setup(indent);
  last_time = millis();
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINTLN("End setup");
  }
}
// end setup


//========================================================
// Walker loop
//========================================================
void loop(){
  //static unsigned long last_time = 0;
  const static uint8_t indent = 0; // for debug prints
  unsigned long this_time = millis();
  boolean show_time = true;
  if(show_time){
    Serial.print("In loop, cycle time: ");
    Serial.print(this_time - last_time);
    Serial.print(", mode: ");
    Serial.print(MODE_NAME[mode_mode_phase_get().mode]);
    Serial.print(", phase: ");
    Serial.println(mode_mode_phase_get().phase);
  }
  last_time = this_time;
//  servo_math_test();
//  servo_test();
//  sbus_print_channels();
  sbus_update();
  mode_update(indent+1);
//  body_update();
//  position_update();
//  legs_update();
  delay(100);
} // end loop



