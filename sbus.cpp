//====================================================
// sbus.cpp
// sbus functions
// all functions for getting sBus data from
// RC receiver
//====================================================

#include "Arduino.h"
#include "sbus.h"
#include "debug.h"
#include "com.h"

#include <FUTABA_SBUS.h> //note, FUTABA_SBUS.h was modified to "#define port Serial2"

// create sBus, a FUTABA_SBUS object
FUTABA_SBUS sBus;

//#define SBUS_THROTTLE 0
//#define SBUS_AILERON 1
//#define SBUS_ELEVATOR 2
//#define SBUS_RUDDER 3
//#define SBUS_GEAR 4
//#define SBUS_FLAPS 5
//#define SBUS_AUX2 6
//#define SBUS_AUX3 7

static const uint8_t SBUS_NUM_CHANNELS = 6;
//uint16_t sbus_channel[SBUS_NUM_CHANNELS];

//====================================================
// setup sbus
//====================================================
void sbus_setup(int8_t indent){
  sBus.begin();
//  for(uint8_t i=0; i<; i++){
//    sbus_channel[i] = 0;
//  }
  delay(20); // needed to get receiver ready?
  sbus_update(indent);
}
// end sbus_setup


//====================================================
// update sbus
//====================================================
void sbus_update(int8_t indent){
  const static char *routine = "sbus_update";
  LOCAL_DEBUG_DISABLED
  if (local_debug) DEBUG_PRINT_BEG(routine, indent);
  
  static unsigned long oldmilliseconds, newmilliseconds;
  sBus.FeedLine();
  if (sBus.toChannels == 1){
    sBus.UpdateChannels();
    sBus.toChannels = 0;
    oldmilliseconds = newmilliseconds;
    newmilliseconds = millis();
    if (local_debug) {
      DEBUG_INDENT(indent+1);
      DEBUG_PRINT(newmilliseconds-oldmilliseconds);
      if(true){
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_THROTTLE]); // throttle
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_AILERON]); // aileron
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_ELEVATOR]); // elevator
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_RUDDER]); // rudder
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_GEAR]); // gear
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_FLAPS]); // flaps
        DEBUG_PRINTF("\t%d",sBus.channels[SBUS_AUX2]); // aux2
        DEBUG_PRINTF("\t%d\n",sBus.channels[SBUS_AUX3]); // aux3
      }
    }
  }  
  if (local_debug) DEBUG_PRINT_END(routine, indent);
}
// end update_sbus


//====================================================
// sbus_print_channels
//====================================================
void sbus_print_channels(void){
  if(true){
    DEBUG_PRINT("in sbus_print_channels\t");
    DEBUG_PRINT(sBus.channels[SBUS_THROTTLE]); // throttle
    DEBUG_PRINT("\t");
    DEBUG_PRINT(sBus.channels[SBUS_AILERON]); // aileron
    DEBUG_PRINT("\t");
    DEBUG_PRINT(sBus.channels[SBUS_ELEVATOR]); // elevator
    DEBUG_PRINT("\t");
    DEBUG_PRINT(sBus.channels[SBUS_RUDDER]); // rudder
    DEBUG_PRINT("\t");
    DEBUG_PRINT(sBus.channels[SBUS_GEAR]); // gear
    DEBUG_PRINT("\t");
    DEBUG_PRINT(sBus.channels[SBUS_FLAPS]); // flaps
    DEBUG_PRINT("\t");
    DEBUG_PRINT(sBus.channels[SBUS_AUX2]); // aux2
    DEBUG_PRINT("\t");
    DEBUG_PRINT(sBus.channels[SBUS_AUX3]); // aux3
    DEBUG_PRINTLN();
  }
} // end sbus_print_channels


//====================================================
// sbus_channel returns data from specified channel
//====================================================
int16_t sbus_channel(uint8_t channel){
  return sBus.channels[channel];
}
// end sbus_channel


//====================================================
// SBUS_GEAR_up returns boolean if "gear" is up
//====================================================
boolean sbus_gear_up(int8_t indent){
  LOCAL_DEBUG_DISABLED
  if(local_debug){
    DEBUG_INDENT(indent);
    DEBUG_PRINT("in SBUS_GEAR_up, sBus.channels[SBUS_GEAR]:");
    DEBUG_PRINTLN(sBus.channels[SBUS_GEAR]);
  }
  return (sBus.channels[SBUS_GEAR] < 1024);
}
// end SBUS_GEAR_up

