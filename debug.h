//========================================================
// debug.h
// definitions for debug functions
//========================================================

#ifndef debug_h
  #define debug_h
  #include "Arduino.h"

  #define DEBUG

  #ifdef DEBUG
    #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
    #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
    #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
    #define DEBUG_INDENT(...) com_indent(__VA_ARGS__)
    #define DEBUG_PRINT_BEG(...) com_print_beg(__VA_ARGS__)
    #define DEBUG_PRINT_END(...) com_print_beg(__VA_ARGS__)
  #else
    #define DEBUG_PRINT(...)
    #define DEBUG_PRINTLN(...)
    #define DEBUG_PRINTF(...)
    #define DEBUG_INDENT(...)
    #define DEBUG_PRINT_BEG(...)
    #define DEBUG_PRINT_END(...)
  #endif
  
#endif
