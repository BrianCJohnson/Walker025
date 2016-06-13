//========================================================
// common.cpp
// common functions
//========================================================

#include "Arduino.h"
#include "debug.h"
#include "com.h"


//========================================================
// com_free_mem()
//========================================================
void com_free_mem(void){
    uint32_t stackTop;
    uint32_t heapTop;

    // current position of the stack.
    stackTop = (uint32_t) &stackTop;

    // current position of heap.
    void* hTop = malloc(1);
    heapTop = (uint32_t) hTop;
    free(hTop);

    // The difference is the free, available ram.
    Serial.print(", free: ");
    Serial.println(stackTop - heapTop);
//  extern int __heap_start, *__brkval; 
//  int v; 
//  int fr = (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
//  Serial.print("com_free_ram: ");
//  Serial.println(fr);
} // end com_free_ram


//========================================================
// com_indent()
//========================================================
void com_indent(uint8_t indent){
  for(uint8_t i=0; i<indent; i++){
    Serial.print("|  ");  
  }
} // end com_indent


//========================================================
// com_print_beg()
//========================================================
void com_print_beg(const char routine[], uint8_t indent){
  com_indent(indent);
  Serial.printf("Beg %s", routine);
//  Serial.print("Beg ");
//  Serial.print(routine);
  // print time?
} // com_print_beg


//========================================================
// com_print_end()
//========================================================
void com_print_end(const char routine[], uint8_t indent){
  com_indent(indent);
  Serial.printf("End %s", routine);
//  Serial.print("End ");
//  Serial.print(routine);
  // print time?
} // com_print_end


//========================================================
// com_sign_mag
//========================================================
void com_sign_mag(float number, int8_t *sign, float *mag){
  static const boolean local_debug = false;
  if(number>=0.0){
    *mag = number;
    *sign = 1;
  } else {
    *mag = -number;
    *sign = -1;
  }
  if(local_debug){
    DEBUG_PRINT("In common_sign_mag, number: ");
    DEBUG_PRINTF("%7.2f", number);
    DEBUG_PRINT(", sign: ");
    DEBUG_PRINT(*sign);
    DEBUG_PRINT(", *mag: ");
    DEBUG_PRINTF("%7.2f\n", *mag);
  }
} // com_sign_mag


//========================================================
// com_err_msg_int
//========================================================
void com_err_msg_int(const char routine[], const char err_msg[]){
  Serial.print("COM_ERR_MSG: routine:");
  Serial.print(routine);
  Serial.print(", error: ");
  Serial.print(err_msg);
} // end com_err_msg_int


//========================================================
// com_err_msg
//========================================================
void com_err_msg(const char routine[], const char err_msg[]){
  com_err_msg_int(routine, err_msg);
  Serial.println();
} 

//void com_err_msg(const char routine[], const char err_msg[], uint8_t uint8_t_value){
//  com_err_msg_int(routine, err_msg);
//  Serial.print(" uint8_t: ");
//  Serial.println(uint8_t_value);
//}
//
//void com_err_msg(const char routine[], const char err_msg[], float float_value){
//  com_err_msg_int(routine, err_msg);
//  Serial.print(" float: ");
//  Serial.println(float_value);
//}

void com_err_msg(const char routine[], const char err_msg[], uint8_t uint8_t_value, float float_value){
  com_err_msg_int(routine, err_msg);
  Serial.print(" uint8_t: ");
  Serial.print(uint8_t_value);
  Serial.print(", float: ");
  Serial.println(float_value);  
} // end com_err_msg

