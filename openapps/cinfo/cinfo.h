#ifndef __CINFO_H
#define __CINFO_H

/**
\addtogroup AppCoAP
\{
\addtogroup cinfo
\{
*/

#include "opencoap.h"
#include "opentimers.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

typedef union{
   float flt;
   uint8_t bytes[4];
}floatbyte_t;

typedef union{
   short shrt;
   uint8_t bytes[2];
}shortbyte_t;

typedef struct{
   floatbyte_t x;
   floatbyte_t y;
   floatbyte_t z;	
}position_t;

//typedef struct{
 //  floatbyte_t position[3];
//} neighbor_t;

typedef struct {
   coap_resource_desc_t desc;
   opentimers_id_t  timerId;
   uint16_t period;
   floatbyte_t xlocation;
   floatbyte_t ylocation;
   floatbyte_t zlocation;
   uint16_t accelx;
   uint16_t accely;
   uint16_t accelz;
   uint8_t frame_start;
   uint32_t byte_count;
   uint8_t rx_buf[200];
   uint8_t rx_ready;
   uint8_t listening;
   float time;
   floatbyte_t controls[3];
  // neighbor_t neighbors[30];
} cinfo_vars_t;

//=========================== prototypes ======================================


/**
\}
\}
*/

#endif
