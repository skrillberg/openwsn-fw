#ifndef __tutorial_H
#define __tutorial_H

/**
\addtogroup AppUdp
\{
\addtogroup tutorial
\{
*/

#include "opentimers.h"
#include "openudp.h"

//=========================== define ==========================================

#define tutorial_PERIOD_MS 10000

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
   opentimers_id_t     timerId;  ///< periodic timer which triggers transmission
   uint16_t             counter;  ///< incrementing counter which is written into the packet
   uint16_t              period;  ///< tutorial packet sending period>
   udp_resource_desc_t     desc;  ///< resource descriptor for this module, used to register at UDP stack
   uint16_t	     send_count;  ///<our send counter used to track how many packets we have left to send
   uint16_t	     rssi; //rssi from radio
   uint16_t	     corr; //7 bit correlation from received packet
} tutorial_vars_t;

//=========================== prototypes ======================================

void tutorial_init(void);
void tutorial_sendDone(OpenQueueEntry_t* msg, owerror_t error);
void tutorial_receive(OpenQueueEntry_t* msg);
/**
\}
\}
*/

#endif

