#ifndef __BOARD_H
#define __BOARD_H

/**
\addtogroup BSP
\{
\addtogroup board
\{

\brief Cross-platform declaration "board" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "board_info.h"
#include "toolchain_defs.h"

//=========================== define ==========================================

typedef enum {
   DO_NOT_KICK_SCHEDULER,
   KICK_SCHEDULER,
} kick_scheduler_t;

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void board_init(void);
void board_sleep(void);
void board_reset(void);
void board_get_location(int16_t* x, int16_t* y,int16_t* z,int16_t** neighbor_list,uint8_t num_neighbors);
void board_cmd_vel(float x_cmd, float y_cmd, float z_cmd);
/**
\}
\}
*/

#endif
