#ifndef __UROCKET_H
#define __UROCKET_H

#include "openudp.h"
#include "opentimers.h"
#include "flash_mimsy.h"
//=========================== define ==========================================

#define BUFFER_SIZE		50

//=========================== typedef =========================================
enum{
	INITIAL_STATE	=-1,
	RC_BYPASS		=0,
	TRAJ_PROGRAM	=1,

};
enum{
	DISARMED = 0,
	TRIGGERED = 1,
	INFLIGHT = 2,
	POSTLAUNCH = 3,
};
//=========================== variables =======================================
typedef struct{
	float roll[50];   ///desired roll angles for each sample period
	float pitch[50];  ///desired pitch angles for each sample period
	float yaw[50];    ///desired yaw angle for ewach sample period
	float time[50];   ///time at each trajectory point
	float sample_time;///sample period between each trajectory point
} rocket_trajectory;

typedef struct {
   opentimers_id_t     timerId;  ///< periodic timer which triggers control
   opentimers_id_t     timerIdSend;  ///< periodic timer which triggers transmission
   uint16_t             counter;  ///< incrementing counter which is written into the packet
   uint16_t              period;  ///< uinject packet sending period>
   IMUData 			logdata[BUFFER_SIZE];  ///<array of datapoints stored in ram. Eventually saved into flash
   int				buff_length;
   rocket_trajectory  trajectory;  ///desired rocket trajectory that is uploaded to rocket via UDP
   short 			gyro[3];
   short 			accel[3];
   long 			quat[4];
   unsigned long timestamp;
   float 		servo_time_0;
   float 		servo_time_1;

   union {
     float flt;
     unsigned char bytes[4];
   } roll;

   union {
     float flt;
     unsigned char bytes[4];
   } pitch;

   union {
     float flt;
     unsigned char bytes[4];
   } yaw;
   int	rocket_mode;
   float yaw_last;
   float roll_last;
   float pitch_last;
   float yaw_ref;
   float roll_ref;
   float pitch_ref;
   union {
     float flt;
     unsigned char bytes[4];
   } rc_yaw;

   union {
     float flt;
     unsigned char bytes[4];
   } rc_pitch;

   union {
     float flt;
     unsigned char bytes[4];
   } rc_roll;

   int rocket_state;
   IMUDataCard 		cards_stable[1];
   udp_resource_desc_t     desc;  ///< resource descriptor for this module, used to register at UDP stack
} urocket_vars_t;

void urocket_init(void);
void urocket_receive(OpenQueueEntry_t* msg);
void urocket_sendDone(OpenQueueEntry_t* msg, owerror_t error);
void urocket_control(void);

#endif
