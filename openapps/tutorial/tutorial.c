#include "opendefs.h"
#include "tutorial.h"
#include "openqueue.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "scheduler.h"
#include "IEEE802154E.h"
#include "idmanager.h"

//accelerometer firmware
#include "accel_mimsy.h"
#include "flash_mimsy.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml_math_func.h"
#include <math.h>

//=========================== variables =======================================

tutorial_vars_t tutorial_vars;

static const uint8_t tutorial_payload[]    = "tutorial";
static const uint8_t tutorial_dst_addr[]   = {
   0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
}; 

//=========================== prototypes ======================================

void tutorial_timer_cb(opentimers_id_t id);
void tutorial_task_cb(void);

//=========================== public ==========================================

void tutorial_init(void) {
   
    // clear local variables
    memset(&tutorial_vars,0,sizeof(tutorial_vars_t));

    // register at UDP stack
    tutorial_vars.desc.port              = WKP_UDP_TUTORIAL;
    tutorial_vars.desc.callbackReceive   = &tutorial_receive;
    tutorial_vars.desc.callbackSendDone  = &tutorial_sendDone;
    tutorial_vars.send_count = 0;
    openudp_register(&tutorial_vars.desc);
	
    tutorial_vars.period = tutorial_PERIOD_MS;
    // start periodic timer
    tutorial_vars.timerId = opentimers_create();
    opentimers_scheduleIn(
        tutorial_vars.timerId,
        tutorial_PERIOD_MS,
        TIME_MS,
        TIMER_PERIODIC,
        tutorial_timer_cb
    );
    //uartMimsyInit();
    // initialize imu on mimsy
    mimsyIMUInit();
    mpu_set_sensors(INV_XYZ_ACCEL|INV_XYZ_GYRO); //turn on sensor
    mpu_set_accel_fsr(16); //set fsr for accel
    mpu_set_gyro_fsr(2000); //set fsr for accel

    mimsyDmpBegin();

}

void tutorial_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}

void tutorial_receive(OpenQueueEntry_t* pkt) {
   //if we recieve an 'i', we intiate 10 packets to be sent by timer
  // if(pkt->payload[0]=='i' ){
	tutorial_vars.send_count = 10;
scheduler_push_task(tutorial_task_cb,TASKPRIO_COAP);
  // }
   openqueue_freePacketBuffer(pkt);
   
}

//=========================== private =========================================

/**
\note timer fired, but we don't want to execute task in ISR mode instead, push
   task to scheduler with CoAP priority, and let scheduler take care of it.
*/
void tutorial_timer_cb(opentimers_id_t id){
   //if send_count is non zero, push task and decrement counter
   if(tutorial_vars.send_count>-1){
   	scheduler_push_task(tutorial_task_cb,TASKPRIO_COAP);
	tutorial_vars.send_count--;
   }
}

void tutorial_task_cb(void) {
   OpenQueueEntry_t*    pkt;
   uint8_t              asnArray[5];
   
   // don't run if not synch
   if (ieee154e_isSynch() == FALSE) return;
   
   // don't run on dagroot
   if (idmanager_getIsDAGroot()) {
      opentimers_destroy(tutorial_vars.timerId);
      return;
   }
   
   // if you get here, send a packet
   
   // get a free packet buffer
   pkt = openqueue_getFreePacketBuffer(COMPONENT_tutorial);
   if (pkt==NULL) {
      openserial_printError(
         COMPONENT_tutorial,
         ERR_NO_FREE_PACKET_BUFFER,
         (errorparameter_t)0,
         (errorparameter_t)0
      );
      return;
   }
   
   pkt->owner                         = COMPONENT_tutorial;
   pkt->creator                       = COMPONENT_tutorial;
   pkt->l4_protocol                   = IANA_UDP;
   pkt->l4_destination_port           = WKP_UDP_TUTORIAL;
   pkt->l4_sourcePortORicmpv6Type     = WKP_UDP_TUTORIAL;
   pkt->l3_destinationAdd.type        = ADDR_128B;
   memcpy(&pkt->l3_destinationAdd.addr_128b[0],tutorial_dst_addr,16);
   

   //read imu fifo
   short gyro[3]={0,0,0};
   short accel[3]={0,0,0};
   long quat[4]={0,0,0,0};
   short sensors;
   short more;
   unsigned long timestamp;
   //reads until fifo read doesn't return with an error code
   while(dmp_read_fifo(&gyro, &accel, &quat,&timestamp, &sensors, &more)!=0){

   }

	//*********************euler angle conversion**************
   union {
     float flt;
     unsigned char bytes[4];
   } yaw;
   union {
     float flt;
     unsigned char bytes[4];
   } pitch;
   union {
     float flt;
     unsigned char bytes[4];
   } roll;
   //takes quaternions from quats and converts to euler angles, storing each anlge in yaw, pitch, and roll unions
   quat2euler(quat,&(yaw.flt),&(pitch.flt),&(roll.flt));  

//*******************************


   // add payload  
   packetfunctions_reserveHeaderSize(pkt,6*sizeof(uint16_t)+3*sizeof(float));

   pkt->payload[1] = (uint8_t)((accel[0] & 0xff00)>>8);
   pkt->payload[0] = (uint8_t)(accel[0] & 0x00ff);

   pkt->payload[3] = (uint8_t)((accel[1] & 0xff00)>>8);
   pkt->payload[2] = (uint8_t)(accel[1] & 0x00ff);

   pkt->payload[5] = (uint8_t)((accel[2] & 0xff00)>>8);
   pkt->payload[4] = (uint8_t)(accel[2] & 0x00ff);

   pkt->payload[7] = (uint8_t)((gyro[0] & 0xff00)>>8);
   pkt->payload[6] = (uint8_t)(gyro[0] & 0x00ff);

   pkt->payload[9] = (uint8_t)((gyro[1] & 0xff00)>>8);
   pkt->payload[8] = (uint8_t)(gyro[1] & 0x00ff);

   pkt->payload[11] = (uint8_t)((gyro[2] & 0xff00)>>8);
   pkt->payload[10] = (uint8_t)(gyro[2] & 0x00ff);

   //***************euler angels****************************
   pkt->payload[15] = roll.bytes[3];
   pkt->payload[14] = roll.bytes[2];
   pkt->payload[13] = roll.bytes[1];
   pkt->payload[12] = roll.bytes[0];

   pkt->payload[19] = pitch.bytes[3];
   pkt->payload[18] = pitch.bytes[2];
   pkt->payload[17] = pitch.bytes[1];
   pkt->payload[16] = pitch.bytes[0];

   pkt->payload[23] = yaw.bytes[3];
   pkt->payload[22] = yaw.bytes[2];
   pkt->payload[21] = yaw.bytes[1];
   pkt->payload[20] = yaw.bytes[0];

   if ((openudp_send(pkt))==E_FAIL) {
      openqueue_freePacketBuffer(pkt);
   }
}

void quat2euler(long * quat,float * yaw, float * pitch, float * roll){
      float fquats[4];
   //conversion to float
   fquats[0]=(float)quat[0]/(float)0x40000000;
   fquats[1]=(float)quat[1]/(float)0x40000000;
   fquats[2]=(float)quat[2]/(float)0x40000000;
   fquats[3]=(float)quat[3]/(float)0x40000000;

   inv_q_norm4(fquats);

   *pitch = asinf( 2*(fquats[0]*fquats[2]-fquats[3]*fquats[1])); //computes sin of pitch

   //gyro yaw

   *yaw = atan2f(2*(fquats[0] * fquats[3] + fquats[1] * fquats[2]),1 - 2*(fquats[2]*fquats[2] + fquats[3]*fquats[3]));

   //roll control

   *roll=  atan2f(2 * (fquats[0]*fquats[1] + fquats[2] * fquats[3]) ,(1 -2*(fquats[1] * fquats[1] +fquats[2]*fquats[2])));
}

