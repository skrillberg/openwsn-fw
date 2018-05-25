#include "opendefs.h"
#include "openqueue.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "opentimers.h"
#include "urocket.h"
#include "scheduler.h"

#include "accel_mimsy.h"
#include "flash_mimsy.h"
#include "servo.c"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "uart_mimsy.h"
#include "gpio.h"
#include "headers/hw_memmap.h"
#include "ioc.h"
#include "led_mimsy.h"
#include "inchworm.h"
#include <math.h>
//========================== Defines ==========================================

#define UROCKET_PERIOD_MS	10  //how often control task gets pushed to scheduler in ms
#define GYRO_FSR			2000 //gyro full scale range in deg/s

//========================== Global Variables =================================

urocket_vars_t urocket_vars;

//========================== Prototypes =======================================

void urocket_timer_cb(opentimers_id_t id);
void urocket_control_cb(void);

//=========================== public ==========================================

void urocket_init(void) {

    // clear local variables
    memset(&urocket_vars,0,sizeof(urocket_vars_t));

    // register at UDP stack
    urocket_vars.desc.port              = WKP_UDP_ROCKET;
    urocket_vars.desc.callbackReceive   = &urocket_receive;
    urocket_vars.desc.callbackSendDone  = &urocket_sendDone;
    openudp_register(&urocket_vars.desc);

    //initialize uart and imu/dmp
	uartMimsyInit();
	mimsyPrintf("\nClock Speed: %d",SysCtrlClockGet());
	mimsyLedInit();
	mimsyLedSet(GREEN_LED);

    servo_init(3,20,1.45);

    mpu_set_sensors(INV_XYZ_ACCEL|INV_XYZ_GYRO); //turn on sensor
    mpu_set_accel_fsr(16); //set fsr for accel
    mpu_set_gyro_fsr(GYRO_FSR); //set fsr for accel

    mimsyDmpBegin();

	short gyro[3]={0,0,0};
	short accel[3]={0,0,0};
	long quat[4]={0,0,0,0};
	unsigned long timestamp2 =0;
	unsigned char more =0;
	short sensors=INV_XYZ_GYRO | INV_WXYZ_QUAT|INV_XYZ_ACCEL;

    IMUData datapoint;
	dmp_read_fifo(gyro, accel, quat,&timestamp2, &sensors, &more);
	mimsyPrintf("\n Clearing Fifo:%d,%d,%d,%d,%d,%d",accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]);

	// start periodic timer
    urocket_vars.period = UROCKET_PERIOD_MS;
	urocket_vars.timerId = opentimers_create();
	opentimers_scheduleIn(
		urocket_vars.timerId,
		UROCKET_PERIOD_MS,
		TIME_MS,
		TIMER_PERIODIC,
		urocket_timer_cb
	);
}

//================================================================================
//Timer callback that pushes rocket control to scheduler

void urocket_timer_cb(opentimers_id_t id){

   scheduler_push_task(urocket_control_cb,TASKPRIO_COAP);
}

//================================================================================
///control code that is run everytime the control task is pushed to the scheduler

void urocket_control_cb(void){
	mimsyLedToggle(GREEN_LED);

}

//===================================================================

void urocket_receive(OpenQueueEntry_t* request) {
   uint16_t          temp_l4_destination_port;
   OpenQueueEntry_t* reply;

   reply = openqueue_getFreePacketBuffer(COMPONENT_UROCKET);
   if (reply==NULL) {
      openserial_printError(
         COMPONENT_UROCKET,
         ERR_NO_FREE_PACKET_BUFFER,
         (errorparameter_t)0,
         (errorparameter_t)0
      );
      openqueue_freePacketBuffer(request); //clear the request packet as well
      return;
   }

   reply->owner                         = COMPONENT_UROCKET;

   // reply with the same OpenQueueEntry_t
   reply->creator                       = COMPONENT_UROCKET;
   reply->l4_protocol                   = IANA_UDP;
   temp_l4_destination_port           = request->l4_destination_port;
   reply->l4_destination_port           = request->l4_sourcePortORicmpv6Type;
   reply->l4_sourcePortORicmpv6Type     = temp_l4_destination_port;
   reply->l3_destinationAdd.type        = ADDR_128B;

   // copy source to destination to echo.
   memcpy(&reply->l3_destinationAdd.addr_128b[0],&request->l3_sourceAdd.addr_128b[0],16);

   packetfunctions_reserveHeaderSize(reply,request->length);
   memcpy(&reply->payload[0],&request->payload[0],request->length);
   openqueue_freePacketBuffer(request);

   if ((openudp_send(reply))==E_FAIL) {
      openqueue_freePacketBuffer(reply);
   }

}

//======================================================================

void urocket_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}



