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
#include "ml_math_func.h"
#include <math.h>
//========================== Defines ==========================================

#define UROCKET_PERIOD_MS	50 //how often control task gets pushed to scheduler in ms
#define UROCKET_TX_PERIOD_MS 200 //how often send packet task gets pushed to scheduler in ms
#define GYRO_FSR			2000 //gyro full scale range in deg/s
#define DATAPOINTS			100 //number of datapoints?
#define FLASH_PAGES_TOUSE	50
#define FLASH_PAGE_STORAGE_START 100 //first flash page to start at. TODO: make sure this doesn't overlap with code base
//========================== Global Variables =================================

urocket_vars_t urocket_vars;

//========================== Prototypes =======================================
void urocket_sendpacket(void);
void urocket_timer_cb(opentimers_id_t id);
void urocket_send_timer_cb(opentimers_id_t id);
void urocket_control_cb(void);
void printFlash(IMUDataCard * cards_stable, int page_struct_capacity);
void create_datapoint(urocket_vars_t urocket_var,IMUData* datapoint);
//=========================== public ==========================================

void urocket_init(void) {

    // clear local variables
    memset(&urocket_vars,0,sizeof(urocket_vars_t));

    urocket_vars.rocket_mode = INITIAL_STATE;
    // register at UDP stack
    urocket_vars.desc.port              = WKP_UDP_ROCKET;
    urocket_vars.desc.callbackReceive   = &urocket_receive;
    urocket_vars.desc.callbackSendDone  = &urocket_sendDone;
    openudp_register(&urocket_vars.desc);
    urocket_vars.buff_length=BUFFER_SIZE;
    //initialize uart and imu/dmp
	uartMimsyInit();
	mimsyPrintf("\nClock Speed: %d",SysCtrlClockGet());
	//mimsyLedInit();
	//mimsyLedSet(GREEN_LED);
	urocket_vars.rocket_state = DISARMED;
	urocket_vars.rocket_mode = INITIAL_STATE;
	urocket_vars.launched = 0;
	urocket_vars.armed = 0;
	//initialize trajectory
	urocket_vars.trajectory.roll[1]=3;
	urocket_vars.trajectory.sample_time = 1000; //1 second sample period for trajectory

    servo_init(3,20,1.45);
    urocket_vars.servo_time_0.flt=1.45;
    urocket_vars.servo_time_1.flt=1.45;

    //create index cards for flash storage
  //  for(int i=0;i<FLASH_PAGES_TOUSE;i++){
   //   (urocket_vars.cards_stable[i].page)=FLASH_PAGE_STORAGE_START+i;
   // }

    //initialize and test imu
    mimsyIMUInit();
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

	urocket_vars.timerIdSend = opentimers_create();
	opentimers_scheduleIn(
		urocket_vars.timerIdSend,
		UROCKET_TX_PERIOD_MS,
		TIME_MS,
		TIMER_PERIODIC,
		urocket_send_timer_cb
	);
}

//================================================================================
//Timer callback that pushes rocket control to scheduler

void urocket_timer_cb(opentimers_id_t id){


   scheduler_push_task(urocket_control_cb,TASKPRIO_COAP+1);

}

void urocket_send_timer_cb(opentimers_id_t id){

	scheduler_push_task(urocket_sendpacket,TASKPRIO_COAP);


}

//================================================================================
///control code that is run everytime the control task is pushed to the scheduler

void urocket_control_cb(void){

	//**********************vars*******************************************
	//i should put these in the global struct

	IMUData datapoint;
	short sensors=INV_XYZ_GYRO | INV_WXYZ_QUAT|INV_XYZ_ACCEL;
	short more;
	float fquats[4] = {0,0,0,0};

	short gyro[3] = {0,0,0};
	short accel[3] = {0,0,0};
	int error;
	//***********************read state information************************
	//mimsyLedToggle(GREEN_LED);

	//read fifo from dmp. I currently have two reads in a row
	//because otherwise, every other read returns nothing

	//error = dmp_read_fifo((urocket_vars.gyro), (urocket_vars.accel), (urocket_vars.quat),&(urocket_vars.timestamp), &sensors, &more);
	//error = dmp_read_fifo((gyro), (accel), (urocket_vars.quat),&(urocket_vars.timestamp), &sensors, &more);

	while(dmp_read_fifo((urocket_vars.gyro), (urocket_vars.accel), (urocket_vars.quat),&(urocket_vars.timestamp), &sensors, &more)!=0){
		//mimsyPrintf("\n dmp fifo error");
	}
	//mimsyPrintf("\n Clearing Fifo:%d,%d,%d,%d,%d,%d",urocket_vars.accel[0],urocket_vars.accel[1],urocket_vars.accel[2],urocket_vars.gyro[0],urocket_vars.gyro[1],urocket_vars.gyro[2]);
	//mimsyPrintf("\n Clearing Fifo:%d,%d,%d,%d,%d,%d",accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]);
	// mimsyPrintf("\n Sensors: %d",sensors);
	//*********************euler angle conversion*****************************************************************
   //pitch control
   fquats[0]=(float)urocket_vars.quat[0]/(float)0x40000000;
   fquats[1]=(float)urocket_vars.quat[1]/(float)0x40000000;
   fquats[2]=(float)urocket_vars.quat[2]/(float)0x40000000;
   fquats[3]=(float)urocket_vars.quat[3]/(float)0x40000000;

   //mag = sqrtf( fquats[1] * fquats[1] + fquats[2] * fquats[2] + fquats[3] * fquats[3]);
   inv_q_norm4(fquats);
   urocket_vars.pitch_last = urocket_vars.pitch.flt; //save previous state
   urocket_vars.pitch.flt = asinf( 2*(fquats[0]*fquats[2]-fquats[3]*fquats[1])); //computes sin of pitch

   //gyro yaw
   urocket_vars.yaw_last = urocket_vars.yaw.flt;
   urocket_vars.yaw.flt = atan2f(2*(fquats[0] * fquats[3] + fquats[1] * fquats[2]),1 - 2*(fquats[2]*fquats[2] + fquats[3]*fquats[3]));

   //roll control
   urocket_vars.roll_last = urocket_vars.roll.flt;
   urocket_vars.roll.flt=  atan2f(2 * (fquats[0]*fquats[1] + fquats[2] * fquats[3]) ,(1 -2*(fquats[1] * fquats[1] +fquats[2]*fquats[2])));
   //mimsyPrintf("\n Roll: %d. Pitch: %d, Yaw: %d",(int)(roll*100),(int)(pitch*100), (int)(yaw*100));

   //**********************control servo or finchworm********************************************************
   servo_rotate_time(urocket_vars.servo_time_0.flt,0);
   servo_rotate_time(urocket_vars.servo_time_1.flt,1);

	//*********************send data and save data********************************************************

   create_datapoint(urocket_vars,&datapoint);	//convert dmp data into a flash-saveable format. this is inefficient at the moment

   //state transistion from disarmed to rc_bypass if rc command is received
   if((urocket_vars.rocket_mode==RC_BYPASS) && (urocket_vars.rocket_state == INFLIGHT)){

	   urocket_vars.roll_ref=urocket_vars.rc_roll.flt;
	   urocket_vars.yaw_ref = urocket_vars.rc_yaw.flt;
	   urocket_vars.pitch_ref = urocket_vars.rc_pitch.flt;
	   //mimsyPrintf("RC Command Received");
   }
   else if((urocket_vars.rocket_mode==TRAJ_PROGRAM) && (urocket_vars.rocket_state == DISARMED)){

   }

}

//===================================================================

void urocket_receive(OpenQueueEntry_t* request) {
   uint16_t          temp_l4_destination_port;
   OpenQueueEntry_t* reply;

   reply = openqueue_getFreePacketBuffer(COMPONENT_UROCKET);
   mimsyPrintf("\n mode: %d, command", request->payload[0], request->payload[1]);

   //rc bypass mode state
   if((request->payload[0]==RC_BYPASS) && (urocket_vars.rocket_state==DISARMED)){
	   urocket_vars.rocket_mode=RC_BYPASS;
	   urocket_vars.rocket_state=INFLIGHT;
	   mimsyPrintf("state changed to inflight and rc_bypass");

   }else if((urocket_vars.rocket_mode==RC_BYPASS) && (urocket_vars.rocket_state==INFLIGHT)){
	   switch( (char)(request->payload[0])){
	   case 'l':
		   urocket_vars.servo_time_0.flt = SERVO_MIN;
		   urocket_vars.servo_time_1.flt = SERVO_MAX;
		   break;
	   case 'r':
		   urocket_vars.servo_time_0.flt = SERVO_MAX;
		   urocket_vars.servo_time_1.flt = SERVO_MIN;
		   break;
	   case 'm':
		   urocket_vars.servo_time_0.flt = (SERVO_MIN+SERVO_MAX)/2;
		   urocket_vars.servo_time_1.flt = (SERVO_MIN+SERVO_MAX)/2;
		   break;
	   default:
		   mimsyPrintf('bad rx for state');
	   }

	   /*
	    if((urocket_vars.rocket_mode==RC_BYPASS) && (urocket_vars.rocket_state==INFLIGHT)){
	 	   urocket_vars.rc_roll.bytes[0] =  request->payload[0];
	 	   urocket_vars.rc_roll.bytes[1] =  request->payload[1];
	 	   urocket_vars.rc_roll.bytes[2] =  request->payload[2];
	 	   urocket_vars.rc_roll.bytes[3] =  request->payload[3];
	 	   mimsyPrintf("roll updated");
	    }*/
   }

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

void urocket_sendpacket(){
	static const uint8_t urocket_payload[]    = "urocket";
	static const uint8_t urocket_dst_addr[]   = {
	   0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
	}; //local host address of computer
	OpenQueueEntry_t*    pkt;
	uint8_t              asnArray[5];

	// don't run if not synch
	if (ieee154e_isSynch() == FALSE) return;

	// don't run on dagroot
	if (idmanager_getIsDAGroot()) {
	  opentimers_destroy(urocket_vars.timerId);
	  return;
	}

	// get a free packet buffer
   pkt = openqueue_getFreePacketBuffer(COMPONENT_UROCKET);
   if (pkt==NULL) {
	  openserial_printError(
		 COMPONENT_UROCKET,
		 ERR_NO_FREE_PACKET_BUFFER,
		 (errorparameter_t)0,
		 (errorparameter_t)0
	  );
	  return;
   }

   pkt->owner                         = COMPONENT_UROCKET;
   pkt->creator                       = COMPONENT_UROCKET;
   pkt->l4_protocol                   = IANA_UDP;
   pkt->l4_destination_port           = WKP_UDP_ROCKET;
   pkt->l4_sourcePortORicmpv6Type     = WKP_UDP_ROCKET;
   pkt->l3_destinationAdd.type        = ADDR_128B;
   memcpy(&pkt->l3_destinationAdd.addr_128b[0],urocket_dst_addr,16);

   // add payload
   packetfunctions_reserveHeaderSize(pkt,sizeof(urocket_payload)-1);
   memcpy(&pkt->payload[0],urocket_payload,sizeof(urocket_payload)-1);

   packetfunctions_reserveHeaderSize(pkt,6*sizeof(uint16_t)+5*sizeof(float));
   //pkt->payload[1] = (uint8_t)((urocket_vars.counter & 0xff00)>>8);
   //pkt->payload[0] = (uint8_t)(urocket_vars.counter & 0x00ff);
   //mimsyPrintf("\n Clearing Fifo:%d,%d,%d,%d,%d,%d",urocket_vars.accel[0],urocket_vars.accel[1],urocket_vars.accel[2],urocket_vars.gyro[0],urocket_vars.gyro[1],urocket_vars.gyro[2]);

   pkt->payload[1] = (uint8_t)((urocket_vars.accel[0] & 0xff00)>>8);
   pkt->payload[0] = (uint8_t)(urocket_vars.accel[0] & 0x00ff);

   pkt->payload[3] = (uint8_t)((urocket_vars.accel[1] & 0xff00)>>8);
   pkt->payload[2] = (uint8_t)(urocket_vars.accel[1] & 0x00ff);

   pkt->payload[5] = (uint8_t)((urocket_vars.accel[2] & 0xff00)>>8);
   pkt->payload[4] = (uint8_t)(urocket_vars.accel[2] & 0x00ff);

   pkt->payload[7] = (uint8_t)((urocket_vars.gyro[0] & 0xff00)>>8);
   pkt->payload[6] = (uint8_t)(urocket_vars.gyro[0] & 0x00ff);

   pkt->payload[7] = (uint8_t)((urocket_vars.gyro[1] & 0xff00)>>8);
   pkt->payload[6] = (uint8_t)(urocket_vars.gyro[1] & 0x00ff);

   pkt->payload[9] = (uint8_t)((urocket_vars.gyro[2] & 0xff00)>>8);
   pkt->payload[8] = (uint8_t)(urocket_vars.gyro[2] & 0x00ff);

   //***************euler angels****************************
   pkt->payload[13] = urocket_vars.roll.bytes[3];
   pkt->payload[12] = urocket_vars.roll.bytes[2];
   pkt->payload[11] = urocket_vars.roll.bytes[1];
   pkt->payload[10] = urocket_vars.roll.bytes[0];

   pkt->payload[17] = urocket_vars.pitch.bytes[3];
   pkt->payload[16] = urocket_vars.pitch.bytes[2];
   pkt->payload[15] = urocket_vars.pitch.bytes[1];
   pkt->payload[14] = urocket_vars.pitch.bytes[0];

   pkt->payload[21] = urocket_vars.yaw.bytes[3];
   pkt->payload[20] = urocket_vars.yaw.bytes[2];
   pkt->payload[19] = urocket_vars.yaw.bytes[1];
   pkt->payload[18] = urocket_vars.yaw.bytes[0];

//SERVO STATES
   pkt->payload[25] = urocket_vars.servo_time_0.bytes[3];
   pkt->payload[24] = urocket_vars.servo_time_0.bytes[2];
   pkt->payload[23] = urocket_vars.servo_time_0.bytes[1];
   pkt->payload[22] = urocket_vars.servo_time_0.bytes[0];

   pkt->payload[29] = urocket_vars.servo_time_1.bytes[3];
   pkt->payload[28] = urocket_vars.servo_time_1.bytes[2];
   pkt->payload[27] = urocket_vars.servo_time_1.bytes[1];
   pkt->payload[26] = urocket_vars.servo_time_1.bytes[0];


  // packetfunctions_reserveHeaderSize(pkt,6*2);
  // ieee154e_getAsn(asnArray);

 //  pkt->payload[1] = (uint8_t)((urocket_vars.accel[0] & 0xff00)>>8);
 //  pkt->payload[0] = (uint8_t)(urocket_vars.accel[0] & 0x00ff);

   //pkt->payload[0] = urocket_vars.accel[0];
  // pkt->payload[1] = urocket_vars.accel[1];
   //pkt->payload[2] = urocket_vars.accel[2];
  // pkt->payload[3] = urocket_vars.gyro[0];
  // pkt->payload[4] = urocket_vars.gyro[1];
  // pkt->payload[5] = urocket_vars.gyro[2];


   if ((openudp_send(pkt))==E_FAIL) {
	  openqueue_freePacketBuffer(pkt);
   }
}
//-------------------------print flash function---------------------------------
//one caveat with flash is that if the flash write is interrupted, everything fails, may need to make it highest priority
void printFlash(IMUDataCard * cards_stable, int page_struct_capacity){


		 mimsyPrintf("\n data starts here:+ \n"); //+ is start condition

		 for(int cardindex=0;cardindex<FLASH_PAGES_TOUSE;cardindex++){

			 for(int words = 0; words < page_struct_capacity*IMU_DATA_STRUCT_SIZE/4*DATAPOINTS; words+=IMU_DATA_STRUCT_SIZE/4*DATAPOINTS){

				  IMUData sendData[DATAPOINTS];
				  flashReadIMUSection(cards_stable[cardindex],sendData,DATAPOINTS,words);

				  //loop through each data point
				  for(int dataindex=0;dataindex<DATAPOINTS;dataindex++){



					  //print csv data to serial
					  //format: xl_x,xl_y,xl_z,gyrox,gyroy,gyroz,timestamp
					mimsyPrintf("%d,%d,%d,%d,%d,%d,%d,%x,%x,%d,%d \n",
								  sendData[dataindex].signedfields.accelX,
								  sendData[dataindex].signedfields.accelY,
								  sendData[dataindex].signedfields.accelZ,
								  sendData[dataindex].signedfields.gyroX,
								  sendData[dataindex].signedfields.gyroY,
								  sendData[dataindex].signedfields.gyroZ,
								  sendData[dataindex].fields.timestamp,
								  sendData[dataindex].bits[4],
								  sendData[dataindex].bits[5],
								  cardindex,
								  dataindex+words*4/IMU_DATA_STRUCT_SIZE);


				  }
			 }
		}
		mimsyPrintf("= \n data ends here\n"); //= is end
}

void create_datapoint(urocket_vars_t urocket_var,IMUData* datapoint){
	(*datapoint).signedfields.accelX = urocket_var.accel[0];
	(*datapoint).signedfields.accelY = urocket_var.accel[1];
	(*datapoint).signedfields.accelZ = urocket_var.accel[2];
	(*datapoint).signedfields.gyroX = urocket_var.gyro[0];
	(*datapoint).signedfields.gyroY = urocket_var.gyro[1];
	(*datapoint).signedfields.gyroZ = urocket_var.gyro[2];
	(*datapoint).fields.timestamp=(uint32_t)urocket_var.timestamp;
	(*datapoint).fields.servo_state_0 = urocket_var.servo_time_0.flt;
	(*datapoint).fields.servo_state_1 = urocket_var.servo_time_1.flt;
}

void alt_inv_q_norm4(float *q)
{
    float mag;
    mag = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (mag) {
        q[0] /= mag;
        q[1] /= mag;
        q[2] /= mag;
        q[3] /= mag;
    } else {
        q[0] = 1.f;
        q[1] = 0.f;
        q[2] = 0.f;
        q[3] = 0.f;
    }
}
