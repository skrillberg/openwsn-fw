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

#define UROCKET_PERIOD_MS	100  //how often control task gets pushed to scheduler in ms
#define GYRO_FSR			2000 //gyro full scale range in deg/s
#define DATAPOINTS			100 //number of datapoints?
#define FLASH_PAGES_TOUSE	50
#define FLASH_PAGE_STORAGE_START 100 //first flash page to start at. TODO: make sure this doesn't overlap with code base
//========================== Global Variables =================================

urocket_vars_t urocket_vars;

//========================== Prototypes =======================================

void urocket_timer_cb(opentimers_id_t id);
void urocket_control_cb(void);
void printFlash(IMUDataCard * cards_stable, int page_struct_capacity);
void create_datapoint(urocket_vars_t urocket_var,IMUData* datapoint);
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
	//mimsyLedInit();
	//mimsyLedSet(GREEN_LED);

	//initialize trajectory
	urocket_vars.trajectory.roll[1]=3;
	urocket_vars.trajectory.sample_time = 1000; //1 second sample period for trajectory

    servo_init(3,20,1.45);

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
	//i should put these in the global struct
	IMUData datapoint;
	short sensors=INV_XYZ_GYRO | INV_WXYZ_QUAT|INV_XYZ_ACCEL;
	short more;
	//mimsyLedToggle(GREEN_LED);

	//read fifo from dmp. I currently have two reads in a row
	//because otherwise, every other read returns nothing

	dmp_read_fifo((urocket_vars.gyro), (urocket_vars.accel), (urocket_vars.quat),&(urocket_vars.timestamp), &sensors, &more);
	dmp_read_fifo((urocket_vars.gyro), (urocket_vars.accel), (urocket_vars.quat),&(urocket_vars.timestamp), &sensors, &more);
	mimsyPrintf("\n Clearing Fifo:%d,%d,%d,%d,%d,%d",urocket_vars.accel[0],urocket_vars.accel[1],urocket_vars.accel[2],urocket_vars.gyro[0],urocket_vars.gyro[1],urocket_vars.gyro[2]);

	create_datapoint(urocket_vars,&datapoint);	//convert dmp data into a flash-saveable format. this is inefficient at the moment



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

void urocket_sendpacket(short * accel,short accel_len, short * gyro,short gyro_len, long *quat,short quat_len){
	static const uint8_t uinject_payload[]    = "uinject";
	static const uint8_t uinject_dst_addr[]   = {
	   0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
	};
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
   memcpy(&pkt->l3_destinationAdd.addr_128b[0],uinject_dst_addr,16);

   // add payload
   packetfunctions_reserveHeaderSize(pkt,sizeof(uinject_payload)-1);
   memcpy(&pkt->payload[0],uinject_payload,sizeof(uinject_payload)-1);

   packetfunctions_reserveHeaderSize(pkt,sizeof(uint16_t));
   pkt->payload[1] = (uint8_t)((urocket_vars.counter & 0xff00)>>8);
   pkt->payload[0] = (uint8_t)(urocket_vars.counter & 0x00ff);
   urocket_vars.counter++;

   packetfunctions_reserveHeaderSize(pkt,sizeof(asn_t));
   ieee154e_getAsn(asnArray);
   pkt->payload[0] = asnArray[0];
   pkt->payload[1] = asnArray[1];
   pkt->payload[2] = asnArray[2];
   pkt->payload[3] = asnArray[3];
   pkt->payload[4] = asnArray[4];

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
	(*datapoint).fields.servo_state_0 = urocket_var.servo_time_0;
	(*datapoint).fields.servo_state_1 = urocket_var.servo_time_1;
}
