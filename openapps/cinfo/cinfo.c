/**
\brief A CoAP resource which indicates the board its running on.
*/

#include "opendefs.h"
#include "cinfo.h"
#include "opencoap.h"
#include "openqueue.h"
#include "packetfunctions.h"
#include "openserial.h"
#include "openrandom.h"
#include "board.h"
#include "idmanager.h"
#include "uart_obj.h"
#include "opentimers.h"
#include <stdlib.h>
#include <math.h>
#include "neighbors.h"
#include "idmanager.h"
#include "opendefs.h"
//=========================== defines =========================================

const uint8_t cinfo_path0[] = "i";
#define CINFO_PERIOD_MS 150
//=========================== variables =======================================

cinfo_vars_t cinfo_vars;

//=========================== prototypes ======================================

owerror_t     cinfo_receive(
        OpenQueueEntry_t* msg,
        coap_header_iht*  coap_header,
        coap_option_iht*  coap_incomingOptions,
        coap_option_iht*  coap_outgoingOptions,
        uint8_t*          coap_outgoingOptionsLen
);
void          cinfo_sendDone(
   OpenQueueEntry_t* msg,
   owerror_t error
);
void cinfo_timer_cb(opentimers_id_t id);
void txCb(void);
void rxCb(void);
//=========================== public ==========================================

/**
\brief Initialize this module.
*/
void cinfo_init(void) {
   // do not run if DAGroot
   if(idmanager_getIsDAGroot()==TRUE) return; 
   if(idmanager_getIsDAGroot()==FALSE){
   //uart_setCallbacks( txCb, rxCb);
   //uart_enableInterrupts();
   }
   cinfo_vars.time = 0;
   cinfo_vars.listening = 0;
   cinfo_vars.frame_start=0;
   cinfo_vars.byte_count=0;
   cinfo_vars.rx_ready=0;
   // prepare the resource descriptor for the /i path
   cinfo_vars.desc.path0len             = sizeof(cinfo_path0)-1;
   cinfo_vars.desc.path0val             = (uint8_t*)(&cinfo_path0);
   cinfo_vars.desc.path1len             = 0;
   cinfo_vars.desc.path1val             = NULL;
   cinfo_vars.desc.componentID          = COMPONENT_CINFO;
   cinfo_vars.desc.securityContext      = NULL;
   cinfo_vars.desc.discoverable         = TRUE;
   cinfo_vars.desc.callbackRx           = &cinfo_receive;
   cinfo_vars.desc.callbackSendDone     = &cinfo_sendDone;
   
   // register with the CoAP module
   opencoap_register(&cinfo_vars.desc);

   // start timer for requesting data from ROS Simulator
    cinfo_vars.period = CINFO_PERIOD_MS;
	cinfo_vars.timerId = opentimers_create();
	opentimers_scheduleIn(
		cinfo_vars.timerId,
		CINFO_PERIOD_MS,
		TIME_MS,
		TIMER_PERIODIC,
		cinfo_timer_cb
	);
}

//=========================== private =========================================
//this function request data updates from the simulator. Data will come in TODO structure
void cinfo_timer_cb(opentimers_id_t id){

	 uint8_t i;
	 i = 0;
   

   	 for (i=0;i<MAXNUMNEIGHBORS;i++) {
      		if (neighbors_vars.neighbors[i].used==TRUE) {
        		 //printf("Me: %d, Neighbor Address: %x \n",(uint8_t)(idmanager_getMyID(ADDR_64B)->addr_64b[7]),neighbors_vars.neighbors[i].addr_64b.addr_64b[7]);
     		 }
  	 }
	 //printf("\n");
	 cinfo_vars.time += (float)(CINFO_PERIOD_MS/1000.0);
	 floatbyte_t controls[3];
	 if(((uint8_t)(idmanager_getMyID(ADDR_64B)->addr_64b[7]) == 6)&&(cinfo_vars.time > 5)){
		controls[0].flt = (float) 5 + cinfo_vars.controls[0].flt;
		controls[1].flt	= 0 + cinfo_vars.controls[1].flt  ;
		printf("hi, i'm mote 6. Here are my controls: %f, %f \n",controls[0].flt,controls[1].flt);
	 }else{
	    	 //controls[0].flt = (float)rand()/(float)(RAND_MAX/1.0)-0.5;
	     	//controls[1].flt = (float)rand()/(float)(RAND_MAX/1.0)-0.5;
	controls[0].flt = cinfo_vars.controls[0].flt;
	 controls[1].flt = cinfo_vars.controls[1].flt;
	 }
	 //printf("time: %f \n",cinfo_vars.time);
	 //controls[0].flt = sinf(cinfo_vars.time)*0.25;
	 //controls[1].flt = cosf(cinfo_vars.time)*0.25;
	 //controls[2].flt = sinf(cinfo_vars.time)*3;

	 controls[2].flt = cinfo_vars.controls[2].flt;
       	 shortbyte_t accelx;
	 shortbyte_t accely;
	 shortbyte_t accelz;
	 floatbyte_t position[3];
	 floatbyte_t velocity[3];
	 floatbyte_t angular[3];
	
	 uint8_t control_buf[3*4];
	int j;
	j=0;
	i=0;
		
	 //create buffer to send controls data out
	 for(i =0; i<3;i++){
		for( j =0; j <4 ; j++){
			control_buf[4*i+j] = controls[i].bytes[j];

		}
	 	
	 }
	 //printf("controls: %f, %f, %f,%d \n",controls[0].flt,controls[1].flt,controls[2].flt,sizeof(controls[0].flt));
	 //printf("control bytes: %x, %x, %x,%x \n",controls[0].bytes[0],controls[0].bytes[1],controls[0].bytes[2],controls[0].bytes[3],sizeof(controls[0].flt));
	 //printf("control buffer: %x, %x, %x, %x \n",control_buf[0],control_buf[1],control_buf[2],control_buf[3]);
	 uart_enableInterrupts();
         openserial_vars.mode=MODE_INPUT;
	 uart_writeBufferByLen_FASTSIM(control_buf,12);
	 cinfo_vars.listening = 1;
	 //openserial_startInput();
	 //openserial_getInputBuffer(bytes, 3);
	/*
	 bytes[0] = uart_readByte();
	 bytes[1] = uart_readByte();
	 bytes[2] = uart_readByte();
	 
        // cinfo_vars.accelx = bytes[0];
	 //cinfo_vars.accely = bytes[1];
        // cinfo_vars.accelz = bytes[2];
	 //printf("hi"); */
	 if(cinfo_vars.rx_ready == 1){
		int isSix=0;
		 if(((uint8_t)(idmanager_getMyID(ADDR_64B)->addr_64b[7])) == 6){
			isSix=1;
		}
		cinfo_vars.rx_ready=0;
                //printf("uart rx: %d, %d, %d, %d,%d,%d,%d \n",cinfo_vars.rx_buf[0],cinfo_vars.rx_buf[1],cinfo_vars.rx_buf[2],cinfo_vars.rx_buf[3],cinfo_vars.rx_buf[4],cinfo_vars.rx_buf[5],cinfo_vars.rx_buf[6]); 
		accelx.bytes[0] = cinfo_vars.rx_buf[1];
		accelx.bytes[1] = cinfo_vars.rx_buf[2];

		accely.bytes[0] = cinfo_vars.rx_buf[3];
		accely.bytes[1] = cinfo_vars.rx_buf[4];

		accelz.bytes[0] = cinfo_vars.rx_buf[5];
		accelz.bytes[1] = cinfo_vars.rx_buf[6];
		int buf_start_value = 7;

		//iterate through all bytes to get position from buffer
		for(i =0;i<3;i++){
			for(int j = 0; j<4;j++){
				position[i].bytes[j] = cinfo_vars.rx_buf[buf_start_value+i*4+j];
			}
		}
		//printf("position (x: %f, y: %f, z: %f) \n",position[0].flt,position[1].flt,position[2].flt);
		
		if( position[2].flt<4){
			cinfo_vars.controls[2].flt=2;
		}else{
			cinfo_vars.controls[2].flt=0;
		}
		buf_start_value = 19;
		//5 neighbors
		floatbyte_t neighbors[30][3];
		//printf("size of buf: %d\n",sizeof(neighbors));
		int neighbor = 0;
		int coord = 0;
		int byte_i = 0;
		
		for(neighbor = 0;neighbor<10;neighbor++){
			for(int coord = 0; coord<3;coord++){
				for(byte_i = 0; byte_i<4;byte_i++){
					neighbors[neighbor][coord].bytes[byte_i] = cinfo_vars.rx_buf[buf_start_value+neighbor*12+coord*4+byte_i];
					//printf("(i: %d, j: %d, k: %d, rxbuf index: %d) \n",neighbor,coord,byte_i,buf_start_value+neighbor*12+coord*4+byte_i);
					//printf("buf: %d)\n",cinfo_vars.rx_buf[buf_start_value+neighbor*12+coord*4+byte_i]);
				}
			}
			//printf("neighbor (x: %f, y: %f, z: %f) \n",neighbors[neighbor][0].flt,neighbors[neighbor][1].flt,neighbors[neighbor][2].flt);
		}
		

		//compute gradients
		i=0;
		j=0;
		float R = 50;
		floatbyte_t sum[3]; //[x gradient sum, y gradient sum, z gradient sum]

		sum[0].flt = 0;
		sum[1].flt=0;
		sum[2].flt=0;
		float kconn=.01;
		float kcol=0.011;
		float sig = 10;
		for(i=0;i<10;i++){
			float distance =pow((position[0].flt - neighbors[i][0].flt),2) + pow((position[1].flt - neighbors[i][1].flt),2) +pow((position[2].flt - neighbors[i][2].flt),2);	
			for(j=0;j<3;j++){
				float coord_dif = position[j].flt - neighbors[i][j].flt;
						
				float prox_grad = -2*coord_dif/pow(distance,2);
				if(isSix){
					//printf("distance squared: %f, coord dif: %f, proximity grad: %f \n",distance,coord_dif, prox_grad);
				}
				
				if(distance>0){
				//sum[j].flt += prox_grad + k*2*(coord_dif)/pow((R*R -distance),2); //1/r-xij
				sum[j].flt += -kcol*2*(coord_dif)*exp(-(distance)/(2*R*R*sig)) + kconn*2*(coord_dif)*exp((distance)/(2*R*R*sig));
			}	}

			if(isSix){
			//printf("denominator not squared: %f \n",R*R-distance);
			}
			
		}
		if(isSix){
			printf("gradient: %f, %f, %f \n",sum[0].flt,sum[1].flt,sum[2].flt);
			printf("\n");
		}
		
		cinfo_vars.controls[0].flt=-1*sum[0].flt;
		cinfo_vars.controls[1].flt=-1*sum[1].flt;
		//printf("My Address: %d, Num of Neighbors: %d \n",(uint8_t)(idmanager_getMyID(ADDR_64B)->addr_64b[7]),neighbors_getNumNeighbors());
		//printf("Accelerations (x: %f, y: %f, z: %f) \n",((float)accelx.shrt)*9.8/36767*16,((float)accely.shrt)*9.8/36767*16,((float)accelz.shrt)*9.8/36767*16);
	}
  	
		
}

void txCb(void){
	//printf("uart tx interrupt \n");
}
void rxCb(void){

	uint8_t byte;
	byte = uart_readByte();

	//if the starting flag is received, tell the app that a frame has started
	if(cinfo_vars.frame_start == 0){
 		if(byte == 126){
			cinfo_vars.frame_start = 1;
  			cinfo_vars.byte_count = 0;
		}
	}
	else if(cinfo_vars.frame_start == 1 && (byte != 126) ){
		//first byte in buffer says how long the packet is
		cinfo_vars.rx_buf[cinfo_vars.byte_count+1] = byte;
		cinfo_vars.byte_count++;
	}else{
		//this occurs when a another frame byte shows up
		cinfo_vars.rx_buf[0] = cinfo_vars.byte_count;
		cinfo_vars.frame_start = 0;
		//printf("uart rx: %d, %d, %d, %d,%d,%d,%d \n",cinfo_vars.rx_buf[0],cinfo_vars.rx_buf[1],cinfo_vars.rx_buf[2],cinfo_vars.rx_buf[3],cinfo_vars.rx_buf[4],cinfo_vars.rx_buf[5],cinfo_vars.rx_buf[6]);
	}
	
	//printf("uart rx interrupt: %d \n",byte);
	
}

/**
\brief Called when a CoAP message is received for this resource.

\param[in] msg          The received message. CoAP header and options already
   parsed.
\param[in] coap_header  The CoAP header contained in the message.
\param[in] coap_options The CoAP options contained in the message.

\return Whether the response is prepared successfully.
*/
owerror_t cinfo_receive(
        OpenQueueEntry_t* msg,
        coap_header_iht*  coap_header,
        coap_option_iht*  coap_incomingOptions,
        coap_option_iht*  coap_outgoingOptions,
        uint8_t*          coap_outgoingOptionsLen
) {
   owerror_t outcome;
   
   switch (coap_header->Code) {
      case COAP_CODE_REQ_GET:
         //=== reset packet payload (we will reuse this packetBuffer)
         msg->payload                     = &(msg->packet[127]);
         msg->length                      = 0;
         
         //=== prepare  CoAP response
         
         // radio name
         packetfunctions_reserveHeaderSize(msg,sizeof(infoRadioName)-1);
         memcpy(&msg->payload[0],&infoRadioName,sizeof(infoRadioName)-1);
         
         // uC name
         packetfunctions_reserveHeaderSize(msg,1);
         msg->payload[0] = '\n';
         packetfunctions_reserveHeaderSize(msg,sizeof(infouCName)-1);
         memcpy(&msg->payload[0],&infouCName,sizeof(infouCName)-1);
         
         // board name
         packetfunctions_reserveHeaderSize(msg,1);
         msg->payload[0] = '\n';
         packetfunctions_reserveHeaderSize(msg,sizeof(infoBoardname)-1);
         memcpy(&msg->payload[0],&infoBoardname,sizeof(infoBoardname)-1);
         
         // stack name and version
         packetfunctions_reserveHeaderSize(msg,1);
         msg->payload[0] = '\n';
         packetfunctions_reserveHeaderSize(msg,sizeof(infoStackName)-1+6);
         memcpy(&msg->payload[0],&infoStackName,sizeof(infoStackName)-1);
         msg->payload[sizeof(infoStackName)-1+6-6] = '0'+OPENWSN_VERSION_MAJOR;
         msg->payload[sizeof(infoStackName)-1+6-5] = (uint8_t)(cinfo_vars.accely);
         msg->payload[sizeof(infoStackName)-1+6-4] = '0'+OPENWSN_VERSION_MINOR / 10;
         msg->payload[sizeof(infoStackName)-1+6-3] = '0'+OPENWSN_VERSION_MINOR % 10;
         msg->payload[sizeof(infoStackName)-1+6-2] = '.';
         msg->payload[sizeof(infoStackName)-1+6-1] = '0'+OPENWSN_VERSION_PATCH;
         
         // set the CoAP header
         coap_header->Code                = COAP_CODE_RESP_CONTENT;
        //uart_writeByte(0x34);
	// uint8_t bytes[2];
	 //bytes[0] = 0x34;
	 //bytes[1] = 0x45;
	// uart_writeBufferByLen_FASTSIM(bytes,2);
	// bytes[0] = uart_readByte();
	 //printf("hi");
	// printf("bytes: %x",bytes[0]);
         outcome                          = E_SUCCESS;
         break;
      default:
         // return an error message
         outcome = E_FAIL;
   }
   
   return outcome;
}

/**
\brief The stack indicates that the packet was sent.

\param[in] msg The CoAP message just sent.
\param[in] error The outcome of sending it.
*/
void cinfo_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}


