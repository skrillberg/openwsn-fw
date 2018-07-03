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

//=========================== defines =========================================

const uint8_t cinfo_path0[] = "i";
#define CINFO_PERIOD_MS 1000
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
	 uint8_t bytes[3];
	 bytes[0] = rand()%200;
	 bytes[1] = rand()%200;
	 bytes[2] = rand()%200;
	 uart_enableInterrupts();
         openserial_vars.mode=MODE_INPUT;
	 uart_writeBufferByLen_FASTSIM(bytes,3);
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
		cinfo_vars.rx_ready=0;
                printf("uart rx: %d, %d, %d, %d \n",cinfo_vars.rx_buf[0],cinfo_vars.rx_buf[1],cinfo_vars.rx_buf[2],cinfo_vars.rx_buf[3]); 
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
		printf("uart rx: %d, %d, %d, %d \n",cinfo_vars.rx_buf[0],cinfo_vars.rx_buf[1],cinfo_vars.rx_buf[2],cinfo_vars.rx_buf[3]);
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


