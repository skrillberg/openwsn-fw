/**
\brief Python-specific definition of the "board" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, May 2013.
*/

#include <stdio.h>
#include "board_obj.h"
// bsp modules
#include "debugpins_obj.h"
#include "leds_obj.h"
#include "uart_obj.h"
#include "radio_obj.h"
#include "eui64_obj.h"
#include "sctimer_obj.h"
#include "gazebo_obj.h"


//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

void board_init(OpenMote* self) {
   PyObject*   result;
   
#ifdef TRACE_ON
   printf("C@0x%x: board_init()...\n",self);
#endif
   
   // initialize bsp modules
   debugpins_init(self);
   leds_init(self);
   sctimer_init(self);
   uart_init(self);
   radio_init(self);
   
   // forward to Python
   result     = PyObject_CallObject(self->callback[MOTE_NOTIF_board_init],NULL);
   if (result == NULL) {
      printf("[CRITICAL] board_init() returned NULL\r\n");
      return;
   }
   Py_DECREF(result);
   
#ifdef TRACE_ON
   printf("C@0x%x: ...done.\n",self);
#endif
}

void board_sleep(OpenMote* self) {
   PyObject*   result;
   
#ifdef TRACE_ON
   printf("C@0x%x: board_sleep()... \n",self);
#endif
   
   // forward to Python
   result     = PyObject_CallObject(self->callback[MOTE_NOTIF_board_sleep],NULL);
   if (result == NULL) {
      printf("[CRITICAL] board_sleep() returned NULL\r\n");
      return;
   }
   Py_DECREF(result);
   
#ifdef TRACE_ON
   printf("C@0x%x: ...done.\n",self);
#endif
}

void board_reset(OpenMote* self) {
   PyObject*   result;
   
#ifdef TRACE_ON
   printf("C@0x%x: board_reset()... \n",self);
#endif
   
   // forward to Python
   result     = PyObject_CallObject(self->callback[MOTE_NOTIF_board_reset],NULL);
   if (result == NULL) {
      printf("[CRITICAL] board_reset() returned NULL\r\n");
      return;
   }
   Py_DECREF(result);
   
#ifdef TRACE_ON
   printf("C@0x%x: ...done.\n",self);
#endif
}

void board_get_location(OpenMote* self,int16_t* x, int16_t* y, int16_t* z,int16_t** neighbor_list, uint8_t num_neighbors){
	PyObject*  result;
	PyObject*  item;
	PyObject*  neighbors;
	int16_t* neighbor_row;
	int16_t i =0;
	//printf("calling PyObject_CallObject in board_obj.c\n");

 	result = PyObject_CallObject(self->callback[MOTE_NOTIF_board_get_location],NULL);
	item       = PyTuple_GetItem(result,0);
	
	*x = (int16_t)PyInt_AsLong(item);
	//printf("x: %d ",*x);

	item       = PyTuple_GetItem(result,1);
	*y = (int16_t)PyInt_AsLong(item);
	//printf("y: %d ",*y);

	item       = PyTuple_GetItem(result,2);
	*z = (int16_t)PyInt_AsLong(item);
	//printf("z: %d \n",*z);
	
	neighbors = PyTuple_GetItem(result,3);
	//printf("neighbors size: %d\n",(int)PyList_Size(neighbors));

	for(i=0;i<(int)PyList_Size(neighbors);i++){
		PyObject* coordinates = PyList_GetItem(neighbors,i);
		//printf("neuighbor list address, %x \n",neighbor_list[i]);
		neighbor_row = neighbor_list[i];
		*(neighbor_row)=(int16_t)PyInt_AsLong(PyList_GetItem(coordinates,0));
		*(neighbor_row+1)=(int16_t)PyInt_AsLong(PyList_GetItem(coordinates,1));
		*(neighbor_row+2)=(int16_t)PyInt_AsLong(PyList_GetItem(coordinates,2));
		//printf("mote %d: row addresses:%x,  %d, %d, %d \n",i+1,neighbor_row,neighbor_row[0],neighbor_row[1],neighbor_row[2]);
	}	
	
	//printf("successfully called PyObject_CallObject\n");	
}
void board_cmd_vel(OpenMote* self,float x_cmd, float y_cmd, float z_cmd){
   	PyObject*   result;
   	PyObject*   arglist;
	arglist    = Py_BuildValue("(f,f,f)",x_cmd,y_cmd,z_cmd);
	result     = PyObject_CallObject(self->callback[MOTE_NOTIF_board_cmd_vel],arglist);

}
//=========================== private =========================================
