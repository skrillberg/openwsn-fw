from socket import *
import struct 
import binascii 
import keyboard
import numpy as np

#called when a key is pressed
def key_cb(e):
	#print e.name
	global serverSocket
	if e.name == 'i':
		print "data burst command sent"
                #INSERT SOCKET UDP SEND CODE HERE


	if e.name =='v':
		serverSocket.close()


#assign keyboard press callback function
keyboard.on_press(key_cb,suppress = True)


####################################################
#
#socket intialization code here
#
####################################################


while True:
	
	try:
		#place UDP receive code here
	except:
		print "receive failed"
