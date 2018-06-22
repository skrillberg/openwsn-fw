
from socket import *
import struct 
import binascii 
import keyboard
import numpy as np

def key_cb(e):
	#print e.name

	global serverSocket
	if e.name == 'i':
		print "data burst initiated"
		serverSocket.sendto(struct.pack('<c','i'),('bbbb:0:0:0:12:4b00:14d1:dcb9', 2018))

	if e.name =='v':
		serverSocket.close()

ipv6_address = "bbbb::1"
# Create a UDP socket
# Ntice the use of SOCK_DGRAM for UDP packets
serverSocket = socket(AF_INET6, SOCK_DGRAM)

# Assign IP address and port number to socket
serverSocket.bind((ipv6_address, 2018))

serverSocket.sendto(struct.pack('<c','i'),('bbbb:0:0:0:12:4b00:14d1:db71', 2018))
#assign keyboard press callback function
keyboard.on_press(key_cb)
serverSocket.sendto(struct.pack('<c','i'),('bbbb:0:0:0:12:4b00:14d1:db71', 2018))
while True:
	try:
		message, address = serverSocket.recvfrom(1024)
		
		#unpack with struct.unpack
		if len(message) <= 20: #means it is not a data point
			print message
		else:
			accel0  = struct.unpack('<h',message[0:2])  
			accel1  = struct.unpack('<h',message[2:4]) 
			accel2  = struct.unpack('<h',message[4:6])  
			gyro0 = struct.unpack('<h',message[6:8]) 
			gyro1 = struct.unpack('<h',message[8:10]) 
			gyro2 = struct.unpack('<h',message[10:12]) 
			roll = struct.unpack('<f',message[12:16])
			pitch = struct.unpack('<f',message[16:20])
			yaw = struct.unpack('<f',message[20:24])



	
		print 'roll: {:=6.4f}, pitch: {:6=.4f}, yaw: {:6=.4f} '.format(roll[0],pitch[0],yaw[0])	
		print 'servo_0: {:=6.4f}, servo_1 {:=6.4f}'.format(servo0[0],servo1[0])
		x.append([accel0[0],accel1[0],accel2[0],gyro0[0],gyro1[0],gyro2[0],roll[0],pitch[0],yaw[0]])	

	except: 
		print "receive failed"



