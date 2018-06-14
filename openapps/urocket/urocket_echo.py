x =[]	
from socket import *
import struct 
import binascii 
import keyboard
import numpy as np

def key_cb(e):
	#print e.name
	global x
	global serverSocket
	if e.name == 'a':
		print " fin left"
		serverSocket.sendto(struct.pack('<c','l'),('bbbb:0:0:0:12:4b00:14d1:dc81', 2010))

	if e.name == 'd':
		print " fin right"
		serverSocket.sendto(struct.pack('<c','r'),('bbbb:0:0:0:12:4b00:14d1:dc81', 2010))
	if e.name == 's':
		print " fin middle" 
		serverSocket.sendto(struct.pack('<c','m'),('bbbb:0:0:0:12:4b00:14d1:dc81', 2010))
	if e.name == '0':
		mode_message = struct.pack('<b',0)
		serverSocket.sendto(mode_message,('bbbb:0:0:0:12:4b00:14d1:dc81', 2010))
	if e.name =='g':
		print "begin logging"
		serverSocket.sendto(struct.pack('<c','g'),('bbbb:0:0:0:12:4b00:14d1:dc81', 2010))

	if e.name =='m':
		print "saved"
		np.savetxt('data.csv',np.array(x), delimiter=',')
		x=[]

	if e.name =='v':
		serverSocket.close()

ipv6_address = "bbbb::1"
# Create a UDP socket
# Ntice the use of SOCK_DGRAM for UDP packets
serverSocket = socket(AF_INET6, SOCK_DGRAM)

# Assign IP address and port number to socket
serverSocket.bind((ipv6_address, 2010))

# get user input on which mode to run the rocket in

mode=raw_input("Enter operating mode for urocket (0 for RC bypass mode, 1 for programmed trajectory mode) ")

while((mode != '0') and (mode != '1')):
	print "invalid input, must be 0 or 1"
	mode=raw_input("Enter operating mode for urocket (0 for RC bypass mode, 1 for programmed trajectory mode) ")
x=[]

mode_message = struct.pack('<b',int(mode))
serverSocket.sendto(mode_message,('bbbb:0:0:0:12:4b00:14d1:dc81'
, 2010))
keyboard.on_press(key_cb,suppress = True)
#run rc_bypass code
if mode == '0':
	while True:
		try:
			message, address = serverSocket.recvfrom(1024)
			if len(message) <= 31: #means it is not a data point
				print message
			else:
				accel0  = struct.unpack('<h',message[0:2])  #this may need to be [0:1]
				accel1  = struct.unpack('<h',message[2:4])  #this may need to be [0:1]
				accel2  = struct.unpack('<h',message[4:6])  #this may need to be [0:1]
				gyro0 = struct.unpack('<h',message[6:8]) 
				gyro1 = struct.unpack('<h',message[8:10]) 
				gyro2 = struct.unpack('<h',message[10:12]) 
				roll = struct.unpack('<f',message[12:16])
				pitch = struct.unpack('<f',message[16:20])
				yaw = struct.unpack('<f',message[20:24])
				servo0 = struct.unpack('<f',message[24:28])
				servo1 = struct.unpack('<f',message[28:32])

				#print ""
				#print 'accel 0: "{0}", accel 1: "{1}", accel 2: "{2}" '.format(accel0,accel1,accel2)
	
				print 'roll: {:=6.4f}, pitch: {:6=.4f}, yaw: {:6=.4f} '.format(roll[0],pitch[0],yaw[0])	
				print 'servo_0: {:=6.4f}, servo_1 {:=6.4f}'.format(servo0[0],servo1[0])
				x.append([accel0[0],accel1[0],accel2[0],gyro0[0],gyro1[0],gyro2[0],roll[0],pitch[0],yaw[0],servo0[0],servo1[0]])	

		except: 
			print "receive failed"

#receive input on which 

# send request
serverSocket.sendto(struct.pack('<b',0),('bbbb:0:0:0:12:4b00:14d1:dc81'
, 2010))
serverSocket.sendto(struct.pack('<f',3.14159),('bbbb:0:0:0:12:4b00:14d1:dc81', 2010))


