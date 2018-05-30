from socket import *
import struct 
import binascii 


ipv6_address = "bbbb::1"
# Create a UDP socket
# Ntice the use of SOCK_DGRAM for UDP packets
serverSocket = socket(AF_INET6, SOCK_DGRAM)

# Assign IP address and port number to socket
serverSocket.bind((ipv6_address, 2010))

while True:

    # Receive the client packet along with the address it is coming from
    message, address = serverSocket.recvfrom(1024)
    accel0  = struct.unpack('<h',message[0:2])  #this may need to be [0:1]
    accel1  = struct.unpack('<h',message[2:4])  #this may need to be [0:1]
    accel2  = struct.unpack('<h',message[4:6])  #this may need to be [0:1]
    roll = struct.unpack('<f',message[10:14])
    pitch = struct.unpack('<f',message[14:18])
    yaw = struct.unpack('<f',message[18:22])

    #print ""
    #print 'accel 0: "{0}", accel 1: "{1}", accel 2: "{2}" '.format(accel0,accel1,accel2)
    print 'roll: {:=6.4f}, pitch: {:6=.4f}, yaw: {:6=.4f} '.format(roll[0],pitch[0],yaw[0])
