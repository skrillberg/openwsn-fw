
from socket import *
import struct 
import binascii 


ipv6_address = "bbbb::1"
# Create a UDP socket
# Ntice the use of SOCK_DGRAM for UDP packets
serverSocket = socket(AF_INET6, SOCK_DGRAM)

# Assign IP address and port number to socket
serverSocket.bind((ipv6_address, 2010))


# send request
serverSocket.sendto(struct.pack('<b',0),('bbbb:0:0:0:12:4b00:14d1:dc81'
, 2010))
serverSocket.sendto(struct.pack('<f',3.14159),('bbbb:0:0:0:12:4b00:14d1:dc81', 2010))

  
