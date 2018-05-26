from socket import *
 
##UDP_IP = "127.0.0.1"
##UDP_PORT = 2010
##MESSAGE = "Hello, World!"
## 
##print "UDP target IP:", UDP_IP
##print "UDP target port:", UDP_PORT
##print "message:", MESSAGE
##sock = socket.socket(socket.AF_INET, # Internet
##                        socket.SOCK_DGRAM) # UDP
##sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
#sock.bind(('', UDP_PORT))



# Create a UDP socket
# Ntice the use of SOCK_DGRAM for UDP packets
serverSocket = socket(AF_INET, SOCK_DGRAM)

# Assign IP address and port number to socket
serverSocket.bind(('', 2010))

while True:

    # Receive the client packet along with the address it is coming from
    message, address = serverSocket.recvfrom(1024)
##while True:
##    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
##    print "received message:", data
