from socket import*
serverPort = 5011
serverSocket = socket(AF_INET,SOCK_DGRAM)
serverSocket.bind(('192.168.0.203',serverPort))
print "The server is ready to receive"

while True:
	message, clientAddress = serverSocket.recvfrom(5021)
	print message
	print "next"







