import socket

serversocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP socket
serversocket.bind(('0.0.0.0', 5002))  # any interface

while True:
    # connection, address = serversocket.accept()
    buf = serversocket.recv(64)
    if len(buf) > 0:
        print(bin(ord(buf[0])) + " " + bin(ord(buf[1])))
