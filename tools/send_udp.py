# simple python script thats sends udp message with steering angle to truck maker
import socket

UDP_IP = "192.168.0.90"
UDP_PORT = 10002
steering_angle = 100
message ="STEER,1," + str(steering_angle)
MESSAGE = str.encode(message)


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

print("message sent: " + message)