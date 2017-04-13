import socket
import serial

UDP_IP = "127.0.0.1"
UDP_PORT = 31415

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=10)
time.sleep(2)

while True:
    data, addr = sock.recvfrom(1)
    ser.write(bytearray(data))
