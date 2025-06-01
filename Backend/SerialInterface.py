import serial

PORT_NAME = "/dev/tty/ACM0"
# PORT_NAME = "COM5"
BAUD_RATE = 115200
BYTE_SIZE = 8
PARITY = "N"
STOP_BITS = 1

mcu = serial.Serial(port=PORT_NAME, baudrate=BAUD_RATE, bytesize=BYTE_SIZE, parity=PARITY, stopbits=STOP_BITS)
mcu.close()
mcu.open()
print(f"Connected to MCU at port {PORT_NAME}")


def read_incoming():
	mcu.read()
