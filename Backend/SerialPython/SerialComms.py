import serial
import serial.tools.list_ports


def dict_ports():
	"""
	Returns a dictionary of available serial ports with their descriptions
	:return: Dictionary with serial port names as keys and their descriptions as values
	"""
	port_dict = {"-1": "Close"}
	for comport in serial.tools.list_ports.comports():
		desc = comport.description
		idx = desc.find("(COM")			# Chop off the (COMx) at the end of the description, for Windows only
		desc = desc[:(idx - 1)]
		port_dict[comport.device] = desc
		print("{}: {}".format(comport.device, comport.description))
	return port_dict


def list_ports():
	"""
	Converts dictionary of serial ports to a list of serial ports (for GUI dropdown)
	:return: List of serial ports in the format "COMx Description"
	"""
	ports = dict_ports()
	port_list = []
	for i in ports.keys():
		j = i + " " + ports[i]
		port_list.append(j)
	return port_list


def connect_serial(serial_port):
	"""
	Connect to the specified serial port
	:param serial_port: Serial port to connect to (e.g., "COM9" for Windows)
	:return: Serial port object
	"""
	ser = serial.Serial(serial_port, 115200)
	return ser

def close_serial(serial_port):
	"""
	Close the serial port connection
	:param serial_port: Serial port object to close
	"""
	serial_port.close()


def read_serial(ser):
	"""
	Read data from the serial port and return a dictionary with Roll, Pitch, Yaw, and Temp values.
	:param ser: Serial port object
	:return: Dictionary with Roll, Pitch, Yaw, and Temp values
	"""
	data_dict = {"Roll": 0, "Pitch": 0, "Yaw": 0, "Temp": 0}
	for i in range(4):
		line = ser.readline()
		if line:
			line = line.decode('utf-8').strip()
			if line.startswith("Roll:"):
				data = line.split(":")[1].strip()
				data_dict["Roll"] = data
			if line.startswith("Pitch:"):
				data = line.split(":")[1].strip()
				data_dict["Pitch"] = data
			if line.startswith("Yaw:"):
				data = line.split(":")[1].strip()
				data_dict["Yaw"] = data
			if line.startswith("Temp:"):
				data = line.split(":")[1].strip()
				data_dict["Temp"] = data
	return data_dict


if __name__ == '__main__':
	ports_list = list_ports()
	print(ports_list)
	print(len(ports_list))
	ser1 = connect_serial("COM9")
	data1 = read_serial(ser1)
	ser1.close()
	print(data1)
