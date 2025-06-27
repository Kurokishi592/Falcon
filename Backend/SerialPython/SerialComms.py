import serial
import serial.tools.list_ports


def dict_ports():
	port_dict = {}
	for comport in serial.tools.list_ports.comports():
		desc = comport.description
		idx = desc.find("(COM")			# Chop off the (COMx) at the end of the description
		desc = desc[:(idx - 1)]
		port_dict[comport.device] = desc
		print("{}: {}".format(comport.device, comport.description))
	return port_dict


def list_ports():
	ports = dict_ports()
	port_list = []
	for i in ports.keys():
		j = i + " " + ports[i]
		port_list.append(j)
	return port_list


def connect_serial(serial_port):
	ser = serial.Serial(serial_port, 115200)
	return ser


def read_serial(ser):
	data_dict = {}
	for i in range(4):
		line = ser.readline()
		if line:
			line = line.decode('utf-8').strip()
			print(line)
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
