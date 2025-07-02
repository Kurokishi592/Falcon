import tkinter as tk
from tkinter import ttk
from cv2_enumerate_cameras import enumerate_cameras
import cv2
from PIL import Image, ImageTk
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../..")
from Backend.SerialPython import SerialComms
from Backend.AprilTagDetection import AprilTagDetector 	# Importing the AprilTag detector class from the Backend module
from Backend.Controls1 import FalconController
import numpy as np


class Monitor:
	def __init__(self, window):
		self.window = window

		# Camera instance stuff
		self.vid = None
		self.use_cam = None
		self.num_cams = 0

		# Camera frame stuff
		self.cam_frame = None
		self.width, self.height = 600, 400

		# Camera select stuff
		self.dict_cams = {}
		self.cam_dropdown = None
		self.cam_select = None
		self.cam_refresh = None

		# Serial Port select stuff
		self.port_dropdown = None
		self.port_select = None
		self.port_refresh = None
		self.teensy_serial = None
		self.imu_data = {}

		# AprilTag stuff
		self.detected_message_label = None
		self.detected_message_data = "Nothing"
		self.apriltag_detector = AprilTagDetector()

		self.velocity_estimator = FalconController()

		# Control buttons
		self.start = None
		self.failsafe = None

		# PID stuff
		self.param_p_f = None
		self.param_i_f = None
		self.param_d_f = None
		self.param_pid_button = None
		self.param_pid_pass_fail = None
		self.param_p = 0
		self.param_i = 0
		self.param_d = 0

		# +/- buttons
		self.param_p_p = None
		self.param_p_m = None
		self.param_i_p = None
		self.param_i_m = None
		self.param_d_p = None
		self.param_d_m = None

		# Marker loss timeout stuff
		self.timeout = None
		self.timeout_button = None

		# Velocity stuff
		self.x_label = None
		self.y_label = None
		self.z_label = None

		# Parameter labels
		self.roll_label = None
		self.pitch_label = None
		self.yaw_label = None
		self.cam_fps_label = None
		self.wind_spd_label = None
		self.wind_dir_label = None
		self.batt_vol_label = None
		self.int_temp_label = None

		self.find_cams()
		if self.num_cams == 1:
			self.cam_start(list(self.dict_cams)[0])
		self.create_gui()

	def find_cams(self):
		print("List of cameras detected: ")
		self.dict_cams = check_cams()
		self.num_cams = len(self.dict_cams) - 1
		print(f'Number of cameras detected: {self.num_cams}')

	def cam_start(self, use_cam):
		if self.use_cam != use_cam:
			self.use_cam = use_cam
			print(f'Using camera: {self.use_cam}')
			if self.vid is not None:
				self.vid.release()
			self.vid = cv2.VideoCapture(self.use_cam)
			self.vid.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
			self.vid.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
			self.cam_feed()

	def cam_stop(self):
		self.use_cam = None
		if self.vid is not None:
			self.vid.release()
			self.vid = None
			self.cam_frame.photo_image = None

	def cam_feed(self):
		if self.vid is not None:
			ret, frame = self.vid.read()
			if not ret:
				self.cam_frame.after(10, self.cam_feed)
				return

			# Process the frame for AprilTag detection
			self.apriltag_detector.cx = self.width / 2
			self.apriltag_detector.cy = self.height / 2	

			# Run detection
			detections = self.apriltag_detector.detect(frame)

			# Draw detections on the frame
			for det in detections:
				corners = np.int32(det.corners)
				cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
				cv2.putText(frame, f"ID: {det.tag_id}", tuple(np.int32(det.center)),
							cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

			# Display detections in label
			if detections:
				message = f"Detected {len(detections)} tag(s): " + ', '.join(str(d.tag_id) for d in detections)
				self.detected_message_label.config(text=message)
				print(message)

				# Adding velocity estimation
				det = detections[0] if isinstance(detections, list) else detections				# If detections is a list, take first
				self.velocity_estimator.new_detection(det)
				temp_velocity = self.velocity_estimator.get_velocity()
				self.x_label.config(text=str(round(temp_velocity[0], 3)) + " m/s")
				self.y_label.config(text=str(round(temp_velocity[1], 3)) + " m/s")
				self.z_label.config(text=str(round(temp_velocity[2], 3)) + " m/s")
			else:
				self.detected_message_label.config(text="No tags detected")

			# Convert the frame to RGB format for Tkinter
			# _, frame = self.vid.read()
			raw_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
			conv_image = Image.fromarray(raw_image)
			photo_image = ImageTk.PhotoImage(image=conv_image)
			self.cam_frame.photo_image = photo_image
			self.cam_frame.configure(image=photo_image)
			
			self.cam_frame.after(10, self.cam_feed)
		else:
			print("No camera selected or camera feed stopped.")

	def cam_selection(self):
		use_cam = self.cam_dropdown.get()		# Returned in integer: string form
		print(use_cam)
		if use_cam != "-1: Off":
			counter = 0
			str_cam = ""
			while use_cam[counter].isdigit():
				str_cam += use_cam[counter]
				counter += 1
			self.cam_start(use_cam=int(str_cam))
		else:
			self.cam_stop()

	def refresh_cam(self):
		self.find_cams()
		list_cams = [str(key) + ": " + value for key, value in self.dict_cams.items()]
		self.cam_dropdown["values"] = list_cams

	def port_selection(self):
		use_port = self.port_dropdown.get()		# Returned in string form
		print(use_port)
		if use_port == "-1 Close" and self.teensy_serial is None:
			print("No serial port to close")
		elif use_port == "-1 Close" and self.teensy_serial is not None:
			print("Closing serial port")
			self.teensy_serial.close()									# Close the serial port
			self.teensy_serial = None									# Set the serial port to None
		elif use_port != "" and self.teensy_serial is None:
			idx = use_port.find(" ")
			use_port = use_port[:idx]									# Chop off the description
			print(f'Selected serial port: {use_port}')
			self.teensy_serial = SerialComms.connect_serial(use_port)	# Connect to serial port, e.g. "COM9"
			self.get_serial_data()										# Get data from the Teensy, and assign to labels
		elif use_port != "" and self.teensy_serial is not None:			# For users to change serial port
			idx = use_port.find(" ")
			use_port = use_port[:idx]
			print(f'Selected serial port: {use_port}')
			self.teensy_serial.close()
			self.teensy_serial = SerialComms.connect_serial(use_port)
			self.get_serial_data()
		else:
			print("No serial port selected")

	def refresh_ports(self):
		self.port_dropdown["values"] = SerialComms.list_ports()

	def get_serial_data(self):
		if self.teensy_serial is not None:
			# Get data from IMU, in dictionary form
			self.imu_data = SerialComms.read_serial(self.teensy_serial)
			# Assign data from the dictionary to the labels
			self.roll_label.config(text=self.imu_data["Roll"] + "°")
			self.pitch_label.config(text=self.imu_data["Pitch"] + "°")
			self.yaw_label.config(text=self.imu_data["Yaw"] + "°")
			self.int_temp_label.config(text=self.imu_data["Temp"] + "°C")

			# Can choose any label, check every 1s
			self.roll_label.after(1000, self.get_serial_data)

	def get_pid_params(self):
		p = self.param_p_f.get()
		i = self.param_i_f.get()
		d = self.param_d_f.get()
		print(f"P {p}, I {i}, D {d}")
		try:
			if float(p) >= 0 and float(i) >= 0 and float(d) >= 0:
				self.param_p = p
				self.param_i = i
				self.param_d = d
				self.param_pid_pass_fail.config(text="Success!")
				print("Successfully uploaded PID values")
		except ValueError:
			self.param_pid_pass_fail.config(text="Non-integer")

	def pid_param_p_plus(self):
		curr = self.param_p_f.get()
		try:
			if curr != "":
				self.param_pid_pass_fail.config(text="")
				self.param_p_f.delete(0, "end")
				self.param_p_f.insert(0, str(round(float(curr) + 0.1, 3)))
		except ValueError:
			self.param_pid_pass_fail.config(text="Non-integer")

	def pid_param_p_minus(self):
		curr = self.param_p_f.get()
		try:
			if curr != "" and float(curr) > 0:
				self.param_pid_pass_fail.config(text="")
				self.param_p_f.delete(0, "end")
				self.param_p_f.insert(0, str(round(float(curr) - 0.1, 3)))
		except ValueError:
			self.param_pid_pass_fail.config(text="Non-integer")

	def pid_param_i_plus(self):
		curr = self.param_i_f.get()
		try:
			if curr != "":
				self.param_pid_pass_fail.config(text="")
				self.param_i_f.delete(0, "end")
				self.param_i_f.insert(0, str(round(float(curr) + 0.001, 3)))
		except ValueError:
			self.param_pid_pass_fail.config(text="Non-integer")

	def pid_param_i_minus(self):
		curr = self.param_i_f.get()
		try:
			if curr != "" and float(curr) > 0:
				self.param_pid_pass_fail.config(text="")
				self.param_i_f.delete(0, "end")
				self.param_i_f.insert(0, str(round(float(curr) - 0.001, 3)))
		except ValueError:
			self.param_pid_pass_fail.config(text="Non-integer")

	def pid_param_d_plus(self):
		curr = self.param_d_f.get()
		try:
			if curr != "":
				self.param_pid_pass_fail.config(text="")
				self.param_d_f.delete(0, "end")
				self.param_d_f.insert(0, str(round(float(curr) + 0.01, 3)))
		except ValueError:
			self.param_pid_pass_fail.config(text="Non-integer")

	def pid_param_d_minus(self):
		curr = self.param_d_f.get()
		try:
			if curr != "" and int(curr) > 0:
				self.param_pid_pass_fail.config(text="")
				self.param_d_f.delete(0, "end")
				self.param_d_f.insert(0, str(round(float(curr) - 0.01, 3)))
		except ValueError:
			self.param_pid_pass_fail.config(text="Non-integer")

	def create_gui(self):
		self.window.title("Falcon Parameter Screen")

		####################
		# Camera Frame #
		####################

		# Large frame on the left for showing live camera feed
		camera_frame = ttk.LabelFrame(master=self.window, text="Camera View")
		# Grid the camera frame
		camera_frame.grid(column=0, row=0, padx=10, pady=10, sticky="new")
		camera_frame.columnconfigure(0, weight=1, minsize=300)
		camera_frame.rowconfigure(0, weight=1, minsize=400)

		# Live camera feed
		self.cam_frame = ttk.Label(master=camera_frame, text="Multiple cameras detected, use the dropdown to select")
		self.cam_frame.grid(column=0, row=0, padx=20, pady=20, sticky="")

		####################
		# Detection Frame #
		####################

		# Frame for settings and detected message
		detection_frame = ttk.LabelFrame(master=self.window, text="Settings")
		# Grid the detection frame
		detection_frame.grid(column=0, row=1, padx=10, pady=10, sticky="ew")
		detection_frame.columnconfigure((0, 1), weight=1)
		detection_frame.rowconfigure((0, 1), weight=1)

		detection_label = ttk.Label(master=detection_frame, text="Detected AprilTag message")
		detection_label.grid(column=0, row=0, padx=5, pady=5, sticky="n")

		self.detected_message_label = ttk.Label(master=detection_frame, text=self.detected_message_data)
		self.detected_message_label.grid(column=0, row=1, rowspan=2, padx=5, pady=5, sticky="ns")

		cam_select_label = ttk.Label(master=detection_frame, text="Select Camera")
		cam_select_label.grid(column=1, row=0, padx=5, pady=5, sticky="n")

		list_cams = [str(key) + ": " + value for key, value in self.dict_cams.items()]
		self.cam_dropdown = ttk.Combobox(master=detection_frame, values=list_cams)
		self.cam_dropdown.current(0)
		self.cam_dropdown.grid(column=1, row=1, padx=5, pady=5, sticky="n")

		cam_button_frame = ttk.Frame(master=detection_frame)
		cam_button_frame.grid(column=1, row=2, padx=5, pady=5, sticky="n")

		self.cam_refresh = ttk.Button(master=cam_button_frame, text="Refresh", command=self.refresh_cam)
		self.cam_refresh.grid(column=0, row=0, padx=5, pady=5, sticky="n")

		self.cam_select = ttk.Button(master=cam_button_frame, text="Confirm", command=self.cam_selection)
		self.cam_select.grid(column=1, row=0, padx=5, pady=5, sticky="n")

		# Dropdown to select serial port
		port_select_label = ttk.Label(master=detection_frame, text="Select Serial Port")
		port_select_label.grid(column=2, row=0, padx=5, pady=5, sticky="n")
		list_ports = SerialComms.list_ports()
		self.port_dropdown = ttk.Combobox(master=detection_frame, values=list_ports)
		self.port_dropdown.current(0)
		self.port_dropdown.grid(column=2, row=1, padx=5, pady=5, sticky="n")

		# Serial port button frame
		serial_port_button_frame = ttk.Frame(master=detection_frame)
		serial_port_button_frame.grid(column=2, row=2, padx=5, pady=5, sticky="n")

		self.port_refresh = ttk.Button(master=serial_port_button_frame, text="Refresh", command=self.refresh_ports)
		self.port_refresh.grid(column=0, row=0, padx=5, pady=5, sticky="n")

		self.port_select = ttk.Button(master=serial_port_button_frame, text="Confirm", command=self.port_selection)
		self.port_select.grid(column=1, row=0, padx=5, pady=5, sticky="n")

		####################
		# Buttons Frame #
		####################

		# Frame to show the Buttons
		button_frame = ttk.LabelFrame(master=self.window, text="Buttons")
		button_frame.grid(column=0, row=2, padx=10, pady=10, sticky="ew")
		button_frame.columnconfigure((0, 1), weight=1)
		button_frame.rowconfigure(0, weight=1)

		# Set and place buttons
		button_style = ttk.Style()
		button_style.configure("my.TButton", font=14)
		self.start = ttk.Button(master=button_frame, style="my.TButton", text="Start")
		self.failsafe = ttk.Button(master=button_frame, style="my.TButton", text="Failsafe")
		self.start.grid(column=0, row=0, padx=10, pady=10, ipadx=20, ipady=20, sticky="n")
		self.failsafe.grid(column=1, row=0, padx=10, pady=10, ipadx=20, ipady=20, sticky="n")

		####################
		# Parameter Frame #
		####################

		# Frame on the right for parameters
		param_frame = ttk.LabelFrame(master=self.window, text="Parameters")
		# Grid the param frame
		param_frame.grid(column=1, row=0, padx=10, pady=10, sticky="new")
		param_frame.columnconfigure(0, weight=1)
		param_frame.rowconfigure((0, 1), weight=1)

		####################
		# PID Frame #
		####################

		# PID frame
		pid_frame = ttk.LabelFrame(master=param_frame, text="PID")
		pid_frame.grid(column=0, row=0, padx=10, pady=10, sticky="nsew")
		pid_frame.columnconfigure((0, 1, 2, 3), weight=1)
		pid_frame.rowconfigure((0, 1, 2, 3), weight=1)

		# PID P
		pid_p_label = ttk.Label(master=pid_frame, text="Proportional")			# Make new label
		pid_p_label.grid(column=0, row=0, padx=10, pady=10, sticky="nw")		# Add label to PID frame
		self.param_p_m = ttk.Button(master=pid_frame, text="-0.1", command=self.pid_param_p_minus)
		self.param_p_m.grid(column=1, row=0, padx=2, pady=2, sticky="e")
		self.param_p_f = ttk.Entry(master=pid_frame, width=30)					# Make new Entry (user input box)
		self.param_p_f.grid(column=2, row=0, padx=10, pady=10, sticky="ns")		# Place Entry beside the label
		self.param_p_p = ttk.Button(master=pid_frame, text="+0.1", command=self.pid_param_p_plus)
		self.param_p_p.grid(column=3, row=0, padx=2, pady=2, sticky="w")

		# Add PID I label
		pid_i_label = ttk.Label(master=pid_frame, text="Integral")				# Make new label
		pid_i_label.grid(column=0, row=1, padx=10, pady=10, sticky="nw")		# Add label to PID frame
		self.param_i_m = ttk.Button(master=pid_frame, text="-0.001", command=self.pid_param_i_minus)
		self.param_i_m.grid(column=1, row=1, padx=2, pady=2, sticky="e")
		self.param_i_f = ttk.Entry(master=pid_frame, width=30)					# Make new Entry (user input box)
		self.param_i_f.grid(column=2, row=1, padx=10, pady=10, sticky="ns")		# Place Entry beside the label
		self.param_i_p = ttk.Button(master=pid_frame, text="+0.001", command=self.pid_param_i_plus)
		self.param_i_p.grid(column=3, row=1, padx=2, pady=2, sticky="w")

		# Add PID D label
		pid_d_label = ttk.Label(master=pid_frame, text="Derivative")			# Make new label
		pid_d_label.grid(column=0, row=2, padx=10, pady=10, sticky="nw")		# Add label to PID frame
		self.param_d_m = ttk.Button(master=pid_frame, text="-0.01", command=self.pid_param_d_minus)
		self.param_d_m.grid(column=1, row=2, padx=2, pady=2, sticky="e")
		self.param_d_f = ttk.Entry(master=pid_frame, width=30)					# Make new Entry (user input box)
		self.param_d_f.grid(column=2, row=2, padx=10, pady=10, sticky="ns")		# Place Entry beside the label
		self.param_d_p = ttk.Button(master=pid_frame, text="+0.01", command=self.pid_param_d_plus)
		self.param_d_p.grid(column=3, row=2, padx=2, pady=2, sticky="w")

		# Add Submit button
		self.param_pid_button = ttk.Button(master=pid_frame, text="Upload PID Params", command=self.get_pid_params)
		self.param_pid_button.grid(column=0, row=3, columnspan=3, padx=10, pady=10, ipadx=15, ipady=15, sticky="n")

		self.param_pid_pass_fail = ttk.Label(master=pid_frame, text="")
		self.param_pid_pass_fail.grid(column=3, row=3, padx=10, pady=10, sticky="ns")

		####################
		# Timeout Frame #
		####################

		# Second subframe for Timeout
		timeout_frame = ttk.LabelFrame(master=param_frame, text="Timeout")
		timeout_frame.grid(column=0, row=1, padx=10, pady=10, sticky="nsew")
		timeout_frame.columnconfigure((0, 1), weight=1)
		timeout_frame.rowconfigure((0, 1), weight=1)

		# Add Timeout label
		timeout_label = ttk.Label(master=timeout_frame, text="Timeout (seconds)")		# Make new label
		timeout_label.grid(column=0, row=0, padx=10, pady=10, sticky="nw")				# Add label to Timeout frame
		self.timeout = ttk.Entry(master=timeout_frame, width=30)						# Make new Entry (user input box)
		self.timeout.grid(column=1, row=0, padx=10, pady=10, sticky="ns")				# Place Entry beside the label

		# Add Timeout button
		self.timeout_button = ttk.Button(master=timeout_frame, text="Upload Timeout")
		self.timeout_button.grid(column=0, row=1, columnspan=2, padx=20, pady=10, ipadx=15, ipady=15, sticky="n")

		####################
		# Velocity Data Frame #
		####################
		velo_frame = ttk.LabelFrame(master=self.window, text="Velocity Data")
		velo_frame.grid(column=1, row=1, padx=10, pady=10, sticky="ew")
		velo_frame.columnconfigure((0, 1, 2), weight=1)
		velo_frame.rowconfigure((0, 1), weight=1)

		# Add labels and add them to velocity frame
		v_x_label = ttk.Label(master=velo_frame, text="X")
		v_y_label = ttk.Label(master=velo_frame, text="Y")
		v_z_label = ttk.Label(master=velo_frame, text="Z")
		v_x_label.grid(column=0, row=0, padx=5, pady=5, sticky="n")
		v_y_label.grid(column=1, row=0, padx=5, pady=5, sticky="n")
		v_z_label.grid(column=2, row=0, padx=5, pady=5, sticky="n")

		# Add the data labels and add them to velocity frame
		self.x_label = ttk.Label(master=velo_frame, text="0m/s")
		self.y_label = ttk.Label(master=velo_frame, text="0m/s")
		self.z_label = ttk.Label(master=velo_frame, text="0m/s")
		self.x_label.grid(column=0, row=1, padx=5, pady=5, sticky="n")
		self.y_label.grid(column=1, row=1, padx=5, pady=5, sticky="n")
		self.z_label.grid(column=2, row=1, padx=5, pady=5, sticky="n")

		####################
		# Sensor Data Frame #
		####################
		sensor_frame = ttk.LabelFrame(master=self.window, text="Sensor Data")
		sensor_frame.grid(column=1, row=2, padx=10, pady=10, sticky="ew")
		sensor_frame.columnconfigure((0, 1, 2, 3), weight=1)
		sensor_frame.rowconfigure((0, 1, 2, 3), weight=1)

		# Add all the associated labels and add them to sensor frame
		s_roll_label = ttk.Label(master=sensor_frame, text="Roll")
		s_pitch_label = ttk.Label(master=sensor_frame, text="Pitch")
		s_yaw_label = ttk.Label(master=sensor_frame, text="Yaw")
		s_cam_fps_label = ttk.Label(master=sensor_frame, text="Camera FPS")
		s_wind_spd_label = ttk.Label(master=sensor_frame, text="Wind Speed")
		s_wind_dir_label = ttk.Label(master=sensor_frame, text="Wind Direction")
		s_batt_vol_label = ttk.Label(master=sensor_frame, text="Battery Voltage")
		s_int_temp_label = ttk.Label(master=sensor_frame, text="Internal Temperature")
		s_roll_label.grid(column=0, row=0, padx=5, pady=5, sticky="n")
		s_pitch_label.grid(column=1, row=0, padx=5, pady=5, sticky="n")
		s_yaw_label.grid(column=2, row=0, padx=5, pady=5, sticky="n")
		s_cam_fps_label.grid(column=3, row=0, padx=5, pady=5, sticky="n")
		s_wind_spd_label.grid(column=0, row=2, padx=5, pady=5, sticky="n")
		s_wind_dir_label.grid(column=1, row=2, padx=5, pady=5, sticky="n")
		s_batt_vol_label.grid(column=2, row=2, padx=5, pady=5, sticky="n")
		s_int_temp_label.grid(column=3, row=2, padx=5, pady=5, sticky="n")

		# Add all associated sensor data labels
		self.roll_label = ttk.Label(master=sensor_frame, text="0°")
		self.pitch_label = ttk.Label(master=sensor_frame, text="0°")
		self.yaw_label = ttk.Label(master=sensor_frame, text="0°")
		self.cam_fps_label = ttk.Label(master=sensor_frame, text="0fps")
		self.wind_spd_label = ttk.Label(master=sensor_frame, text="0m/s")
		self.wind_dir_label = ttk.Label(master=sensor_frame, text="0°")
		self.batt_vol_label = ttk.Label(master=sensor_frame, text="0V")
		self.int_temp_label = ttk.Label(master=sensor_frame, text="0°C")

		self.roll_label.grid(column=0, row=1, padx=5, pady=5, sticky="n")
		self.pitch_label.grid(column=1, row=1, padx=5, pady=5, sticky="n")
		self.yaw_label.grid(column=2, row=1, padx=5, pady=5, sticky="n")
		self.cam_fps_label.grid(column=3, row=1, padx=5, pady=5, sticky="n")
		self.wind_spd_label.grid(column=0, row=3, padx=5, pady=5, sticky="n")
		self.wind_dir_label.grid(column=1, row=3, padx=5, pady=5, sticky="n")
		self.batt_vol_label.grid(column=2, row=3, padx=5, pady=5, sticky="n")
		self.int_temp_label.grid(column=3, row=3, padx=5, pady=5, sticky="n")


def check_cams():
	cam_dict = {-1: "Off"}
	for camera_info in enumerate_cameras():
		print(f'{camera_info.index}: {camera_info.name}')
		cam_dict[camera_info.index] = camera_info.name
	return cam_dict


if __name__ == '__main__':
	main_window = tk.Tk()
	main_window.wm_state("zoomed")
	main_window.columnconfigure((0, 1), weight=1)
	main_window.rowconfigure((0, 1, 2), weight=1)
	main_window.minsize(1050, 710)
	app = Monitor(main_window)
	main_window.mainloop()
