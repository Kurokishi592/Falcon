import tkinter as tk
from tkinter import ttk
from cv2_enumerate_cameras import enumerate_cameras
import cv2
from PIL import Image, ImageTk


class Monitor:
	def __init__(self, window):
		self.window = window

		# AprilTag stuff
		self.detected_message_label = None
		self.detected_message_data = "Nothing"

		# Camera instance stuff
		self.vid = None

		# Camera frame stuff
		self.cam_frame = None
		self.width, self.height = 600, 400

		# Camera select stuff
		self.list_cams = []
		self.cam_dropdown = None
		self.cam_select = None

		# Control buttons
		self.start = None
		self.stop = None
		self.failsafe = None

		# PID stuff
		self.param_p = None
		self.param_i = None
		self.param_d = None
		self.param_pid_button = None

		# Marker loss timeout stuff
		self.timeout = None
		self.timeout_button = None

		# Velocity stuff
		self.x_label = None
		self.y_label = None
		self.z_label = None
		self.x_data = str(0)
		self.y_data = str(0)
		self.z_data = str(0)

		# Parameter labels
		self.roll_label = None
		self.pitch_label = None
		self.yaw_label = None
		self.cam_fps_label = None
		self.wind_spd_label = None
		self.wind_dir_label = None
		self.batt_vol_label = None
		self.int_temp_label = None

		# Parameter text
		self.roll_data = str(0)
		self.pitch_data = str(0)
		self.yaw_data = str(0)
		self.cam_fps_data = str(0)
		self.wind_spd_data = str(0)
		self.wind_dir_data = str(0)
		self.batt_vol_data = str(0)
		self.int_temp_data = str(0)

		self.find_cams()
		self.cam_start()
		self.create_gui()

	def find_cams(self):
		self.list_cams = check_cams()
		print(self.list_cams)

	def cam_start(self):
		# Camera stuff
		self.vid = cv2.VideoCapture(700)
		self.vid.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
		self.vid.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

	def cam_feed(self):
		_, frame = self.vid.read()
		raw_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		conv_image = Image.fromarray(raw_image)
		photo_image = ImageTk.PhotoImage(image=conv_image)
		self.cam_frame.photo_image = photo_image
		self.cam_frame.configure(image=photo_image)
		self.cam_frame.after(10, self.cam_feed)

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

		# Placeholder for where the live camera feed will be
		# camera_placeholder_label = ttk.Label(master=camera_frame, text="Camera Live Feed to be added here")
		# camera_placeholder_label.grid(column=0, row=0, padx=20, pady=20, sticky="")

		# Live camera feed
		self.cam_frame = ttk.Label(master=camera_frame)
		self.cam_frame.grid(column=0, row=0, padx=20, pady=20, sticky="")
		self.cam_feed()

		####################
		# Detection Frame #
		####################

		# Frame to show AprilTag detected, and corresponding message
		detection_frame = ttk.LabelFrame(master=self.window, text="Detected Tag")
		# Grid the detection frame
		detection_frame.grid(column=0, row=1, padx=10, pady=10, sticky="ew")
		detection_frame.columnconfigure((0, 1), weight=1)
		detection_frame.rowconfigure((0, 1), weight=1)

		# Placeholder for live cropped view
		camera_placeholder_label = ttk.Label(master=detection_frame, text="AprilTag detection view to be added")
		camera_placeholder_label.grid(column=0, row=0, rowspan=3, padx=5, pady=5, sticky="ns")

		detection_label = ttk.Label(master=detection_frame, text="Detected AprilTag message")
		detection_label.grid(column=1, row=0, padx=5, pady=5, sticky="n")

		self.detected_message_label = ttk.Label(master=detection_frame, text=self.detected_message_data)
		self.detected_message_label.grid(column=1, row=1, rowspan=2, padx=5, pady=5, sticky="ns")

		cam_select_label = ttk.Label(master=detection_frame, text="Select Camera")
		cam_select_label.grid(column=2, row=0, padx=5, pady=5, sticky="n")

		self.cam_dropdown = ttk.Combobox(master=detection_frame, values=self.list_cams)
		self.cam_dropdown.grid(column=2, row=1, padx=5, pady=5, sticky="n")
		self.cam_select = ttk.Button(master=detection_frame, text="Confirm")
		self.cam_select.grid(column=2, row=2, padx=5, pady=5, sticky="n")

		####################
		# Buttons Frame #
		####################

		# Frame to show the Buttons
		button_frame = ttk.LabelFrame(master=self.window, text="Buttons")
		button_frame.grid(column=0, row=2, padx=10, pady=10, sticky="ew")
		button_frame.columnconfigure((0, 1, 2), weight=1)
		button_frame.rowconfigure(0, weight=1)

		# Set and place buttons
		button_style = ttk.Style()
		button_style.configure("my.TButton", font=14)
		self.start = ttk.Button(master=button_frame, style="my.TButton", text="Start")
		self.stop = ttk.Button(master=button_frame, style="my.TButton", text="Stop")
		self.failsafe = ttk.Button(master=button_frame, style="my.TButton", text="Failsafe")
		self.start.grid(column=0, row=0, padx=10, pady=10, ipadx=20, ipady=20, sticky="n")
		self.stop.grid(column=1, row=0, padx=10, pady=10, ipadx=20, ipady=20, sticky="n")
		self.failsafe.grid(column=2, row=0, padx=10, pady=10, ipadx=20, ipady=20, sticky="n")

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
		pid_frame.columnconfigure((0, 1), weight=1)
		pid_frame.rowconfigure((0, 1, 2, 3), weight=1)

		# PID P
		pid_p_label = ttk.Label(master=pid_frame, text="Proportional")			# Make new label
		pid_p_label.grid(column=0, row=0, padx=10, pady=10, sticky="nw")		# Add label to PID frame
		self.param_p = ttk.Entry(master=pid_frame, width=30)					# Make new Entry (user input box)
		self.param_p.grid(column=1, row=0, padx=10, pady=10, sticky="ns")		# Place Entry beside the label

		# Add PID I label
		pid_i_label = ttk.Label(master=pid_frame, text="Integral")				# Make new label
		pid_i_label.grid(column=0, row=1, padx=10, pady=10, sticky="nw")		# Add label to PID frame
		self.param_i = ttk.Entry(master=pid_frame, width=30)					# Make new Entry (user input box)
		self.param_i.grid(column=1, row=1, padx=10, pady=10, sticky="ns")		# Place Entry beside the label

		# Add PID D label
		pid_d_label = ttk.Label(master=pid_frame, text="Derivative")			# Make new label
		pid_d_label.grid(column=0, row=2, padx=10, pady=10, sticky="nw")		# Add label to PID frame
		self.param_d = ttk.Entry(master=pid_frame, width=30)					# Make new Entry (user input box)
		self.param_d.grid(column=1, row=2, padx=10, pady=10, sticky="ns")		# Place Entry beside the label

		# Add Submit button
		self.param_pid_button = ttk.Button(master=pid_frame, text="Upload PID Params")
		self.param_pid_button.grid(column=0, row=3, columnspan=2, padx=10, pady=10, ipadx=15, ipady=15,  sticky="n")

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
		self.x_label = ttk.Label(master=velo_frame, text=self.x_data + "m/s")
		self.y_label = ttk.Label(master=velo_frame, text=self.y_data + "m/s")
		self.z_label = ttk.Label(master=velo_frame, text=self.z_data + "m/s")
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
		self.roll_label = ttk.Label(master=sensor_frame, text=self.roll_data + "°")
		self.pitch_label = ttk.Label(master=sensor_frame, text=self.pitch_data + "°")
		self.yaw_label = ttk.Label(master=sensor_frame, text=self.yaw_data + "°")
		self.cam_fps_label = ttk.Label(master=sensor_frame, text=self.cam_fps_data + "fps")
		self.wind_spd_label = ttk.Label(master=sensor_frame, text=self.wind_spd_data + "m/s")
		self.wind_dir_label = ttk.Label(master=sensor_frame, text=self.wind_dir_data + "°")
		self.batt_vol_label = ttk.Label(master=sensor_frame, text=self.batt_vol_data + "V")
		self.int_temp_label = ttk.Label(master=sensor_frame, text=self.int_temp_data + "°C")

		self.roll_label.grid(column=0, row=1, padx=5, pady=5, sticky="n")
		self.pitch_label.grid(column=1, row=1, padx=5, pady=5, sticky="n")
		self.yaw_label.grid(column=2, row=1, padx=5, pady=5, sticky="n")
		self.cam_fps_label.grid(column=3, row=1, padx=5, pady=5, sticky="n")
		self.wind_spd_label.grid(column=0, row=3, padx=5, pady=5, sticky="n")
		self.wind_dir_label.grid(column=1, row=3, padx=5, pady=5, sticky="n")
		self.batt_vol_label.grid(column=2, row=3, padx=5, pady=5, sticky="n")
		self.int_temp_label.grid(column=3, row=3, padx=5, pady=5, sticky="n")


def check_cams():
	cam_list = []
	for camera_info in enumerate_cameras():
		print(f'{camera_info.index}: {camera_info.name}')
		cam_list.append(camera_info.name)
	return cam_list


if __name__ == '__main__':
	main_window = tk.Tk()
	main_window.wm_state("zoomed")
	main_window.columnconfigure((0, 1), weight=1)
	main_window.rowconfigure((0, 1, 2), weight=1)
	main_window.minsize(1050, 710)
	app = Monitor(main_window)
	main_window.mainloop()
