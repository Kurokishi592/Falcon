import sys
import time
import tkinter as tk
import threading
import asyncio
from Frontend.Tkinter.tkinter_main import Monitor
from Backend.AprilTagDetection import AprilTagDetector
from Backend.Controls2 import FalconController
import Frontend.TelegramBot.tele_main as tele_main


def check_params(tkinter_app, estimator):
	# Function to check if the IMU data and velocity are within expected ranges.
	print("Starting parameter check thread")
	last_time = time.time()
	while True:
		curr_time = time.time()
		delta_time = curr_time - last_time
		# Check that 3s has passed since the last check
		if delta_time < 3:
			time.sleep(0.1)
			continue
		imu_data = tkinter_app.send_serial_data()
		velocity = estimator.get_velocity()
		if imu_data is None and velocity is None:
			time.sleep(0.1)
			continue
		message = ""
		if imu_data is not None:
			if not (-45 <= imu_data["Roll"] <= 45):
				message += f"Roll out of range: {imu_data['Roll']}. "
			if not (-45 <= imu_data["Pitch"] <= 45):
				message += f"Pitch out of range: {imu_data['Pitch']}. "
		if velocity is not None:
			if not (-0.2 <= velocity[0] <= 0.2):
				print("Velocity x out of range")
				message += f"Velocity x out of range: {velocity[0]}. "
			if not (-0.2 <= velocity[1] <= 0.2):
				message += f"Velocity y out of range: {velocity[1]}. "
			if not (-0.2 <= velocity[2] <= 0.2):
				message += f"Velocity z out of range: {velocity[2]}. "
		if message:
			print(f"Sending error message {message}")
			asyncio.run(tele_main.send_msg_ext(message))
			last_time = curr_time
		time.sleep(0.1)

	
if __name__ == '__main__':
	detector = AprilTagDetector()
	velocity_estimator = FalconController()

	# Start the Telegram bot
	telebot_thread = threading.Thread(target=tele_main.botloop)
	telebot_thread.daemon = True  # Ensure the thread exits when the main program does
	telebot_thread.start()

	# Initialize the Tkinter GUI
	main_window = tk.Tk()
	main_window.wm_state("zoomed")
	main_window.columnconfigure((0, 1), weight=1)
	main_window.rowconfigure((0, 1, 2), weight=1)
	main_window.minsize(1050, 710)
	app = Monitor(main_window)
	app.set_detector(detector)
	app.set_velocity_estimator(velocity_estimator)

	# Start monitoring output values from estimator and serial port
	monitor_thread = threading.Thread(target=check_params, args=(app, velocity_estimator))
	monitor_thread.daemon = True  # Ensure the thread exits when the main program does
	monitor_thread.start()

	main_window.protocol("WM_DELETE_WINDOW", sys.exit)
	main_window.mainloop()
