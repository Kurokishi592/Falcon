import sys
import tkinter as tk
import threading
from Frontend.Tkinter.tkinter_main import Monitor
from Backend.AprilTagDetection import AprilTagDetector
from Backend.Controls2 import FalconController

if __name__ == '__main__':
	main_window = tk.Tk()
	main_window.wm_state("zoomed")
	main_window.columnconfigure((0, 1), weight=1)
	main_window.rowconfigure((0, 1, 2), weight=1)
	main_window.minsize(1050, 710)
	app = Monitor(main_window)
	app.set_detector(AprilTagDetector())
	app.set_velocity_estimator(FalconController())
	main_window.protocol("WM_DELETE_WINDOW", sys.exit)
	main_window.mainloop()
