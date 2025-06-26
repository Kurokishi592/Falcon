import cv2
import numpy as np
from pupil_apriltags import Detector # Importing the apriltag detector from pupil_labs, non-ROS python optimised.

# I want to try AprilTag V3 which is said to be optimised "for quadrotor localisation above a landing site".
# ArUco is what jellyfish uses and it's supposed to be much simpler and better compatibility with opencv. 

class AprilTagDetector:
    def __init__(self, fx=800.0, fy=800.0, cx=300.0, cy=200.0, tag_size=0.16):
       # Camera calibration parameters, to be replaced later using cv2.calibrateCamera()
        self.fx = fx                # Focal length in pixels (x axis
        self.fy = fy                # Focal length in pixels (y axis)   
        self.cx = cx                # Principal point x in pixels based on 600x400 res
        self.cy = cy                # Principal point y in pixels based on 600x400 res
        self.tag_size = tag_size    # Size of the AprilTag in meters

        self.detector = Detector(
            families="tagStandard41h12",  # we can use other families like tag36h11 or tag16h5
            nthreads=4,                   # Number of threads to use for detection
            quad_decimate=1.0,            # Decimation factor for the quad detection, can try 2.0 for better performance at cost of accuracy
            quad_sigma=0.0,               # Sigma for the quad detection
            refine_edges=True,            # Whether to refine the edges of the detected tags
            decode_sharpening=0.25,       # Sharpening factor for decoding
            debug=False,             
        )

    def detect(self, frame):
        # Convert to grayscale, a detection step introduced in Apriltag V2.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        # Run detection through webcam stream, not in just one image capture 
        detections = self.detector.detect(
            gray, 
            estimate_tag_pose=True, 
            camera_params=(self.fx, self.fy, self.cx, self.cy), 
            tag_size=self.tag_size
        )
        return detections