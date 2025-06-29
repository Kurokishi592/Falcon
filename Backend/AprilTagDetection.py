import cv2
import numpy as np
import time
from pupil_apriltags import Detector # Importing the apriltag detector from pupil_labs, as a python wrapper for the C version of apriltags

"""
1. Initialisation and configuration of the AprilTag detector
2. Detection of AprilTags in a given frame
3. Processing of detected tags to extract pose and velocity information
4. Results returned in a structured format

Detection:
ArUco is what jellyfish uses, and it's supposed to be much simpler and better compatibility with opencv, but we will upgrade it
I want to try AprilTag V3 which is said to be optimised "for quadrotor localisation above a landing site".
specifically, it has a recursive tag family that is designed for high-speed detection and pose estimation
https://docs.wpilib.org/en/stable/_downloads/e72e01c5464f1a0838751a5cb158087e/krogius2019iros.pdf

Implementation:
This class is a wrapper around the pupil_apriltags library, which is a two-class Python wrapper for the C implementation of AprilTags.
we will build other features as well into our own class, such as the tracking of tag's pose over time and velocity of tag relative to the camera
"""

class AprilTagDetector:
    def __init__(self, fx=800.0, fy=800.0, cx=300.0, cy=200.0, tag_size=0.25):
        """
        Initialise apriltag detector with camera intrinsics and tag size
        camera params can be updated later using cv2.calibrateCamera()
        the following params are needed for pose estimation
        """
        self.fx = fx                # Focal length in pixels (x axis)
        self.fy = fy                # Focal length in pixels (y axis)   
        self.cx = cx                # Principal point x in pixels based on 600x400 res
        self.cy = cy                # Principal point y in pixels based on 600x400 res
        self.tag_size = tag_size    # Size of the AprilTag in meters
        
        # Internal pose state for velocity estimation
        self.prev_pose = None
        self.prev_time = None

        self.detector = Detector(
            families="tagCustom48h12",    # for our recursive tag we must use this family other families like tag36h11 or tag16h5 or tagStandard41h12 have different varieties and purposes
            nthreads=4,                   # Number of threads to use for detection
            quad_decimate=1.0,            # Decimation factor (how much input image is downscaled before the quad detection) on lower res image for better performance at cost of pose accuracy and detection rate. For moving tags, keep low.
            quad_sigma=0.8,               # Applies Gaussian blur (std deviation) to the input image before detection, helps with noisy images
            refine_edges=True,            # Whether to refine the edges of the detected tags
            decode_sharpening=0.25,       # Sharpening factor after detection and before decoding the binary ID of the tag. 
            debug=False,             
        )
        


    def detect(self, frame):
        """ 
        Detect AprilTags in a given frame and compute their poses and velocities.
        """
        # Convert to grayscale, a detection step introduced in Apriltag V2.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Run detection through webcam stream, not in just one image capture 
        detections = self.detector.detect(
            gray, 
            # the following params are set to extract the tag pose
            estimate_tag_pose=True, 
            camera_params=(self.fx, self.fy, self.cx, self.cy), 
            tag_size=self.tag_size
        )
        
        results = [self._process_detection(detection) for detection in detections]
        return results
    
    def _process_detection(self, detection):
        """
        Process a single detection to extract pose and velocity information.
        """
        # Extract the pose of the tag
        pose_t = detection.pose_t #3x1 translation vector
        pose_R = detection.pose_R #3x3 rotation matrix
            
        # Pose estimation: compute the camera position relative to the tag
        # using the pinhole camera model: camera position is the negative of the rotation matrix transposed multiplied by the translation vector
        pose = -np.matmul(pose_R.T, pose_t)  
            
        # Compute velocity of the tag
        current_time = time.time()
        velocity = np.zeros(3) # Initialize velocity to zero
            
        # Calculate the velocity if we have a previous pose
        if self.prev_pose is not None and self.prev_time is not None:
            dt = current_time - self.prev_time
            if dt > 0:
                velocity = (pose - self.prev_pose).flatten() / dt
            
        # Update state for the next detection    
        self.prev_pose = pose
        self.prev_time = current_time
            
        return {
            "id": detection.tag_id,
            "pose": pose.flatten(),             # [x,y,z] relative position
            "rotation": pose_R, 
            "translation": pose_t.flatten(),    # tag-to-camera vector
            "velocity": velocity                # [vx, vy, vz]
        }
        
