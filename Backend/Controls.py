import numpy as np
import time
from AprilTagDetection import AprilTagDetector

"""
Runs the control logic for UAV after detection
Main logic:
Assume the drone is stationary, given the tag's relative velocity from it, 
cancel out the tag's motion by commanding the drone to move in the opposite direction.
If tag is moving → v_relative = tag velocity in camera frame
If drone is moving → v_relative = negative drone velocity in camera frame
If both are moving → v_relative = (tag velocity - drone velocity) in camera frame

To approach and land, the drone will need to:
1. Cancel the tag's velocity
2. Apply correction based on position error, use a PID controller to adjust the drone's velocity

The desired velocity should be part of the result to be sent to GUI
"""

class FalconController:
    def __init__(self):
        self.detector = AprilTagDetector()
        
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.max_velocity = 1.0
        
        self.current_uav_position = np.array([0.0, 0.0, 0.0])
        
    def compute_desired_velocity(self, tag_pose, tag_velocity):
        """
        Compute velocity command for uav to approach the tag
        Inputs:
        - tag_pose: np.array ([x, y, z]) representing the position of the tag in drone frame (meters)
        - tag_velocity: np.array ([vx, vy, vz]) representing the velocity of the tag in drone frame (m/s)
        Returns:
        - desired_velocity: np.array ([vx, vy, vz]) representing the desired velocity of the drone (m/s)
        """
        
        tag_pose = np.array(tag_pose)
        tag_velocity = np.array(tag_velocity)
        
        # use PID controller, here we only use P controller for now
        position_error = tag_pose - self.current_uav_position
        desired_velocity = tag_velocity + self.kp * position_error
        
        # Clip the desired velocity to the maximum velocity
        speed = np.linalg.norm(desired_velocity)
        if speed > self.max_velocity:
            desired_velocity = (desired_velocity / speed) * self.max_velocity
        return desired_velocity.tolist()  # Convert to list for compatibility with GUI
    
    def run_once(self, frame):
        """
        Analysing a frame from the camera, detect AprilTags, compute desired velocity.
        Return a list of tag detection results with added desired velocity.
        """
        detections = self.detector.detect(frame)            # List of detected AprilTags

        results = []
        for detection in detections:
            print(detection)
            tag_pose = detection.pose_t
            tag_velocity = detection['velocity']
            
            desired_velocity = self.compute_desired_velocity(tag_pose, tag_velocity)
            detection["desired_velocity"] = desired_velocity
            results.append(detection)
        return results
    
        
if __name__ == "__main__":
    fc = FalconController()
    try:
        import cv2
        import sys
        import os
        sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../..")

        frame = cv2.imread('Tests/test_3_recursive.png')
        results = fc.run_once(frame)
        print(results)
        if results is not None:
            for det in results:
                print(f"Tag ID {det['id']}: Pose {det['pose']}, Velocity {det['velocity']}, Desired Vel {det['desired_velocity']}")
    except ImportError as e:
        print(f"Error importing modules: {e}")