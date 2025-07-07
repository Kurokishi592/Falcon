import time
import numpy as np

class FalconController():
    def __init__(self):
        self.pose = None
        self._last_pose_t = None
        self._last_time = None

        self.velocity = np.zeros(3)

    def _compute_pose(self, detection):
        """
        Compute pose from detection data
        :param detection: Detection object containing pose_R and pose_t
        :return: Pose (pose_R transposed, t) as a numpy array
        """
        pose = -np.matmul(detection.pose_R.T, detection.pose_t)
        return pose
    
    def new_detection(self, detection):
        """
        Process a new detection and update the pose and velocity
        :param detection: Detection object containing pose_R and pose_t
        :return: None
        """
        if self._last_time is None:                 # For new detections
            self._last_time = time.time()
            self._last_pose_t = self._compute_pose(detection)
        elif self.pose is not None:
            self._last_pose_t = self.pose.copy()
            self.pose = self._compute_pose(detection)
        else:
            self.pose = self._compute_pose(detection)

    def get_velocity(self):
        """
        Calculate the velocity based on the difference between current and previous pose and time
        :return: Velocity as a numpy array, or zero vector if no previous pose exists
        """
        current_time = time.time()

        if self._last_pose_t is not None and self.pose is not None:
            dt = current_time - self._last_time
            if dt > 0:
                delta = (self.pose - self._last_pose_t).flatten()
                self.velocity = delta / dt
                return self.velocity
        else:
            return np.zeros(3)


if __name__ == "__main__":
    """
    Controls test to check if FalconController computes velocity correctly
    Replicated in unittest for automatic testing
    """
    import cv2
    from AprilTagDetection import AprilTagDetector
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../..")
    frame1 = cv2.imread("Tests/original.png")
    frame2 = cv2.imread("Tests/shifted.png")
    detector = AprilTagDetector()
    detection1 = detector.detect(frame1)
    detection2 = detector.detect(frame2)
    fc = FalconController()
    fc.new_detection(detection1[0])
    time.sleep(0.5)
    fc.new_detection(detection2[0])
    fc.get_velocity()
    print("Velocity:", fc.velocity)
    fc.new_detection(detection1[0])
    fc.get_velocity()
    print("Velocity:", fc.velocity)
