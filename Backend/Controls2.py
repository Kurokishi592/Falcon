import time
import numpy as np

class FalconController():
    def __init__(self):
        # self.tag_size_0 = 1.25      # Size of tag_id 0 in meters
        # self.tag_size_1 = 0.25      # Size of tag_id 1 in meters
        # self.tag_size_2 = 0.05      # Size of tag_id 2 in meters
        self.tag_size_0 = 0.161      # Size of tag_id 0 in meters
        self.tag_size_1 = 0.031      # Size of tag_id 1 in meters
        self.tag_size_2 = 0.005      # Size of tag_id 2 in meters

        self._last_id = None
        self._last_corners = None
        self._last_time = None

        self.velocity = np.zeros(3)
    
    def _tag_width_pixels(self, corners):
        """
        Calculate the width of the tag in pixels
        :param corners: List of corners of the tag
        :return: Width in pixels
        """
        edge_l = np.linalg.norm(corners[0] - corners[1])
        edge_b = np.linalg.norm(corners[1] - corners[2])
        return np.mean([edge_l, edge_b])

    def new_detection(self, detection):
        """
        Process a new detection and update the pose and velocity
        :param detection: Detection object containing pose_R and pose_t
        :return: None
        """
        if self._last_time is None:
            self._last_id = detection.tag_id
            self._last_time = time.time()
            self._last_corners = detection.corners
        elif self._last_corners is not None:
            if detection.tag_id != self._last_id:
                self._last_time = time.time()
                self._last_id = detection.tag_id
                self._last_corners = detection.corners
            else:
                # Velocity logic here
                curr_time = time.time()
                dt = curr_time - self._last_time
                if dt > 0:
                    if (detection.tag_id == 0):
                        scale = self.tag_size_0
                    elif (detection.tag_id == 1):
                        scale = self.tag_size_1
                    elif (detection.tag_id == 2):
                        scale = self.tag_size_2

                    curr_xy = np.mean(detection.corners, axis=0)
                    last_xy = np.mean(self._last_corners, axis=0)

                    curr_width = self._tag_width_pixels(detection.corners)
                    last_width = self._tag_width_pixels(self._last_corners)

                    # X/Y velocity based on center movement
                    delta_xy = curr_xy - last_xy
                    self.velocity[:2] = delta_xy * (scale / curr_width) / dt

                    # Z velocity based on tag size change
                    width_change = (curr_width - last_width) / last_width
                    self.velocity[2] = width_change * scale / dt

                    self._last_time = curr_time
                    self._last_id = detection.tag_id
                    self._last_corners = detection.corners

    def get_velocity(self):
        """
        Calculate the velocity based on the difference between current and previous corners and time
        :return: Velocity as a numpy array, or zero vector if no previous corners exist
        """
        if self._last_corners is not None:
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
    print("Velocity:", fc.get_velocity())
    time.sleep(0.5)
    fc.new_detection(detection1[0])
    print("Velocity:", fc.get_velocity())
