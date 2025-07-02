import unittest
import time
import cv2
from Backend.AprilTagDetection import AprilTagDetector
from Backend.Controls1 import FalconController


class TestFalcon(unittest.TestCase):
    """
    Automatic Unit Tests for Falcon
    """

    def test_apriltag_3_detection(self):
        """
        Tests if AprilTag detector detects the tags correctly
        Checks for number of tags detected, and IDs
        Should detect three tags with IDs 0, 1, and 2
        """
        try:
            detector = AprilTagDetector()
            image = cv2.imread('Tests/test_3_recursive.png')
            tags = detector.detect(image)
            self.assertEqual(len(tags), 3, "Expected to detect three tags in the test image")
            self.assertEqual(tags[0].tag_id, 0, "Expected first tag ID to be 0")
            self.assertEqual(tags[1].tag_id, 1, "Expected second tag ID to be 1")
            self.assertEqual(tags[2].tag_id, 2, "Expected third tag ID to be 2")
        except Exception as e:
            self.fail(f"AprilTag detection failed {e}")

    def test_velocity(self):
        """
        Tests if FalconController computes the velocity correctly
        Should return 0.03, 0.0, 0.0 for first and the negation for second
        """
        try:
            frame1 = cv2.imread("Tests/original.png")
            frame2 = cv2.imread("Tests/shifted.png")
            detector = AprilTagDetector()
            detection1 = detector.detect(frame1)
            detection2 = detector.detect(frame2)
            fc = FalconController()
            fc.new_detection(detection1[0])
            time.sleep(0.5)
            fc.new_detection(detection2[0])
            velocity1 = fc.get_velocity()
            fc.new_detection(detection1[0])
            velocity2 = fc.get_velocity()

            self.assertEqual(float(round(velocity1[0], 2)), 0.03, "Expected x velocity to be 0.33")
            self.assertEqual(float(round(velocity1[1], 2)), 0.0, "Expected y velocity to be 0.0")
            self.assertEqual(float(round(velocity1[2], 2)), 0.0, "Expected z velocity to be 0.0")
            self.assertEqual(float(round(velocity2[0], 2)), -0.03, "Expected x velocity to be -0.33")
            self.assertEqual(float(round(velocity2[1], 2)), 0.0, "Expected y velocity to be 0.0")
            self.assertEqual(float(round(velocity2[2], 2)), 0.0, "Expected z velocity to be 0.0")
        except Exception as e:
            self.fail(f"Velocity computation failed {e}")


if __name__ == '__main__':
    unittest.main()