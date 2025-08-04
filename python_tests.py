import unittest
import time
import cv2
from Backend.AprilTagDetection import AprilTagDetector
from Backend.Controls2 import FalconController
from Backend.PID import PID


class TestFalcon(unittest.TestCase):
    """
    Automatic Unit Tests for Falcon
    """

    def test_apriltag_recursive_3_detection(self):
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
            self.fail(f"AprilTag recursive detection failed {e}")

    
    def test_apriltag_no_detection(self):
        """
        Tests if AprilTag detector detects no tags in a blank image
        Should detect zero tags
        """
        try:
            detector = AprilTagDetector()
            image = cv2.imread('Tests/test_0_blank.png')
            tags = detector.detect(image)
            self.assertEqual(len(tags), 0, "Expected to detect zero tags in the test image")
        except Exception as e:
            self.fail(f"AprilTag no detection failed {e}")


    def test_apriltag_rotate_detection(self):
        """
        Tests if AprilTag detector detects tags in a rotated tag
        Should detect two tags with IDs 4, 5 and 6
        """
        try:
            detector = AprilTagDetector()
            image = cv2.imread('Tests/test_3_rotated.png')
            tags = detector.detect(image)
            self.assertEqual(len(tags), 2, "Expected to detect two tags in the test image")
            self.assertEqual(tags[0].tag_id, 4, "Expected first tag ID to be 4")
            self.assertEqual(tags[1].tag_id, 5, "Expected second tag ID to be 5")
        except Exception as e:
            self.fail(f"AprilTag rotation detection failed {e}")


    def test_apriltag_warp_detection(self):
        """
        Tests if AprilTag detector detects tags in a warped tag
        Should detect two tags with IDs 0 and 1 (image has 3 but due to warping, only 2 should be detected)
        """
        try:
            detector = AprilTagDetector()
            image = cv2.imread('Tests/test_3_warped.png')
            tags = detector.detect(image)
            self.assertEqual(len(tags), 2, "Expected to detect two tags in the test image")
            self.assertEqual(tags[0].tag_id, 0, "Expected first tag ID to be 0")
            self.assertEqual(tags[1].tag_id, 1, "Expected second tag ID to be 1")
        except Exception as e:
            self.fail(f"AprilTag warp detection failed {e}")


    def test_velocity(self):
        """
        Tests if FalconController computes the velocity correctly
        Should return 0.03, 0.0, 0.0 for first and the negation for second
        """
        try:
            frame1 = cv2.imread("Tests/test_3_original.png")
            frame2 = cv2.imread("Tests/test_3_shifted.png")
            detector = AprilTagDetector()
            detection1 = detector.detect(frame1)
            detection2 = detector.detect(frame2)
            fc = FalconController()
            fc.new_detection(detection1[0])
            time.sleep(0.5)
            fc.new_detection(detection2[0])
            velocity1 = fc.get_velocity()
            time.sleep(0.5)
            fc.new_detection(detection1[0])
            velocity2 = fc.get_velocity()

            self.assertEqual(float(round(velocity1[0], 2)), 0.02, "Expected x velocity 1 to be -0.02")
            self.assertEqual(float(round(velocity1[1], 2)), 0.0, "Expected y velocity 1 to be 0.0")
            self.assertEqual(float(round(velocity1[2], 2)), 0.0, "Expected z velocity 1 to be 0.0")
            self.assertEqual(float(round(velocity2[0], 2)), 0.02, "Expected x velocity 2 to be 0.02")
            self.assertEqual(float(round(velocity2[1], 2)), 0.0, "Expected y velocity 2 to be 0.0")
            self.assertEqual(float(round(velocity2[2], 2)), 0.0, "Expected z velocity 2 to be 0.0")
        except Exception as e:
            self.fail(f"AprilTag Velocity computation failed {e}")

    def test_zero_velocity_detection(self):
        """
        Tests if FalconController returns zero velocity when no detection is made
        Should return 0.0, 0.0, 0.0
        """
        try:
            frame1 = cv2.imread("Tests/test_3_original.png")
            frame2 = cv2.imread("Tests/test_3_original.png")
            detector = AprilTagDetector()
            detection1 = detector.detect(frame1)
            detection2 = detector.detect(frame2)
            fc = FalconController()
            fc.new_detection(detection1[0])
            fc.new_detection(detection2[0])
            velocity = fc.get_velocity()
            self.assertEqual(velocity[0], 0.0, "Expected x velocity to be 0.0")
            self.assertEqual(velocity[1], 0.0, "Expected y velocity to be 0.0")
            self.assertEqual(velocity[2], 0.0, "Expected z velocity to be 0.0")
        except Exception as e:
            self.fail(f"FalconController zero velocity failed {e}")

    def test_PID(self):
        """
        Tests if FalconController PID controller computes the correct control output
        Should return a non-zero control output for a given error
        """
        try:
            pid = PID(0.2, 0.1, 0.05)
            self.assertTrue(pid.get_values() == (0.2, 0.1, 0.05), "PID values do not match expected values")

            output1 = pid.update(1.0, 0.5)
            self.assertTrue(round(output1, 1) == 2.6, "PID output should be 2.6")

            pid.modify_values(0.3, 0.2, 0.1)
            self.assertTrue(pid.get_values() == (0.3, 0.2, 0.1), "PID values after modification do not match expected values")

            output2 = pid.update(1.0, 0.5)
            self.assertTrue(round(output2, 1) == 0.2, "PID output should be 0.2")

        except Exception as e:
            self.fail(f"FalconController PID computation failed {e}")


if __name__ == '__main__':
    unittest.main()