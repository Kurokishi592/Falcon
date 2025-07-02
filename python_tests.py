import unittest

class TestFalcon(unittest.TestCase):

    def test_apriltag_3_detection(self):
        # Test if AprilTag detector detects the tags correctly
        try:
            import cv2
            from Backend.AprilTagDetection import AprilTagDetector
            detector = AprilTagDetector(cx=742, cy=740)
            image = cv2.imread('Tests/test_3_recursive.png')
            tags = detector.detect(image)
            if tags:
                print(f"Detected {len(tags)} tag(s): " + ', '.join(str(d.tag_id) for d in tags))
            else:
                print("No tags detected.")
        except:
            self.fail("AprilTagDetector import failed")
        self.assertEqual(len(tags), 3, "Expected to detect three tags in the test image")


if __name__ == '__main__':
    unittest.main()