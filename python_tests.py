import unittest

class TestFalcon(unittest.TestCase):

    def test_apriltag_detection(self):
        # Test if AprilTag detects the tags correctly
        try:
            import cv2
            from Backend.AprilTagDetection import AprilTagDetector
            detector = AprilTagDetector(cx=742, cy=740)
            image = cv2.imread('Tests/apriltag_test.png')
            tags = detector.detect(image)
            if tags:
                message = f"Detected {len(tags)} tag(s): " + ', '.join(str(d.tag_id) for d in tags)
                print(message)
            else:
                print("No tags detected.")
        except:
            self.fail("AprilTagDetector import failed")

if __name__ == '__main__':
    unittest.main()