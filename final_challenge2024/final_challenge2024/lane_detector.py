#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from race_msgs.msg import PursuitLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
# from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        super().__init__("cone_detector")
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.cone_pub = self.create_publisher(PursuitLocationPixel, "/relative_cone_px", 10)
        self.debug_pub = self.create_publisher(Image, "/cone_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.get_logger().info("Cone Detector Initialized")

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        img = np.array(image, dtype=np.uint8)

        height, width, channels = img.shape

        # mask = np.zeros(img.shape[:2], dtype=np.uint8)

        # # Define the points of the triangle
        # pts = np.array([[0, height], [width//2, 0], [width, height]])

        # # Draw the triangle
        # cv2.drawContours(mask, [pts], 0, 255, -1)

        # cropped_image = cv2.bitwise_and(img, img, mask=mask)

        middle = width/2

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
        white_pixels_img = cv2.bitwise_and(img, img, mask=thresh)

        cv2.rectangle(white_pixels_img, (0, 0), (width, int(height/4)), (0, 0, 0), -1)

        edges = cv2.Canny(white_pixels_img, 50, 200)

        minLineLength = 110
        maxLineGap = 30
        lines = cv2.HoughLinesP(edges, rho=2, theta=np.pi/180, threshold=100, minLineLength=110, maxLineGap=30)

        right_min_slope = 10  # Minimum slope angle
        right_max_slope = 60  # Maximum slope angle

        left_min_slope = -60  # Minimum slope angle
        left_max_slope = -10  # Maximum slope angle

        # Iterate over each line and filter based on slope
        right_lines = []
        left_lines = []

        closest_to_middle_left = 0

        closest_to_middle_right = width

        for line in lines:
            x1, y1, x2, y2 = line[0]  # Extract line coordinates
            average_x = (x1+x2)/2
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))  # Calculate angle (slope)
            # Check if angle falls within the threshold range
            if right_min_slope <= angle <= right_max_slope:
                right_lines.append(line)
            if left_min_slope <= angle <= left_max_slope:
                left_lines.append(line)
                
        for line in right_lines:
            x1, y1, x2, y2 = line[0]  # Extract line coordinates
            average_x = (x1+x2)/2
                    
            if average_x > middle:
                if average_x < closest_to_middle_right:
                    closest_to_middle_right = average_x
                    
        for line in left_lines:
            x1, y1, x2, y2 = line[0]  # Extract line coordinates
            average_x = (x1+x2)/2
            
            if average_x < middle:
                if average_x > closest_to_middle_left:
                    closest_to_middle_left = average_x
                    
        right_lines_final = []
        left_lines_final = []

        for line in right_lines:
            x1, y1, x2, y2 = line[0]  # Extract line coordinates
            average_x = (x1+x2)/2
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))  # Calculate angle (slope)
            if abs(average_x-closest_to_middle_right) < 50:
                right_lines_final.append(line)
                    
        for line in left_lines:
            x1, y1, x2, y2 = line[0]  # Extract line coordinates
            average_x = (x1+x2)/2
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))  # Calculate angle (slope)
            if abs(average_x-closest_to_middle_left) < 50:
                left_lines_final.append(line)


        # if lines is not None:
        # 	for line in lines:
        # 		x1, y1, x2, y2 = line[0]
        # 		cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 2)
                    
                # cv2.line(edge_image, (x1, y1), (x2, y2), (255, 0, 0), 2)

        # for line in right_lines_final:
        #     x1, y1, x2, y2 = line[0]
        #     cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            
        # for line in left_lines_final:
        #     x1, y1, x2, y2 = line[0]
        #     cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
        def average_lines(lines):
            """
            Calculate the average slope, intercept, and endpoints for a group of lines.
            """
            avg_slope = np.mean([((y2 - y1) / (x2 - x1)) for line in lines for x1, y1, x2, y2 in line])
            avg_intercept = np.mean([(y1 - avg_slope * x1) for line in lines for x1, y1, x2, y2 in line])
            avg_x1 = np.mean([x1 for line in lines for x1, _, _, _ in line])
            avg_x2 = np.mean([x2 for line in lines for _, _, x2, _ in line])
            avg_y1 = avg_slope * avg_x1 + avg_intercept
            avg_y2 = avg_slope * avg_x2 + avg_intercept
            return (int(avg_x1), int(avg_y1), int(avg_x2), int(avg_y2))

        def extend_line(x1, y1, x2, y2, length):
            # Calculate the slope of the line
            dx = x2 - x1
            dy = y2 - y1
            if dx == 0:  # Vertical line
                if dy > 0:
                    y2 += length  # Extend downward
                    y1 -= length  # Extend upward
                else:
                    y2 -= length  # Extend upward
                    y1 += length  # Extend downward
            else:
                m = dy / dx
                # Calculate new endpoint coordinates in the positive direction
                x2_pos = int(x2 + length / np.sqrt(1 + m**2))  # Extend horizontally
                y2_pos = int(y2 + m * length / np.sqrt(1 + m**2))  # Extend vertically
                # Calculate new endpoint coordinates in the negative direction
                x2_neg = int(x2 - length / np.sqrt(1 + m**2))  # Extend horizontally
                y2_neg = int(y2 - m * length / np.sqrt(1 + m**2))  # Extend vertically
            return (x2_pos, y2_pos, x2_neg, y2_neg)
        
        if right_lines_final:
            avg_right_line = average_lines(right_lines_final)
            
            extended_right_line = extend_line(*avg_right_line, 1000)
            cv2.line(img, extended_right_line[0:2], extended_right_line[2:], (255, 255, 0), 2)
            cv2.line(img, avg_right_line[0:2], avg_right_line[2:], (255, 0, 0), 2)

        if left_lines_final:
            avg_left_line = average_lines(left_lines_final)
            
            extended_left_line = extend_line(*avg_left_line, 1000)
            cv2.line(img, extended_left_line[0:2], extended_left_line[2:], (255, 255, 0), 2)
            cv2.line(img, avg_left_line[0:2], avg_left_line[2:], (255, 0, 0), 2)

        def find_intersection(line1, line2):
            # Extract coordinates of the lines
            x1, y1, x2, y2 = line1
            x3, y3, x4, y4 = line2

            # Solve the simultaneous equations to find the intersection point
            det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
            if det == 0:  # Lines are parallel
                return None
            else:
                px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / det
                py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / det
                return int(px), int(py)
        if right_lines_final and left_lines_final:
            intersection = find_intersection(extended_left_line, extended_right_line)

            cv2.circle(img, (intersection[0], intersection[1]+75), 5, (0, 0, 255), -1)

            


            cone_location = PursuitLocationPixel()
            cone_location.u = float(intersection[0])
            cone_location.v = float(intersection[1]+75)
            self.cone_pub.publish(cone_location)


            debug_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    cone_detector = ConeDetector()
    rclpy.spin(cone_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
