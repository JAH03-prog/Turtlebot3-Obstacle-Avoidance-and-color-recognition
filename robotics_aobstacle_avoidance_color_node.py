#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import os
import time
from collections import deque

# Topics
camera_topic = "/camera/rgb/image_raw"
cmd_vel_topic = "/safe_cmd_vel"

# Initialize global variables
bridge = CvBridge()
current_color_index = 0
color_order = ["orange", "white", "yellow", "black"]
color_ranges = {
    "orange": (np.array([10, 100, 100]), np.array([20, 255, 255])),
    "white": (np.array([0, 0, 200]), np.array([255, 50, 255])),
    "yellow": (np.array([20, 100, 100]), np.array([30, 255, 255])),
    "black": (np.array([0, 0, 0]), np.array([50, 50, 50]))
}
state = "SEARCHING"
goal_start_time = None
goal_number = 1
save_path = os.path.expanduser("~/colortask_images/")
os.makedirs(save_path, exist_ok=True)

# Publisher
pub = None

# BFS function to find the largest connected contour
def bfs_largest_contour(mask):
    """Breadth-First Search (BFS) to find the largest connected contour in the binary mask."""
    h, w = mask.shape
    visited = np.zeros((h, w), dtype=bool)
    largest_contour = []
    max_size = 0

    # Directions for moving in 8-connected neighborhood
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    for y in range(h):
        for x in range(w):
            if mask[y, x] > 0 and not visited[y, x]:  # If it's a part of a contour and not visited
                queue = deque([(y, x)])
                visited[y, x] = True
                contour = []

                while queue:
                    cy, cx = queue.popleft()
                    contour.append((cx, cy))

                    for dy, dx in directions:
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] > 0 and not visited[ny, nx]:
                            queue.append((ny, nx))
                            visited[ny, nx] = True

                # Update the largest contour if this one is bigger
                if len(contour) > max_size:
                    max_size = len(contour)
                    largest_contour = contour

    return largest_contour

# Save goal image with overlay
def save_goal_image(cv_image, cx, cy, elapsed_time):
    global goal_number
    cv2.putText(cv_image, f"Goal {goal_number}: ({cx}, {cy})",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(cv_image, f"Time: {elapsed_time:.2f}s",
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    image_path = os.path.join(save_path, f"goal{goal_number}.jpg")
    cv2.imwrite(image_path, cv_image)
    rospy.loginfo(f"Goal image saved: {image_path}")
    goal_number += 1

# Camera callback
def camera_callback(msg):
    global current_color_index, state, goal_start_time, goal_number

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_image = cv2.GaussianBlur(hsv_image, (5, 5), 0)

        lower, upper = color_ranges[color_order[current_color_index]]
        mask = cv2.inRange(hsv_image, lower, upper)

        # Find contours using BFS
        largest_contour = bfs_largest_contour(mask)
        cx, cy = -1, -1
        time_to_goal = 0

        if largest_contour:
            moments = cv2.moments(np.array(largest_contour))
            if moments["m00"] > 0:
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])
                area = len(largest_contour)  # Size of the largest contour found

                if state == "SEARCHING":
                    rospy.loginfo(f"Target {color_order[current_color_index]} detected!")
                    state = "MOVING"
                    if goal_start_time is None:
                        goal_start_time = time.time()

                elif state == "MOVING":
                    time_to_goal = time.time() - goal_start_time

                    if area > 80000:
                        rospy.loginfo(f"Target {color_order[current_color_index]} reached!")
                        state = "NEAR_TARGET"
                        save_goal_image(cv_image, cx, cy, time_to_goal)
                        current_color_index += 1

                        if current_color_index >= len(color_order):
                            rospy.loginfo("All targets reached!")
                            rospy.signal_shutdown("Task completed")
                        else:
                            rospy.loginfo(f"Next target: {color_order[current_color_index]}")
                            state = "SEARCHING"
                    else:
                        twist = Twist()
                        twist.linear.x = 0.2
                        twist.angular.z = (cx - 320) * -0.002
                        pub.publish(twist)
        else:
            if state == "SEARCHING":
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.3
                pub.publish(twist)

        cv2.putText(cv_image, f"Goal {goal_number}: ({cx}, {cy})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(cv_image, f"Time to goal: {time_to_goal:.2f}s",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(cv_image, f"Current Target: {color_order[current_color_index]}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("Camera Feed", cv_image)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

# Main function
if __name__ == "__main__":
    try:
        rospy.init_node("ColorTrackingNode", anonymous=True)
        rospy.Subscriber(camera_topic, Image, camera_callback)
        pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        rospy.spin()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass
