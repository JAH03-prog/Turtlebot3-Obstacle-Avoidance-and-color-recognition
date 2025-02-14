#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from collections import deque

# Topics
laser_topic = "/scan"
cmd_vel_topic = "/cmd_vel"

# Parameters
SAFE_DISTANCE = 1.0  # Minimum safe distance from obstacles
HISTOGRAM_BINS = 180  # Number of bins in the histogram
MAX_LINEAR_SPEED = 0.2  # Maximum forward speed
MAX_ANGULAR_SPEED = 0.5  # Maximum turning speed
TARGET_ALIGNMENT_GAIN = 2.0  # Adjusts how much the robot aligns with the best path

# Publisher
pub = None

def bfs_find_best_direction(histogram):
    """
    Uses BFS (Breadth-First Search) to find the largest safe region in the histogram.
    The goal is to detect the widest gap between obstacles for the robot to navigate.
    """
    visited = set()
    largest_gap = []
    max_gap_size = 0

    # Perform BFS for each bin
    for i in range(HISTOGRAM_BINS):
        if histogram[i] == 0 and i not in visited:  # A safe bin and not visited
            queue = deque([i])
            current_gap = []

            while queue:
                bin_index = queue.popleft()
                if bin_index not in visited and histogram[bin_index] == 0:
                    visited.add(bin_index)
                    current_gap.append(bin_index)

                    # Check neighbors (wraps around for circular space)
                    left_neighbor = (bin_index - 1) % HISTOGRAM_BINS
                    right_neighbor = (bin_index + 1) % HISTOGRAM_BINS

                    if left_neighbor not in visited and histogram[left_neighbor] == 0:
                        queue.append(left_neighbor)
                    if right_neighbor not in visited and histogram[right_neighbor] == 0:
                        queue.append(right_neighbor)

            # Update the largest found gap
            if len(current_gap) > max_gap_size:
                max_gap_size = len(current_gap)
                largest_gap = current_gap

    if largest_gap:
        best_bin = largest_gap[len(largest_gap) // 2]  # Choose middle of the largest gap
    else:
        best_bin = HISTOGRAM_BINS // 2  # Default to forward direction if no gap found

    return best_bin

# LaserScan callback
def laser_callback(msg):
    global pub

    ranges = np.array(msg.ranges)
    ranges[ranges == 0] = np.inf  # Replace invalid readings with infinity

    # Compute the histogram of obstacle density
    angles = np.linspace(-np.pi, np.pi, len(ranges))
    histogram = np.zeros(HISTOGRAM_BINS)

    bin_size = len(angles) // HISTOGRAM_BINS
    for i, distance in enumerate(ranges):
        if distance < SAFE_DISTANCE:  # Obstacle detected
            bin_index = i // bin_size
            histogram[bin_index] += 1 / distance  # Closer obstacles contribute more

    # Smooth the histogram using a moving average filter
    histogram = np.convolve(histogram, np.ones(5) / 5, mode="same")

    # Find the best direction using BFS
    best_bin = bfs_find_best_direction(histogram)
    best_heading = -np.pi + (2 * np.pi / HISTOGRAM_BINS) * best_bin  # Convert bin to angle

    # Create the Twist message for movement
    twist = Twist()
    twist.linear.x = MAX_LINEAR_SPEED
    twist.angular.z = np.clip(best_heading, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

    pub.publish(twist)

if __name__ == "__main__":
    try:
        rospy.init_node("ObstacleAvoidanceNode", anonymous=True)
        rospy.Subscriber(laser_topic, LaserScan, laser_callback)
        pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

