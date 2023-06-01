import rclpy
from rclpy.node import Node
import tf_transformations

from cnnrobomaster.ThymioNode import ThymioNode
from cnnrobomaster.pid import PID
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

from std_msgs.msg import ColorRGBA

from copy import deepcopy
from enum import Enum
from math import sin, cos, inf
import random
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import os
import numpy as np

class ThymioState(Enum):
    # Initially, move straight until the robot reaches an obstacle
    FORWARD = 1
    # Check if the robot didn't stop fast enough and hit the obstacle
    BACKUP = 2
    #  Rotate in a random direction until the robot is clear from obstacles
    ROTATING = 3


class CameraController(ThymioNode):
    # Period of the update timer, set based on update frequencies of proximity sensors (10Hz) and odom (20Hz)
    UPDATE_STEP = 1/20

    # Max range of the Thymio's proximity sensors
    OUT_OF_RANGE = 0.12

    # Target distance of the robot from the wall at the end of FORWARD
    TARGET_DISTANCE = OUT_OF_RANGE - 0.04

    # Minimum distance from the wall to be able to rotate in place
    TOO_CLOSE = 0.05

    # Target difference between the distance measured by the two distance sensors
    TARGET_ERROR = 0.001
    
    def __init__(self):
        super().__init__('explore_controller', update_step=self.UPDATE_STEP)
        
        self.pose2d = np.array([0, 0.5, 0])
        # Initialize the state machine
        self.current_state = None
        self.next_state = ThymioState.FORWARD
        self.bridge = CvBridge()

        # Subscribe to all proximity sensors at the same time
        self.front_sensors = ["center_left", "center", "center_right"]
        self.lateral_sensors = ["left", "right"]
        self.rear_sensors = ["rear_left", "rear_right"]
        self.proximity_sensors = self.front_sensors + self.lateral_sensors + self.rear_sensors
        self.proximity_distances = dict()
        self.proximity_subscribers = [
            self.create_subscription(Range, f'proximity/{sensor}', self.create_proximity_callback(sensor), 10)
            for sensor in self.proximity_sensors
        ]

        self.camera_subscriber = self.create_subscription(Image, '/thymio0/camera', self.camera_callback, 10)

        # Create a publisher to send LED control messages
        self.led_pub = self.create_publisher(ColorRGBA, '/RM0002/leds/color', 10)

        # Create a ColorRGBA message and set the desired colors
        led_msg = ColorRGBA()
        led_msg.r = 0.0  # Red color intensity (0.0-1.0)
        led_msg.g = 0.0  # Green color intensity (0.0-1.0)
        led_msg.b = 255.0  # Blue color intensity (0.0-1.0)
        led_msg.a = 0.0  # Alpha channel (transparency)

        # Publish the LED control message
        self.led_pub.publish(led_msg)



    def camera_callback(self, msg):
        if (self.current_state == ThymioState.FORWARD) or (self.next_state == ThymioState.FORWARD):
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert the image to HSV color space
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define lower and upper thresholds for blue color in HSV
            lower_blue = np.array([100, 50, 50])
            upper_blue = np.array([130, 255, 255])

            # Create a mask for blue color
            mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            robot_in_image = False
            for contour in contours:
                # Calculate the bounding rectangle of the contour
                x, y, w, h = cv2.boundingRect(contour)

                # Draw a red rectangle around the contour
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                robot_in_image = True                

            # Specify the directory where the image should be saved
            save_directory = os.path.expanduser("~/Documents/robotics_cnn_images")
            if robot_in_image:

                distance_to_target = np.sqrt(np.power(self.pose2d[0]-(0.5), 2) + np.power(self.pose2d[1], 2))
                filename = os.path.join(save_directory, f"{distance_to_target}_{self.pose2d[2]}.png")

                cv2.imwrite(filename, cv_image)

                self.get_logger().debug(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*self.pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
            )

        else:
            pass


    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        
        self.get_logger().debug(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
        self.pose2d = pose2d

    def create_proximity_callback(self, sensor):
        # Create a callback function that has access to both the message and the name of the sensor that sent it
        def proximity_callback(msg):
            self.proximity_distances[sensor] = msg.range if msg.range >= 0.0 else inf
            
            self.get_logger().debug(
                f"proximity: {self.proximity_distances}",
                throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
            )
            
        return proximity_callback            
    
    def update_callback(self):
        # Wait until the first update is received from odometry and each proximity sensor
        if self.odom_pose is None or \
           len(self.proximity_distances) < len(self.proximity_sensors):
            return
        
        # Check whether the state machine was asked to transition to a new 
        # state in the previous timestep. In that case, call the initialization
        # code for the new state.
        if self.next_state != self.current_state:
            self.get_logger().info(f"state_machine: transitioning from {self.current_state} to {self.next_state}")
            
            if self.next_state == ThymioState.FORWARD:
                self.init_forward()
            elif self.next_state == ThymioState.BACKUP:
                self.init_backup()
            elif self.next_state == ThymioState.ROTATING:
                self.init_rotating()
            
            self.current_state = self.next_state
        
        # Call update code for the current state
        if self.current_state == ThymioState.FORWARD:
            self.update_forward()
        elif self.current_state == ThymioState.BACKUP:
            self.update_backup()
        elif self.current_state == ThymioState.ROTATING:
            self.update_rotating()
    
    def init_forward(self):
        self.stop()
    
    def update_forward(self):
        # Check if the robot reached an obstacle it cannot pass through.        
        if any(self.proximity_distances[sensor] < self.TARGET_DISTANCE for sensor in self.front_sensors):
            self.next_state = ThymioState.BACKUP
            return
            
        # Just move forward with constant velocity
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 0.3 # [m/s]
        cmd_vel.angular.z = 0.0 # [rad/s]
        self.vel_publisher.publish(cmd_vel)
    
    def init_backup(self):
        self.stop()

    def update_backup(self):
        # Check if the robot didn't stop fast enough and hit the wall
        if all(self.proximity_distances[sensor] > self.TOO_CLOSE for sensor in self.front_sensors):
            self.next_state = ThymioState.ROTATING
            return
            
        # Slowly back up to clear the obstacle
        cmd_vel = Twist() 
        cmd_vel.linear.x  = -0.1 # [m/s]
        cmd_vel.angular.z =  0.0 # [rad/s]
        self.vel_publisher.publish(cmd_vel)
        
    def init_rotating(self):
        self.stop()
        
        # Choose a random rotation direction to clear the obstacle
        self.turn_direction = random.sample([-1, 1], 1)[0]
    
    def update_rotating(self):
        if all(self.proximity_distances[sensor] == inf for sensor in self.front_sensors):
            self.next_state = ThymioState.FORWARD
            return
            
        # Just rotate in place with constant velocity
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 0.0 # [m/s]
        cmd_vel.angular.z = self.turn_direction * 3.0 # [rad/s]
        self.vel_publisher.publish(cmd_vel)
    
def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = CameraController()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()