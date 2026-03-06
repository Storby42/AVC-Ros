'''
PURPOSE: use bucket relative coords and color + current estimated pose
INPUT:
    from vision: bucket rel coords, bucket color, image timestamp, bucket confidence
    from fused (kalman thing) odom: history ~40ms?

to get a more accurate position and publish it (vision corrected pose)
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import numpy as np
import pandas as pd
import csv

class BuckalizationNode(Node):
    def __init__(self):
        super().__init__('bucket_localization_node')

        # Set up subscriber and publisher nodes
        # subscribe to technoblad-- i mean vision get bucket pos, and fused odom for position and heading
        # placeholder examples:
        # self.subscription_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        # self.subscription_mag = self.create_subscription(MagneticField, '/mag', self.mag_callback, 10)
        self.subscription_vision = self.create_subscription(Vector3, '/vision_measurements', self.vision_callback, 10) # PLACEHOLDER NAMES
        self.subscription_fused_odom = self.create_subscription(Odometry, '/fused_odom', self.fusedOdom_callback, 10) # PLACEHOLDER NAMES
        self.publisher_buckalization = self.create_publisher(Odometry, '/buckalization', 10) # output as [roll, pitch, yaw] angles

        self.prev_time = self.get_clock().now().nanoseconds / 10**9 # initialize time checkpoint

        # tunable values
        
        self.MINIMUM_BUCKET_CONFIDENCE = .8
        
        class Bucket():
            x = 0.0
            y = 0.0
            color = ""
            confidence = 0.0
            def __init__(self, x, y, color):
                self.x = x
                self.y = y
                self.color = color
            def __init__(self, x, y, color, confidence):
                self.x = x
                self.y = y
                self.color = color
                confidence 

        known_buckets = []
        with open('buckets.csv', newline='\n') as csvfile:
            bucketreader = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in bucketreader:
                known_buckets.append(Bucket(float(row[0]), float(row[1]), row[2]))

        # variables for the calcs
        self.fusedOdom = None # stored as a 3 long tuple

        # set up output pose params
        self.visionX = 0.0
        self.visionY = 0.0
        self.visionYaw = 0.0

        def id_bucket(relX, relY, color):
            tx, ty = relX + np.cos(self.fusedOdom[2]), relY + np.sin(self.fusedOdom[2])

            id = 0 # id of the closest bucket
            for i in range(len(self.known_buckets)):
                if self.known_buckets[2] != color:
                    if ((self.known_buckets[i][0] - tx)**2 + (self.known_buckets[i][1] - ty)**2) > self.known_buckets[id]:
                        id = i
            return id

        # if (bucket confidence > self.MINIMUM_BUCKET_CONFIDENCE):
            # if (buckets seen > 2):

        def publish_odometry(self):
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = self.visionX
            msg.pose.pose.position.y = self.visionY
            msg.pose.pose.position.z = 0
            msg.pose.pose.orientation.x = 0 # it wants quaternions but we AINT giving it that. this is just yaw
            msg.pose.pose.orientation.y = 0
            msg.pose.pose.orientation.z = self.visionYaw
            msg.pose.pose.orientation.w = 0
            self.publisher_.publish(msg)
        
        # ASSUMED BUCKET STRUCTURE: a list called "buckets" with params of:
        # relX (float), relY (float), color (uppercase str), time (idk), confidence (float 0-1)
        def vision_callback(self, data):
            if len(data.buckets) == 0:
                return
            
            # choose the bucket of highest confidence
            highest_confidence_bucket = None
            for i in range(len(data.buckets)):
                if data.buckets[i].confidence > highest_confidence_bucket.confidence:
                    highest_confidence_bucket = data.buckets[i]
            id_bucket(data)
        # 1. find translation to get bucket of highest confidence (BOHC) to
            # most plausible corresponding actual (based on known map) bucket position
            # transform car pos estimation based on this too
            if len(data.buckets) > 2:
                pass
        # 2. find rotation (around BOHC) to get the 2nd bucket into the angle that makes sense (again based on known map)
        # 3. rotate the car's known position around that same place

        def fusedOdom_callback(self, data):
            self.fusedOdom = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.z)