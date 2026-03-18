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
import math
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
import csv

class BuckalizationNode(Node):
    def __init__(self):
        super().__init__('bucket_localization_node')

        # Set up subscriber and publisher nodes
        # subscribe to technoblad-- i mean vision get bucket pos, and fused odom for position and heading
        self.subscription_vision = self.create_subscription(Vector3, '/vision_measurements', self.vision_callback, 10) # PLACEHOLDER NAMES
        self.subscription_fused_odom = self.create_subscription(Odometry, '/fused_odom', self.fusedOdom_callback, 10) # PLACEHOLDER NAMES

        self.publisher_buckalization = self.create_publisher(Odometry, '/buckalization', 10) # proper odometry object

        self.prev_time = self.get_clock().now().nanoseconds / 10**9 # initialize time checkpoint

        # tunable values
        self.MINIMUM_BUCKET_CONFIDENCE = .8 # percent val between 0-1

        self.known_buckets = []
        with open('buckets.csv', newline='\n') as csvfile:
            bucketreader = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in bucketreader:
                self.known_buckets.append(self.Bucket(float(row[0]), float(row[1]), row[2]))

        # variables for the calcs
        self.fusedOdom = None # stored as a 3 long tuple; x, y, yaw. calculated values.

        # set up output pose params
        self.visionX = 0.0
        self.visionY = 0.0
        self.visionYaw = 0.0

    class Bucket():
        relx = 0.0
        rely = 0.0
        worldx = 0.0
        worldy = 0.0
        color = ""
        confidence = 0.0
        id = None
        def __init__(self, x:float, y:float, color:str, confidence:float = None):
            self.x = x
            self.y = y
            self.color = color
            if confidence is not None:
                self.confidence = confidence
        def id_bucket(self, fusedOdom, known_buckets): # relative coords + color of bucket you are finding the "real" version of
            # abs cords of bucket by measurement
            #tx, ty = relX + fusedOdom[0]*np.cos(fusedOdom[2]), relY + fusedOdom[1]*np.sin(fusedOdom[2])
            self.worldx = fusedOdom[0] + np.cos(fusedOdom[2])*self.rely + np.sin(fusedOdom[2])*self.relx
            self.worldy = fusedOdom[1] + np.sin(fusedOdom[2])*self.rely - np.cos(fusedOdom[2])*self.relx
            print("bucket pos:", self.worldx, self.worldy)
            # split relx and rely into true x and true y

            # compare it to every
            id = 0 # id of the closest bucket
            for i in range(len(known_buckets)):
                if known_buckets[i].color == self.color: # use pseudo distance formula to find closest real counterpart
                    #print(f"squared dist from bucket {i}: {((known_buckets[i].worldx - self.worldx)**2 + (known_buckets[i].worldy - self.worldy)**2)}")
                    if ((known_buckets[i].worldx - self.worldx)**2 + (known_buckets[i].worldy - self.worldy)**2) < (known_buckets[id].worldx - self.worldx)**2 + (known_buckets[id].worldy - self.worldy)**2:
                        id = i
            # return id as index for real bucket
            self.id = id
            return id

        # if (bucket confidence > self.MINIMUM_BUCKET_CONFIDENCE):
            # if (buckets seen > 2):

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.visionX
        msg.pose.pose.position.y = self.visionY
        x, y, z, w, = quaternion_from_euler(0, 0, self.visionYaw)
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = x
        msg.pose.pose.orientation.y = y
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w
        self.publisher_.publish(msg)
        
        # ASSUMED BUCKET STRUCTURE: a list called "buckets" with params of:
        # relX (float), relY (float), color (uppercase str), time (idk), confidence (float 0-1)
    def vision_callback(self, data): # there will be some rel x rel y color confidence
        if len(data.buckets) == 0:
            return
        
        # choose the bucket of highest confidence and find its real identity
        buckified = data # TODO: Make this actually translate whataver the data is in to a list of bucket objects
        buckets_by_con = sorted(buckified, key=lambda bucket: -bucket.confidence)
        for bucket in buckets_by_con:
            bucket.id_bucket(fusedOdom = self.fusedOdom, known_buckets=self.known_buckets)

        # 1. find translation to get bucket of highest confidence (BOHC) to
        # most plausible corresponding actual (based on known map) bucket position
        # transform car pos estimation based on this too
        # new car pos estimation is the fused odom plus the difference between where the bucket was measured and
        self.visionX = self.fusedOdom[0] - (buckets_by_con[0].relx - self.known_buckets[buckets_by_con[0].id].x)
        self.visionY = self.fusedOdom[1] - (buckets_by_con[0].rely - self.known_buckets[buckets_by_con[0].id].y)
        
        # 2. find rotation (around BOHC) to get the 2nd bucket into the angle that makes sense (again based on known map)
        if len(data.buckets) >= 2:
            #v1 = np.array[buckets_by_con[1].worldx - buckets_by_con[0].worldx, buckets_by_con[1].worldy - buckets_by_con[0].worldy]
            #v2 = np.array[self.known_buckets[buckets_by_con[1].id].worldx - self.known_buckets[buckets_by_con[0].id].worldx, self.known_buckets[buckets_by_con[1].id].worldy - self.known_buckets[buckets_by_con[0].id].worldy]
            # theta 1 is what the angle should be, theta 2 is what it is measured to be (global)
            theta1 = math.atan2(self.known_buckets[buckets_by_con[1].id].worldy - self.known_buckets[buckets_by_con[0].id].worldy, self.known_buckets[buckets_by_con[1].id].worldx - self.known_buckets[buckets_by_con[0].id].worldx)
            theta2 = math.atan2(buckets_by_con[1].worldy - buckets_by_con[0].worldy, buckets_by_con[1].worldx - buckets_by_con[0].worldx)
            ccw_rot = theta1 - theta2
        
        # 3. rotate the car's known position around that same place
        self.visionX, self.visionY = self.visionX*math.cos(ccw_rot) - self.visionY*math.sin(ccw_rot), self.visionY*math.cos(ccw_rot) + self.visionX*math.sin(ccw_rot)
        self.visionYaw += ccw_rot

        # publish new vision-based pose!
        self.publish_odometry()
    
    def fusedOdom_callback(self, data):
        self.fusedOdom = (data.pose.pose.position.x, data.pose.pose.position.y, 0)
        _, _, self.fusedOdom[2] = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        # yaw is in rad btw