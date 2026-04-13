'''
PURPOSE: use bucket relative coords and color + current estimated pose
to get a more accurate position and publish it (vision corrected pose)

INPUT:
    from vision: 3D detection array
    from fused (kalman thing) odom: ...Odometry
OUTPUT:
    PoseWithCovarianceStamped (effectively an odom to map transform)
'''

import rclpy
from rclpy.node import Node

from message_filters import Subscriber, ApproximateTimeSynchronizer

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray

from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory
import csv
import os
import math
import yaml

class BuckalizationNode(Node):
    def __init__(self):
        super().__init__('buckalization')

        # tunable values
        self.MINIMUM_BUCKET_CONFIDENCE = .1 # percent val between 0-1
        self.QUEUE_SIZE = 10
        self.MAX_DELAY = .1 # max time diff between det and odom in s
        self.BASE_LINK_OFFSET_X = .2 # in m. since the messages sent are relative to the left camera
        self.BASE_LINK_OFFSET_Y = .128

        # Set up subscriber and publisher nodes
        # subscribe to technoblad-- i mean vision get bucket pos, and fused odom for position and heading
        self.subscription_vision = Subscriber(self, Detection3DArray, '/fused_vision_measurements')
        self.subscription_fused_odom = Subscriber(self, Odometry, '/odometry/global')

        self.publisher_buckalization = self.create_publisher(PoseWithCovarianceStamped, '/buckalization', 10)

        self.time_sync = ApproximateTimeSynchronizer([self.subscription_vision, self.subscription_fused_odom],
                                                     self.QUEUE_SIZE, self.MAX_DELAY)
        self.time_sync.registerCallback(self.SyncCallback)

        # initialize the known buckets
        self.known_buckets = []
        self.color_lookup = {
            "yellow" : 1, # 0 
            "red" : 0, # 1
            "blue" : 2 # 2
        }

        parameters_file_path = os.path.join(get_package_share_directory('buckalization'), 'data')
        sv_lookup_file_path = os.path.join(parameters_file_path, 'scoreval_lookups.yaml')
        with open(sv_lookup_file_path, 'r') as stream:
            sv_yaml = yaml.safe_load(stream)

        self.scoreval_lookup= { # expected (width, height, min width, max width, min height, max height) in meters of buckets
            self.color_lookup["yellow"] : sv_yaml['short_bucket'],
            self.color_lookup["red"] : sv_yaml['tall_bucket'],
            self.color_lookup["blue"] : sv_yaml['short_bucket'],
            "relative_dist_limits" : sv_yaml['relative_dist_limits'], # (ideal, min, max) in meters. Ideal is wherever it does detections best.
            "min_confidence" : sv_yaml['min_confidence'], # min confidence
            "max_id_dist" : sv_yaml['max_id_dist'], #maximum distance in meters that the transform correction is allowed to be
            "max_intrabucket_dist_diff" : sv_yaml['max_intrabucket_dist_diff']
        } #should probably load this from a params file so we don't go insane while tuning it

        buckets_file_path = os.path.join(parameters_file_path, 'buckets.csv')
        with open(buckets_file_path, newline='\n') as csvfile:
            bucketreader = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in bucketreader:
                self.known_buckets.append(self.Bucket(worldx=float(row[0]), worldy=float(row[1]), color=self.color_lookup[row[2]]))
            print("Buckets loaded")
        print(sv_yaml['short_bucket'])
        print(sv_yaml['tall_bucket'])

        # variables for the calcs
        self.fusedOdom = None # stored as a 3 long list; x, y, yaw. calculated values.

        # set up output pose params
        self.visionX = 0.0
        self.visionY = 0.0
        self.visionYaw = 0.0

    class Bucket():
        # 0 = red, 1 = yellow, 2 = blue
        def __init__(self, color: int, relx: float = None, rely: float = None, detboxw: float = None, detboxh: float=None, worldx: float = None, worldy: float = None, confidence: float = None, scorevaldict: dict = None):
            self.color = color # color is a lowercase "yellow", "blue", etc
            self.confidence = confidence # (sent by vision network)
            # relx is forward, rely is left.
            self.relx = relx
            self.rely = rely
            # Calculated bounding box dimensions, can be used to sanity check blatantly incorrect detections
            self.detboxw = detboxw
            self.detboxh = detboxh
            # worldx is right, worldy is up
            self.worldx = worldx
            self.worldy = worldy
            self.id = -1# index of the corresponding real bucket
            self.id_dist = -1
            self.scores = {}
            self.scorevaldict = scorevaldict
            self.isvalid = True
            self.finalscore = 10000

        def id_bucket(self, fusedOdom, known_buckets:list): # relative coords + color of bucket you are finding the "real" version of
            # split relx and rely into world x and world y
            # FIXED FOR ACTUAL ROS REL COORD SYS
            self.worldx = fusedOdom[0] + math.cos(fusedOdom[2])*self.relx - math.sin(fusedOdom[2])*self.rely
            self.worldy = fusedOdom[1] + math.sin(fusedOdom[2])*self.relx + math.cos(fusedOdom[2])*self.rely
            print("bucket pos:", self.worldx, self.worldy)

            best_id = 0 # id of the closest bucket of the same color
            best_dist = float('inf')

            for i, bucket in enumerate(known_buckets):
                if bucket.color != self.color:
                    continue
                # use pseudo distance to find closest real counterpart
                dist = (bucket.worldx - self.worldx) ** 2 + (bucket.worldy - self.worldy) ** 2
                if dist < best_dist:
                    best_dist = dist
                    best_id = i
            
            # return id (index for real bucket)
            print(f"Best id: {best_id} at distance: {best_dist}")
            self.id = best_id
            self.id_dist = math.sqrt(best_dist)
            print("id", self.id)
            return best_id
        
        def compute_scores(self):

            #Filters & sanity checks

            #bbox size check, gets rid of the stupid single pixel detections            
            [expectedW, expectedH, minW, maxW, minH, maxH] = self.scorevaldict[self.color]
            if((minH <= self.detboxh <= maxH) and (minW <= self.detboxw <= maxW)):
                self.scores["size"] = ((expectedH - self.detboxh) ** 2 + (expectedW - self.detboxw) ** 2 )/(((maxH-minH)**2)+((maxW-minW)**2)) #lower score is better
            else:
                self.scores["size"] = -1 #If invalid, set score to -1
                self.isvalid = False
                print(f"Color {self.color} detection at ({self.worldx}, {self.worldy}) is being ignored for being out of size bounds.")

            #ignore detections that are too far away to be accurate
            distance_to_detection = math.sqrt(self.relx ** 2 + self.rely ** 2)
            idealdist, minreldist, maxreldist = self.scorevaldict["relative_dist_limits"]
            if(minreldist <= distance_to_detection <= maxreldist):
                self.scores["detection_range"] = abs(idealdist - distance_to_detection)/maxreldist
            else:
                self.scores["detection_range"] = -1
                self.isvalid = False
                print(f"Color {self.color} detection at ({self.worldx}, {self.worldy}) is being ignored for being too far away.")

            #Check that transform isn't absurdly massive
            if(self.id_dist <= self.scorevaldict["max_id_dist"]):
                self.scores["correction_score"] = self.id_dist/self.scorevaldict["max_id_dist"]
            else:
                self.scores["correction_score"] = self.id_dist
                #self.validDetection = False
                print(f"Mild warning: color {self.color} detection at ({self.worldx}, {self.worldy}) is pretty far from any known buckets.")
            
            #Ignore correction distance score for now, it probably makes more sense to have that after handling red buckets and doing transform stuff.

            #It's just a confidence threshold lol            
            if(self.confidence >= self.scorevaldict["min_confidence"]):
                self.scores["confidence"] = self.confidence
            else:
                self.scores["confidence"] = -1
                self.isvalid = False
                print(f"Color {self.color} detection at ({self.worldx}, {self.worldy}) is being ignored for poor detection quality.")
            
            #Weight and merge scores here, then set to self.finalscore and return.
            if self.isvalid:
                self.finalscore=float(math.log((1-self.scores["confidence"])+1)+math.log(self.scores["correction_score"]+1)+math.log(self.scores["detection_range"]+1)+math.log(self.scores["size"]+1))
                print(f"Valid color {self.color} detection at ({self.worldx}, {self.worldy}) found to have the final score {self.finalscore}.")

    def handle_ided_red(red1:Bucket, red2:Bucket):
        if red1.worldy > red2.worldy:
            red1.id = 7
            red2.id = 6
        else:
            red1.id = 6
            red2.id = 7

    def publish_poseWcovar(self):
        msg = PoseWithCovarianceStamped()
        # heading (timestamp is most important bit)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        # point
        msg.pose.pose.position.x = self.visionX
        msg.pose.pose.position.y = self.visionY
        msg.pose.pose.position.z = 0.0
        print(f"New position determined to be ({self.visionX}, {self.visionY}) at {self.visionYaw} rad")
        # quaternion (orientation)
        x, y, z, w, = quaternion_from_euler(0.0, 0.0, self.visionYaw)
        msg.pose.pose.orientation.x = x
        msg.pose.pose.orientation.y = y
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w
        # covariance
        # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        # FIND THE VARIANCES OF X Y YAW. Assumed 0 for z, pitch, roll, and covariances
        msg.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0
            ]
        self.publisher_buckalization.publish(msg)
    
    # dets are Detection3D arrays
    # relX (float), relY (float), color (uppercase str), time (idk), confidence (float 0-1)
    def SyncCallback(self, dets, odom): # there will be some rel x rel y color confidence
        print("Synced odometry and detections have been found.")
        if len(dets.detections) == 0:
            return
        
        self.fusedOdom = [odom.pose.pose.position.x, odom.pose.pose.position.y, 0]
        orientations = odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w
        _, _, self.fusedOdom[2] = euler_from_quaternion(orientations)
        # yaw is in rad btw

        # change the msg given by the vision node into nice Bucket objects
        buckified = []
        for detection in dets.detections:
            if detection.results[0].hypothesis.score > self.MINIMUM_BUCKET_CONFIDENCE: # (only if the confidence is high enough)
                buckified.append(self.Bucket(
                    color = int(detection.results[0].hypothesis.class_id),
                    confidence = detection.results[0].hypothesis.score,
                    relx = detection.results[0].pose.pose.position.x + self.BASE_LINK_OFFSET_X,
                    rely = detection.results[0].pose.pose.position.y + self.BASE_LINK_OFFSET_Y,
                    scorevaldict=self.scoreval_lookup,
                    detboxh=detection.bbox.size.z,
                    detboxw=detection.bbox.size.x
                ))
        
        # choose the bucket of highest confidence (and start keeping track of red buckets)
        redbucks = []
        for bucket in buckified:
            bucket.id_bucket(fusedOdom=self.fusedOdom, known_buckets=self.known_buckets)
            bucket.compute_scores()
            if bucket.isvalid==False:
                buckified.remove(bucket)
                continue
            if bucket.color == self.color_lookup["red"]:
                redbucks.append(bucket)

        if len(buckified)==0:
            return
        buckets_by_con = sorted(buckified, key=lambda bucket: -(float(bucket.finalscore)))
        
            
        del buckets_by_con[2:]

        if len(redbucks) >= 2:
            self.handle_ided_red(redbucks[0], redbucks[1])
            
            # calculate the difference between the bucket-to-bucket distances (known vs measured) for the 2 red bucks
            # if it's too off, throw it all away
            b2b_dist_diff = math.abs(math.sqrt((buckets_by_con[0].worldx - buckets_by_con[1].worldx)**2 + (buckets_by_con[0].worldy - buckets_by_con[1].worldy)**2)
                - 1.524) #math.sqrt((known_bucket_1.worldx - known_bucket_2.worldx)**2 + (known_bucket_1.worldy - known_bucket_2.worldy)**2))
            if b2b_dist_diff > self.scoreval_lookup['max_intrabucket_dist_diff']:
                print(f"Distances between red buckets found to be {b2b_dist_diff} off. That's too much. No new position will be calculated")
                return
        
        # 1. find translation to get bucket of highest confidence (BOHC) to most plausible corresponding actual (based on known map) bucket position. transform car pos estimation based on this too
        # new car pos estimation is the fused odom plus the difference between where the bucket was measured and where it actually is
        print(buckets_by_con[0].worldx)
        print(self.known_buckets[buckets_by_con[0].id].worldx)

        self.visionX = self.fusedOdom[0] - (buckets_by_con[0].worldx - self.known_buckets[buckets_by_con[0].id].worldx)
        self.visionY = self.fusedOdom[1] - (buckets_by_con[0].worldy - self.known_buckets[buckets_by_con[0].id].worldy)

        # 2. find rotation (around BOHC) to get the 2nd bucket into the angle that makes sense (again based on known map)
        if len(dets.detections) >= 2:
            known_bucket_1 = self.known_buckets[buckets_by_con[0].id]
            known_bucket_2 = self.known_buckets[buckets_by_con[1].id]

            # theta 1 is what the angle should be, theta 2 is what it is measured to be (global)
            theta1 = math.atan2(known_bucket_2.worldy - known_bucket_1.worldy, known_bucket_2.worldx - known_bucket_1.worldx)
            theta2 = math.atan2(buckets_by_con[1].worldy - buckets_by_con[0].worldy, buckets_by_con[1].worldx - buckets_by_con[0].worldx)
            ccw_rot = theta1 - theta2

            self.visionYaw = self.fusedOdom[2] + ccw_rot

            # 3. METHOD 1: rotate the car's known position around that same place. For rotation point = (xr, yr):
            # new x = (x-xr)*cos(theta) - (y-yr)*sin(theta) + xr
            # new y = (x-xr)*sin(theta) + (y-yr)*cos(theta) + yr
            self.visionX, self.visionY = ((self.visionX-known_bucket_1.worldx)*math.cos(ccw_rot) - (self.visionY-known_bucket_1.worldy)*math.sin(ccw_rot) + known_bucket_1.worldx,
                                (self.visionY-known_bucket_1.worldy)*math.cos(ccw_rot) + (self.visionX-known_bucket_1.worldx)*math.sin(ccw_rot) + known_bucket_1.worldy)
            
            # 3b. METHOD 2: do circle intersections. Closest one is prolly right
            # x1, y1, x2, y2 = self.get_intersections(known_bucket_1.worldx, known_bucket_1.worldy, math.sqrt(buckets_by_con[0].relx**2 + buckets_by_con[0].rely**2), known_bucket_2.worldx, known_bucket_2.worldy, math.sqrt(buckets_by_con[1].relx**2 + buckets_by_con[1].rely**2))
            # if (x1-self.fusedOdom[0])**2 + (y1-self.fusedOdom[1])**2 < (x2-self.fusedOdom[0])**2 + (y2-self.fusedOdom[1])**2:
            #     self.visionX = x1
            #     self.visionY = y1
            # else:
            #     self.visionX = x2
            #     self.visionY = y2

        # publish new vision-based pose!
        self.publish_poseWcovar()

    # Source - https://stackoverflow.com/a/55817881
    # Posted by mujjiga, modified by community. See post 'Timeline' for change history
    # Retrieved 2026-03-23, License - CC BY-SA 4.0
    def get_intersections(x0, y0, r0, x1, y1, r1):
        # circle 1: (x0, y0), radius r0
        # circle 2: (x1, y1), radius r1

        d=math.sqrt((x1-x0)**2 + (y1-y0)**2)
        
        # non intersecting
        if d > r0 + r1 :
            return None
        # One circle within other
        if d < abs(r0-r1):
            return None
        # coincident circles
        if d == 0 and r0 == r1:
            return None
        else:
            a=(r0**2-r1**2+d**2)/(2*d)
            h=math.sqrt(r0**2-a**2)
            x2=x0+a*(x1-x0)/d   
            y2=y0+a*(y1-y0)/d   
            x3=x2+h*(y1-y0)/d     
            y3=y2-h*(x1-x0)/d 

            x4=x2-h*(y1-y0)/d
            y4=y2+h*(x1-x0)/d
            
            return (x3, y3, x4, y4)

def main(args=None):
    rclpy.init(args=args)

    buckalization = BuckalizationNode()

    rclpy.spin(buckalization)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    buckalization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
