import csv
import numpy as np
import math

class Bucket():
    # 0 = red, 1 = yellow, 2 = blue
    def __init__(self, color: int, relx: float = None, rely: float = None, worldx: float = None, worldy: float = None, confidence: float = None):
        self.color = color # color is a lowercase "yellow", "blue", etc
        self.confidence = confidence # (sent by vision network)
        # relx is forward, rely is left.
        self.relx = relx
        self.rely = rely
        # worldx is right, worldy is up
        self.worldx = worldx
        self.worldy = worldy
        self.id = None # index of the corresponding real bucket

    def id_bucket(self, fusedOdom, known_buckets:list): # relative coords + color of bucket you are finding the "real" version of
        # split relx and rely into world x and world y
        # FIXED FOR ACTUAL ROS REL COORD SYS
        self.worldx = fusedOdom[0] + math.cos(fusedOdom[2])*self.relx - math.sin(fusedOdom[2])*self.rely
        self.worldy = fusedOdom[1] + math.sin(fusedOdom[2])*self.relx + math.cos(fusedOdom[2])*self.rely
        print("     bucket pos:", self.worldx, self.worldy)

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

        # return id as index for real bucket
        self.id = best_id
        return best_id

known_buckets = []
color_lookup = {
            "yellow" : 0,
            "red" : 1,
            "blue" : 2
        }

with open('buckets_testing_testing.csv', newline='\n') as csvfile:
    bucketreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in bucketreader:
        known_buckets.append(Bucket(worldx=float(row[0]), worldy=float(row[1]), color=color_lookup[row[2]]))
        print(f"loaded color {known_buckets[-1].color} bucket at ({known_buckets[-1].worldx}, {known_buckets[-1].worldy})")

# simulated vision data
buckified = [Bucket(relx=14.7, rely=13.2, color=0, confidence=.9), Bucket(relx=14.7, rely=23.2, color=0, confidence=.7)]
fakeodom = [-7.126, 9.767, -math.pi/6]

# choose the bucket of highest confidence and find its real identity
buckets_by_con = sorted(buckified, key=lambda bucket: -bucket.confidence)
for bucket in buckets_by_con:
    #print(bucket.relx)
    #bucket.id_bucket(fusedOdom = fakeodom, known_buckets=known_buckets)
    print(f"Bucket of confidence {bucket.confidence} matched to id {bucket.id_bucket(fusedOdom = fakeodom, known_buckets=known_buckets)}")

# 1. find translation to get bucket of highest confidence (BOHC) to
# most plausible corresponding actual (based on known map) bucket position
# transform car pos estimation based on this too
# new car pos estimation is the fused odom plus the difference between where the bucket was measured and where it actually is
visionX = fakeodom[0] - (buckets_by_con[0].worldx - known_buckets[buckets_by_con[0].id].worldx)
visionY = fakeodom[1] - (buckets_by_con[0].worldy - known_buckets[buckets_by_con[0].id].worldy)

print(f"first movement: {buckets_by_con[0].worldx - known_buckets[buckets_by_con[0].id].worldx}, {buckets_by_con[0].worldy - known_buckets[buckets_by_con[0].id].worldy}")
print(f"position after first move: {visionX, visionY}")

# 2. find rotation (around BOHC) to get the 2nd bucket into the angle that makes sense (again based on known map)
if len(buckified) >= 2:
    known_bucket_1 = known_buckets[buckets_by_con[0].id]
    known_bucket_2 = known_buckets[buckets_by_con[1].id]

    # theta 1 is what the angle should be, theta 2 is what it is measured to be (global)
    theta1 = math.atan2(known_bucket_2.worldy - known_bucket_1.worldy, known_bucket_2.worldx - known_bucket_1.worldx)
    theta2 = math.atan2(buckets_by_con[1].worldy - buckets_by_con[0].worldy, buckets_by_con[1].worldx - buckets_by_con[0].worldx)
    ccw_rot = theta1 - theta2

    print(f"amount of rotation: {ccw_rot*180/math.pi}")
    visionYaw = fakeodom[2] + ccw_rot
    
    # 3. METHOD 1: rotate the car's known position around that same place
    # rotation point = (xr, yr)
    # new x = (x-xr)*cos(theta) - (y-yr)*sin(theta) + xr
    # new y = (x-xr)*sin(theta) + (y-yr)*cos(theta) + yr
    visionX, visionY = ((visionX-known_bucket_1.worldx)*math.cos(ccw_rot) - (visionY-known_bucket_1.worldy)*math.sin(ccw_rot) + known_bucket_1.worldx,
                        (visionY-known_bucket_1.worldy)*math.cos(ccw_rot) + (visionX-known_bucket_1.worldx)*math.sin(ccw_rot) + known_bucket_1.worldy)
    
    # 3b. METHOD 2: do circle intersections. Closest one is prolly right
    # x1, y1, x2, y2 = get_intersections(known_bucket_1.worldx, known_bucket_1.worldy, math.sqrt(buckets_by_con[0].relx**2 + buckets_by_con[0].rely**2), known_bucket_2.worldx, known_bucket_2.worldy, math.sqrt(buckets_by_con[1].relx**2 + buckets_by_con[1].rely**2))
    # if (x1-fusedOdom[0])**2 + (y1-fusedOdom[1])**2 < (x2-fusedOdom[0])**2 + (y2-fusedOdom[1])**2:
    #     visionX = x1
    #     visionY = y1
    # else:
    #     visionX = x2
    #     visionY = y2

# publish new vision-based pose!
print(f"New estimated position: {visionX}, {visionY}, {visionYaw*180/math.pi} deg")