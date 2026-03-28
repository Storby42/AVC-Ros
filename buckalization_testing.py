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
            self.id = None

        def id_bucket(self, fusedOdom, known_buckets:list): # relative coords + color of bucket you are finding the "real" version of
            # split relx and rely into world x and world y
            # FIXED FOR ACTUAL ROS REL COORD SYS
            self.worldx = fusedOdom[0] + math.cos(fusedOdom[2])*self.relx - math.sin(fusedOdom[2])*self.rely
            self.worldy = fusedOdom[1] + math.sin(fusedOdom[2])*self.relx + math.cos(fusedOdom[2])*self.rely
            print("bucket pos:", self.worldx, self.worldy)

            best_id = 0 # id of the closest bucket of the same color
            best_dist = float('inf')

            for i, bucket in enumerate(known_buckets):
                if bucket.color == self.color:
                    continue
                # use pseudo distance to find closest real counterpart
                dist = (bucket.worldx - self.worldx) ** 2 + (bucket.worldy - self.worldy) ** 2
                print(f"distance from point {i}: {dist}")
                if dist < best_dist:
                    best_dist = dist
                    best_id = i
            # return id as index for real bucket
            self.id = best_id
            return best_id

known_buckets = []
with open('buckets_testing.csv', newline='\n') as csvfile:
    bucketreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in bucketreader:
        known_buckets.append(Bucket(worldx=float(row[0]), worldy=float(row[1]), color=row[2]))
        print(f"loaded color {known_buckets[-1].color} bucket at ({known_buckets[-1].worldx}, {known_buckets[-1].worldy})")

buckified = [Bucket(relx=4.0, rely=2.0, color=1, confidence=.9), Bucket(relx=15.0, rely=25.0, color=1, confidence=.5)]
fakeodom = [0, 0, 0]

buckets_by_con = sorted(buckified, key=lambda bucket: -bucket.confidence)
for bucket in buckets_by_con:
    #print(bucket.relx)
    #bucket.id_bucket(fusedOdom = fakeodom, known_buckets=known_buckets)
    print(bucket.id_bucket(fusedOdom = fakeodom, known_buckets=known_buckets))