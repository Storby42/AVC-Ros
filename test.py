import csv
import numpy as np

class Bucket():
    relx = 0.0
    rely = 0.0
    worldx = 0.0
    worldy = 0.0
    color = ""
    confidence = 0.0
    id = None
    def __init__(self, relx:float = None, rely:float = None, color:str = "", confidence:float = None, worldx:float = None, worldy:float = None):
        self.color = color

        if self.relx is not None and self.rely is not None:
            self.relx = relx
            self.rely = rely

        if self.worldx is not None and self.worldy is not None:
            self.worldx = worldx
            self.worldy = worldy

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

known_buckets = []
with open('buckets.csv', newline='\n') as csvfile:
    bucketreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in bucketreader:
        known_buckets.append(Bucket(worldx=float(row[0]), worldy=float(row[1]), color=row[2]))

buckified = [Bucket(relx=4.0, rely=2.0, color="yellow", confidence=.9), Bucket(relx=15.0, rely=25.0, color="yellow", confidence=.5)]
fakeodom = [0, 0, np.pi/2]

buckets_by_con = sorted(buckified, key=lambda bucket: -bucket.confidence)
for bucket in buckets_by_con:
    #print(bucket.relx)
    #bucket.id_bucket(fusedOdom = fakeodom, known_buckets=known_buckets)
    print(bucket.id_bucket(fusedOdom = fakeodom, known_buckets=known_buckets))