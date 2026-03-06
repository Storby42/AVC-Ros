import csv
import numpy as np

known_buckets = []
with open('buckets.csv', newline='\n') as csvfile:
    bucketreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    i = 0
    for row in bucketreader:
        print(float(row[0]) + float(row[1]), row[2])
        # print(', '.join(row))

# fusedOdom[]

# def id_bucket(relX, relY, color):
#             tx, ty = relX + np.cos(fusedOdom[2]), relY + np.sin(self.fusedOdom[2])

#             id = 0 # id of the closest bucket
#             for i in range(len(self.known_buckets)):
#                 if self.known_buckets[2] != color:
#                     if ((self.known_buckets[i][0] - tx)**2 + (self.known_buckets[i][1] - ty)**2) > self.known_buckets[id]:
#                         id = i
#             return id