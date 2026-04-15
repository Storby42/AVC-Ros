import numpy as np
import csv

# ASSUMES THAT THE STARTING POSITION OF THE CAR IS ALWAYS CALLED (0, 0, 0)

#buckets=np.array([[80,20,'yellow'], [50,25,'yellow'],[50,35,'blue'],[20,20,'yellow'],[20,60,'yellow'],[50,57.5,'red'],[50,62.5,'red'],[80,60,'yellow']])
# CHANGE THINGS HERE
buckets=np.array([[80,20], [50,25],[50,35],[20,20],[20,60],[50,57.5],[50,62.5],[80,60]]) # in ft
bucket_colors = ['yellow', 'yellow', 'blue', 'yellow', 'yellow', 'red', 'red', 'yellow']
starting_pos = np.array([[84], [40]]) # x y
starting_angle = -np.pi/2

# buckets=np.array([[35,40], [35,20],[20,40]])
# bucket_colors = ['blue', 'yellow', 'blue']
# starting_pos = np.array([[20], [20]]) # x y
# starting_angle = 0 # yaw
# dont change things down here maybe

rel_buckets = []

for bucket in buckets:
    trans_bucket = np.atleast_2d(bucket).T - starting_pos
    #print(trans_bucket)
    R = np.array([[np.cos(-starting_angle), -np.sin(-starting_angle)],
                   [np.sin(-starting_angle),  np.cos(-starting_angle)]])
    # #print(np.atleast_2d(trans_bucket))
    rot_bucket = R @ np.atleast_2d(trans_bucket)
    rel_buckets.append(rot_bucket*.3048) # ft to m

with open('calcd_buckets.csv', 'w', newline='\n') as csvfile:
    bucketwriter = csv.writer(csvfile, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
    for i in range(len(rel_buckets)):
        #print([rel_buckets[i][0][0]] + [','] + [rel_buckets[i][1][0]] + [bucket_colors[i]])
        bucketwriter.writerow([rel_buckets[i][0][0], rel_buckets[i][1][0], bucket_colors[i]])