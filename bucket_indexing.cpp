/*
GOAL: using relative position of buckets to the car,
get the coordinates of the car

This will eventually be a node
*/

#include <cmath>

// should subscribe to position (the one that's combined camera + encoders) and encoder odom;
// should keep a history of encoder based odom (or encoder might publish history itself?)

int buckets_position[8] = {}; // will contain the positions of buckets

struct Bucket // also could just use buckets_position all the time
{
    float x = -1;
    float y = -1;
    float dist = -1;
    int id = -1;
};

int identify_bucket(Bucket bucket) { 
    return 1;
    // determine which buckets are being seen based on current
    // estimated position (fused odom) + pose (yaw) + distances/angles/colors
    // store that, + stuff from vision, in the bucket object
}

std::tuple<float, float> Triangulate(Bucket bucket1, Bucket bucket2)
{
    // the math is with circles. Paul Bourke.
    // use bucket positions + distances
        // for 2 buckets: r1 and r2 for distances to car
    // return absolute coords

    float d = std::sqrt(pow(bucket1.x - bucket2.x, 2) + pow(bucket1.y - bucket2.y, 2)); // distance between the buckets

    // (r02 - r12 + d2 ) / (2 d)
    float a = (pow(r1, 2) - pow(r2, 2) + pow(d, 2))/(2*d);

    // P2 = P0 + a ( P1 - P0 ) / d
    float P2x = bucket1.x + (a*(bucket1.x - bucket1.x))/d;
    float P2y = bucket1.y + (a*(bucket1.y - bucket1.y))/d;

    float h = std::sqrt(pow(bucket1.dist, 2) + pow(a, 2));
    // x3 = x2 +- h ( y1 - y0 ) / d
    // y3 = y2 -+ h ( x1 - x0 ) / d
    float car_x = P2x + (h*(bucket2.y - bucket1.y))/d;
    float car_y = P2y + (h*(bucket2.x - bucket1.x))/d;
    return std::make_tuple(car_x, car_y);
}