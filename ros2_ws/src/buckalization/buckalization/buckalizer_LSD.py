import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.clock import Clock

from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose


class buckalizer_LSD_node(Node):
    def __init__(self):
        super().__init__('sync_node')
        qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/global', qos)
        self.det_pub = self.create_publisher(Detection3DArray, '/fused_vision_measurements', qos)

        self.timer = self.create_timer(1, self.TimerCallback)
        self.second_timer = self.create_timer(1.05, self.SecondTimerCallback)

    def TimerCallback(self):
        odom = Odometry()
        self.now = Clock().now().to_msg()

        odom.header.stamp = self.now
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = -7.126
        odom.pose.pose.position.y = 9.767
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = -0.259
        odom.pose.pose.orientation.w = 0.966

        print("dummy odom pub'd")
        self.odom_pub.publish(odom)

    def SecondTimerCallback(self):
        dets = Detection3DArray()
        self.now = Clock().now().to_msg()

        dets.header.stamp = self.now
        dets.header.frame_id = "map"
        
        det1 = Detection3D()
        det1.header.stamp = self.now
        det1.header.frame_id = "map"
        res1 = ObjectHypothesisWithPose()
        res1.pose.pose.position.x = 14.7
        res1.pose.pose.position.y = 13.2
        res1.hypothesis.score = .9
        res1.hypothesis.class_id = "1"
        det1.results.append(res1)
        dets.detections.append(det1)

        det2 = Detection3D()
        det2.header.stamp = self.now
        det2.header.frame_id = "map"
        res2 = ObjectHypothesisWithPose()
        res2.pose.pose.position.x = 14.7
        res2.pose.pose.position.y = 23.2
        res2.hypothesis.score = 0.7
        res2.hypothesis.class_id = "1"
        det2.results.append(res2)
        dets.detections.append(det2)

        print("dummy det pub'd")
        self.det_pub.publish(dets)

def main(args=None):
    rclpy.init(args=args)

    buckalizer_LSD = buckalizer_LSD_node()

    rclpy.spin(buckalizer_LSD)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    buckalizer_LSD.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()