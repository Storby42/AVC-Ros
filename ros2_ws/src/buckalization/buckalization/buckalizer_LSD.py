import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.clock import Clock

from message_filters import Subscriber, ApproximateTimeSynchronizer

from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray


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
        print("dummy odom pub'd")
        self.odom_pub.publish(odom)

    def SecondTimerCallback(self):
        det = Detection3DArray()
        self.now = Clock().now().to_msg()

        det.header.stamp = self.now
        print("dummy det pub'd")
        self.det_pub.publish(det)

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