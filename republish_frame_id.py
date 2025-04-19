# republish_with_new_frame.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import copy

class RepublishWithNewFrame(Node):
    def __init__(self):
        super().__init__('republish_with_new_frame')
        self.sub = self.create_subscription(
            PointCloud2, '/velodyne_points_2', self.callback, 10)
        self.pub = self.create_publisher(PointCloud2, '/velodyne_points_2_fixed', 10)

    def callback(self, msg):
        new_msg = copy.deepcopy(msg)
        new_msg.header.frame_id = 'velodyne_2'  # set a new frame
        self.pub.publish(new_msg)

def main():
    rclpy.init()
    rclpy.spin(RepublishWithNewFrame())
    rclpy.shutdown()
if __name__ == '__main__':
    main()