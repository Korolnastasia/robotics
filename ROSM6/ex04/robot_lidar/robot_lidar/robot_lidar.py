import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class Lidar_data(Node):
    def __init__(self):
        super().__init__('robot_lidar')
        self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(LaserScan, '/robot/scan', self.update_pose, 10)
        self.scan = LaserScan()
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def update_pose(self, ranges):
    	self.scan = ranges

    def timer_callback(self):
        twist_msg = Twist()
        laser = self.scan.ranges
        twist_msg.linear.x = 0.0
        if (len(laser) != 0):
        	if (laser[180] < 0.5):
        		twist_msg.linear.x = 0.0
        	else:
        		twist_msg.linear.x = 0.5
        twist_msg.linear.z = 0.0
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    time.sleep(2)
    node = Lidar_data()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
