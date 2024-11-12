import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import time

class Depth_data(Node):
    def __init__(self):
        super().__init__('robot_depth')
        self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Image, '/depth/image', self.update_pose, 10)
        self.scan = Image()
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def update_pose(self, data):
    	self.scan = data

    def timer_callback(self):
        twist_msg = Twist()
        laser = self.scan.data
        twist_msg.linear.x = 0.0
        if (len(laser) != 0):
        	#self.get_logger().info(f'{laser[179]}')
        	tmp = int(laser[int(self.scan.width * self.scan.height/2 + self.scan.width/2)])
        	if (tmp==0.0 or tmp == 127 or tmp == 128):
                	twist_msg.linear.x = 0.5
        	else:
        		twist_msg.linear.x = 0.0
        twist_msg.linear.z = 0.0
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    time.sleep(2)
    node = Depth_data()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
