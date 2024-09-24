import rclpy
from rclpy.node import Node
import sys
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose
import math

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.timer_period = 0.1 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.target_x = float(sys.argv[1])
        self.target_y = float(sys.argv[2])
        self.target_theta = float(sys.argv[3]) * math.pi / 180 

        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg

    def timer_callback(self):
        if self.current_pose is None:
            return

        delta_x = self.target_x - self.current_pose.x
        delta_y = self.target_y - self.current_pose.y

        distance = math.sqrt(delta_x**2 + delta_y**2)
        target_angle = math.atan2(delta_y, delta_x)
        angle_diff = target_angle - self.current_pose.theta

        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        msg = Twist()

        if distance > 0:
            msg.linear = Vector3(x=distance, y=0.0, z=0.0)
            msg.angular = Vector3(x=0.0, y=0.0, z=angle_diff)
        else:
            msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
            msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
            self.get_logger().info("I'm here")

        self.publisher_.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

