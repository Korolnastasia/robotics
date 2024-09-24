import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        command = input()
        commands = {'turn_right': Twist(linear= Vector3(x=0.0, y=0.0, z=0.0), angular= Vector3(x=0.0, y=0.0, z=-1.5)), 'turn_left': Twist(linear= Vector3(x=0.0, y=0.0, z=0.0), angular= Vector3(x=0.0, y=0.0, z=1.5)), 'move_forward': Twist(linear= Vector3(x=1.5, y=0.0, z=0.0), angular= Vector3(x=0.0, y=0.0, z=0.0)), 'move_backward': Twist(linear= Vector3(x=-1.5, y=0.0, z=0.0), angular= Vector3(x=0.0, y=0.0, z=0.0))}
        
        if command not in commands.keys():
            msg = "Invalid command"
        else:
            msg = commands[command]
            self.publisher_.publish(msg)
            
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
