import math
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtle_commands.action import MessageTurtleCommands

PI = 3.14

position = Pose()

class CommandsActionServer(Node):
    def __init__(self):
        super().__init__('action_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'execute_turtle_commands',
            self.execute_callback
        )
        
        global position

        self.x0, self.y0, t0 = position.x, position.y, position.theta

        
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        global position

        x0, y0, t0 = position.x, position.y, position.theta

        twist = Twist()
        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 0
        
        if goal_handle.request.command == 'forward':
            twist.linear.x = float(goal_handle.request.s)
            self.publisher.publish(twist)
            
            while feedback_msg.odom != goal_handle.request.s:
                feedback_msg.odom = int(math.sqrt((position.x - x0)**2 + (position.y - y0)**2))
                self.get_logger().info(f'I went: {position.x - self.x0} m')
                goal_handle.publish_feedback(feedback_msg)


        elif goal_handle.request.command == 'turn_right':
            twist.angular.z = -1.0 * float(goal_handle.request.angle) * PI / 180
            self.publisher.publish(twist)
            while abs(position.theta - t0) < float(goal_handle.request.angle) * PI / 180:
                self.get_logger().info(f'I rotated: {(position.theta - t0) * 180 / PI} degrees')
        else:
            twist.angular.z = float(goal_handle.request.angle) * PI / 180
            self.publisher.publish(twist)
            while abs(position.theta - t0) < float(goal_handle.request.angle) * PI / 180:
                self.get_logger().info(f'I rotated: {abs(position.theta - t0) * 180 / PI} degrees')

        goal_handle.succeed()
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        
        if goal_handle.request.command == 'forward':
            self.get_logger().info(f'Current position: {position.x} {position.y} m')
        else:
            self.get_logger().info(f'Current angle: {position.theta * 180 / PI} degrees')
        
        result = MessageTurtleCommands.Result()
        result.result = True
        return result

class CommandsActionSubscriber(Node):
    def __init__(self):
        super().__init__('action_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global position
        position = msg

def main(args=None):
    rclpy.init(args=args)
    
    try:
        action_server = CommandsActionServer()
        action_subscriber = CommandsActionSubscriber()
    
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(action_server)
        executor.add_node(action_subscriber)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            action_server.destroy_node()
            action_subscriber.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
