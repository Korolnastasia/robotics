import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircularTurtle(Node):
  
  def __init__(self):
    super().__init__('circular_turtle')
    self.publisher_ = self.create_publisher(Twist, 'robot/cmd_vel', 10)
    self.timer_period = 0.1 
    self.timer = self.create_timer(self.timer_period, self.timer_callback)
    
  def timer_callback(self):
    msg = Twist()
   
    msg.linear.x = 2.0
    msg.angular.z = 1.0 
    self.publisher_.publish(msg)

def main(args=None):
  rclpy.init(args=args)

  circular_turtle = CircularTurtle()

  rclpy.spin(circular_turtle)
  circular_turtle.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

