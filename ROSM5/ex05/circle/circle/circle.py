import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircularTurtle(Node):

  def __init__(self, spiral=False):
    super().__init__('circular_turtle')
    self.publisher_ = self.create_publisher(Twist, 'robot/cmd_vel', 10)
    self.timer_period = 0.1 
    self.timer = self.create_timer(self.timer_period, self.timer_callback)
    self.spiral = spiral
    if spiral:
      self.linear_speed = 0.5
      self.angular_speed = 3.0
    self.linear_speed = 2.0
    self.angular_speed = 1.0

  def timer_callback(self):
    msg = Twist()
    
    if self.spiral:
      self.linear_speed += 0.02
      self.angular_speed -= 0.02 if self.angular_speed > 0.2 else 0.0 

    msg.linear.x = self.linear_speed
    msg.angular.z = self.angular_speed
    self.publisher_.publish(msg)

def main(args=None):
  rclpy.init(args=args)

  spiral_mode = False
  if sys.argv[1] != '0':
    spiral_mode = True
    print(sys.argv[1])

  circular_turtle = CircularTurtle(spiral=spiral_mode)

  rclpy.spin(circular_turtle)
  circular_turtle.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

