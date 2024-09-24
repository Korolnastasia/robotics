from service_full_name.srv import SummFullName

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SummFullName, 'sum_full_name', self.sum_full_name_callback)

    def sum_full_name_callback(self, request, response):
        response.full_name = ' '.join([request.last_name, request.name, request.first_name])
        self.get_logger().info(f'Full name = {response.full_name}')

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
