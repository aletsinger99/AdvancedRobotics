
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
import rclpy
from rclpy.node import Node


class StopPublisher(Node):

    def __init__(self):
        super().__init__('stop_publisher')
        self.publisher_ = self.create_publisher(ServoCtrlMsg, '/deepracer_navigation_pkg/auto_drive', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = ServoCtrlMsg()
        msg.angle = 10.0
        msg.throttle= 0.0
        self.publisher_.publish(msg)
        print('Publishing drive command')
        self.get_logger().info('Publishing')
        self.i += 1


class StopService(Node):

    def __init__(self):
        super().__init__('stop_service')
        self.srv = self.create_service(ServoCtrlMsg, 'stop_servo', self.stop_servo_callback)

    def stop_servo_callback(self, request, response):
        if request.a == 'stop':
            stop_publisher = StopPublisher()
            rclpy.spin_once(stop_publisher)
            rclpy.spin_once(stop_publisher)
            

        return response



if __name__ == '__main__':
    main()