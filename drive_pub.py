import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from deepracer_interfaces_pkg.srv  import ActiveStateSrv, EnableStateSrv

class DrivePublisher(Node):

    def __init__(self):
        super().__init__('drive_publisher')
        self.publisher_ = self.create_publisher(ServoCtrlMsg, '/deepracer_navigation_pkg/auto_drive', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = ServoCtrlMsg()
        msg.angle = 0
        msg.throttle= 0
        self.publisher_.publish(msg)
        print('Publishing drive command')
        self.get_logger().info('Publishing')
        self.i += 1


class StartupClientAsync(Node):

    def __init__(self):
        super().__init__('startup_client_async')
        self.cli1 = self.create_client(ActiveStateSrv, '/ctrl_pkg/vehicle_state')
        self.cli2 = self.create_client(EnableStateSrv, '/ctrl_pkg/enable_state')
        while not self.cli1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Vehicle State service not available, waiting again...')

        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Enable State not available, waiting again...')

        self.req1 = ActiveStateSrv.Request()
        self.req2 = EnableStateSrv.Request()

    def send_request(self):
        self.req1.state = 1
        self.req2.is_active = True
        self.future1 = self.cli1.call_async(self.req1)
        print('State req sent')
        self.future2 = self.cli2.call_async(self.req2)
        print('Enable state req sent')

        rclpy.spin_until_future_complete(self, self.future1)
        print('State req resp rec')
        rclpy.spin_until_future_complete(self, self.future2)
        print('Enable state resp rec')
        
        print(self.future1.result())
        print(self.future2.result())
        return self.future1.result(), self.future2.result()


def main(args=None):
    rclpy.init(args=args)

    startup_client = StartupClientAsync()
    response = startup_client.send_request()
    
    drive_publisher = DrivePublisher()

    rclpy.spin_once(drive_publisher)
    rclpy.spin_once(drive_publisher)
    # rclpy.spin_once(drive_publisher)
    # rclpy.spin_once(drive_publisher)
    # rclpy.spin_once(drive_publisher)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    startup_client.destroy_node()
    drive_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()