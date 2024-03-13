import rclpy
from rclpy.node import Node
import keyboard
from std_msgs.msg import String
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from deepracer_interfaces_pkg.srv  import ActiveStateSrv, EnableStateSrv

class DrivePublisher(Node):

    def __init__(self):
        super().__init__('drive_publisher')
        self.publisher_ = self.create_publisher(ServoCtrlMsg, '/deepracer_navigation_pkg/auto_drive', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.angle = 0
        self.throttle = 0

    def timer_callback(self):
        msg = ServoCtrlMsg()
        msg.angle = self.angle
        msg.throttle= self.throttle
        self.publisher_.publish(msg)
        # print('Publishing drive command')
        self.get_logger().info('Publishing: Angle: '+str(msg.angle) + ' Throttle: '+str(msg.throttle))
        self.i += 1

class ParsedLidarSubscriber(Node):

    def __init__(self):
        super().__init__('parsed_lidar_subscriber')
        self.subscription = self.create_subscription(
            ServoCtrlMsg,
            'lidar_angle_thrott_cmds',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.angle = 0.0
        self.throttle = 0.0

    def listener_callback(self, msg):
        self.get_logger().info('I heard: Angle: ' + str(msg.angle) +' Throttle: '+str(msg.throttle))
        self.angle = msg.angle
        self.throttle = msg.throttle
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
    
    # Send the startup requests twice just to make sure everything is enabled
    response = startup_client.send_request()
    response = startup_client.send_request()

    # Instantiate our driving publisher and our lidar subscriber that receives drive cmds
    drive_publisher = DrivePublisher()
    parsed_lidar_subscriber = ParsedLidarSubscriber()

    
    while (1):
        # Get updated command from the lidar parser node
        rclpy.spin_once(parsed_lidar_subscriber)
        # Set attributes of drive publisher to the received values
        drive_publisher.throttle = parsed_lidar_subscriber.throttle
        drive_publisher.angle = parsed_lidar_subscriber.angle
        # Spin the drive publisher
        rclpy.spin_once(drive_publisher)
    


    startup_client.destroy_node()
    drive_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()