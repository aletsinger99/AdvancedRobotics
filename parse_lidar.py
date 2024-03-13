import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import String

from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from sensor_msgs.msg import LaserScan
class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(LaserScan, '/rplidar_ros/scan', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.angle_increment = 0
        self.angle_max = 0
        self.angle_min = 0
        self.frame_id = ""
        self.sec = 0
        self.nanosec = 0
        self.intensities = []
        self.range_min = 0
        self.range_max = 0
        self.ranges = []
        self.scan_time = 0
        self.time_increment = 0
        self.angles = []
        self.angle2range = {}
    def listener_callback(self, msg):
        self.get_logger().info('angle inc:'+str(msg.angle_increment))
        self.angle2range = {} # Clear the dict to throw out old lidar data
        self.angle_increment = msg.angle_increment
        self.angle_max = msg.angle_max
        self.angle_min = msg.angle_min
        self.frame_id = msg.header.frame_id
        self.sec = msg.header.stamp.sec
        self.nanosec = msg.header.stamp.nanosec
        self.intensities = msg.intensities
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.ranges = msg.ranges
        self.scan_time = msg.scan_time
        self.time_increment = msg.time_increment
        # Make a np array of angles
        self.angles = np.linspace(self.angle_min, self.angle_max, len(self.ranges))
        # Use this np array to make a dictionary mapping angles to ranges
        for i in range(len(self.angles)-1):
            self.angle2range[self.angles[i]] = self.ranges[i]
        
class DriveCommandPublisher(Node):

    def __init__(self):
        super().__init__('drive_command_publisher')
        self.publisher_ = self.create_publisher(ServoCtrlMsg, 'lidar_angle_thrott_cmds', 10)
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
        self.get_logger().info('Publishing drive command: Angle: '+str(msg.angle) + '  Throttle: '+str(msg.throttle))
        self.i += 1



def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = LidarSubscriber()
    drive_command_publisher = DriveCommandPublisher()
    speed = 0.4
    while(1):

        # Spin once to get a new batch of lidar data
        rclpy.spin_once(lidar_subscriber)
    
        # Get the first key as this point is closes to the bow
        keys = list(lidar_subscriber.angle2range.keys())
        q = keys[0]
        
        # Check lidar range to see how close you are to the wall. If you are near the wall stop
        if(lidar_subscriber.angle2range[q] < .3):
            drive_command_publisher.angle = 0.0
            drive_command_publisher.throttle = 0.2
            print('publishing stop')
        else:
            drive_command_publisher.angle = 0.0
            drive_command_publisher.throttle = speed
            # speed = speed + .005
        
        # Publish that command to the drive command topic which can be read by drive node
        rclpy.spin_once(drive_command_publisher)



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_subscriber.destroy_node()
    drive_command_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()