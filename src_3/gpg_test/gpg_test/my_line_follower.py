#https://di-sensors.readthedocs.io/en/master/examples/basic_lf.html
#https://github.com/ros/std_msgs/blob/kinetic-devel/msg/Int32MultiArray.msg
#http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int32MultiArray.html
#https://foxglove.dev/blog/how-to-use-ros2-parameters

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Pose2D
from gpg_remote_msgs.msg import State
from nav_msgs.msg import Odometry
import math
import random


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.create_subscription(State, '/state', self.line_callback, 10)

        self.declare_parameter('linear_speed', 0.04)
        self.declare_parameter('angular_speed', -(0.3))

        #self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        #self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

    def line_callback(self, msg):
        line_data = msg.line
        print(line_data)
        self.process_sensor_data(line_data)

    def process_sensor_data(self, line_data):
        velocity_msg = TwistStamped()

        left_sensors = sum(line_data[0:2])  
        center_sensor = line_data[2]        
        right_sensors = sum(line_data[3:5]) 
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

        if left_sensors > right_sensors:  
            velocity_msg.twist.angular.z = self.angular_speed 
        elif right_sensors > left_sensors:  
            velocity_msg.twist.angular.z = -(self.angular_speed) 
        else:
            velocity_msg.twist.angular.z = 0.0

        velocity_msg.twist.linear.x = self.linear_speed
        self.publisher_.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    line_follower_node = LineFollower()
    rclpy.spin(line_follower_node)
    line_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

