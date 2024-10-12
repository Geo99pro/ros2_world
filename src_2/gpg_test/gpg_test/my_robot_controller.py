import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Pose2D
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from angles import shortest_angular_distance
from tf_transformations import euler_from_quaternion

class MovePhysicalRobot(Node):
    def __init__(self):
        super().__init__('move_physical_robot')
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.pose_callback, 10)
        self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)

        self.goal_pose = None
        self.current_pose = None

        self.declare_parameter('linear_gain', 0.3)
        self.declare_parameter('angular_gain', 0.5)

    def goal_callback(self, msg):
        self.goal_pose = msg

    def pose_callback(self, msg):
        self.current_pose = msg
        if self.goal_pose is not None:
            self.move_physical_robot()

    def move_physical_robot(self):
        if self.goal_pose is None or self.current_pose is None:
            return 

        msg = TwistStamped()
        #https://stackoverflow.com/questions/74976911/create-an-odometry-publisher-node-in-python-ros2
        euclidean_distance = sqrt(pow(self.goal_pose.x - self.current_pose.pose.pose.position.x, 2) +
                                   pow(self.goal_pose.y - self.current_pose.pose.pose.position.y, 2))
        
        goal_angle = atan2(self.goal_pose.y - self.current_pose.pose.pose.position.y,
                           self.goal_pose.x - self.current_pose.pose.pose.position.x)

        orientation = self.current_pose.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        #print(quaternion)

        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        angle_to_goal = shortest_angular_distance(goal_angle, yaw)

        linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
        angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value

        velocity_msg = TwistStamped()
        velocity_msg.twist.linear.x = min(linear_gain * euclidean_distance, 0.1)
        velocity_msg.twist.angular.z = -angular_gain * angle_to_goal
        
        self.publisher_.publish(velocity_msg)
        #print(f"Current Pose: {self.current_pose.pose.pose.position}")
        #print(f"Goal Pose: {self.goal_pose}")

def main(args=None):
    rclpy.init(args=args)
    move_physical_robot = MovePhysicalRobot()
    rclpy.spin(move_physical_robot)
    move_physical_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

