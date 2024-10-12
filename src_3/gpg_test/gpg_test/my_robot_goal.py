#(c) Make the robot move to the goal using the same strategy as in exercise
  
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import math
import random

class RandomGoal(Node):
    def __init__(self):
        super().__init__('random_goal_publisher')
        self.publisher_ = self.create_publisher(Pose2D, '/goal', 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.pose_callback, 10)
        self.current_pose = None
        self.goal = Pose2D()
        self.declare_parameter('goal_range', 1.0)
        self.declare_parameter('distance_threshold', 0.5)
        self.random_goal()

    def pose_callback(self, msg):
        #print(f'i am {msg}')
        self.current_pose = msg.pose
    	
        distance = math.sqrt((self.current_pose.pose.position.x - self.goal.x) ** 2 + (self.current_pose.pose.position.y - self.goal.y) ** 2)
        print(f'My distance is {distance}:)')

        if distance < self.get_parameter('distance_threshold').get_parameter_value().double_value:
            self.random_goal()
            
    def random_goal(self):
        goal_range = self.get_parameter('goal_range').get_parameter_value().double_value
        self.goal.x = random.uniform(0, goal_range)
        self.goal.y = random.uniform(0, goal_range)
        self.goal.theta = 0.0
        self.publisher_.publish(self.goal)


def main(args=None):
    rclpy.init(args=args)
    random_goal_node = RandomGoal()
    rclpy.spin(random_goal_node)
    random_goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        

