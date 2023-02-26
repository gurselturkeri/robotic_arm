import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
import time

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('robot_controller_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['joint_1','joint_2','joint_4']
        self.goal_positions = [1.17, -0.10, 1.74]
        

        
        

    def timer_callback(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints

        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=1)
        trajectory_msg.points.append(point)
        self.trajectory_publihser.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()

    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()