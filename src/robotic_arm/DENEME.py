import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import ikpy.chain as ikc
import numpy as np

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) #10hz
        self.kuka_robot = None
        self.robot_initialize('/home/gursel/robotics_ws/src/robotic_arm/urdf/robotic_arm.urdf') #replace with your URDF file

    def robot_initialize(self, urdf_file):
        self.kuka_robot = ikc.Chain.from_urdf_file(urdf_file)

    def inverse_kinematics(self, x, y, z):
        target_position = [x, y, z]
        target_orientation = [0, 0, 0, 1] #default orientation
        joint_angles = self.kuka_robot.inverse_kinematics(target_position, target_orientation)
        if joint_angles is not None:
            #publish joint trajectory
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ['joint_1', 'joint_2', 'joint_4']
            point = JointTrajectoryPoint()
            point.positions = joint_angles
            point.time_from_start = self.get_clock().now().to_msg() + rclpy.duration.Duration(seconds=1) #1 second duration
            trajectory_msg.points.append(point)
            self.pub.publish(trajectory_msg)

    def timer_callback(self):
        self.inverse_kinematics(0.5, 0.5, 0.5) #example target position

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
