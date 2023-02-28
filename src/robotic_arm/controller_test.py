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
        self.timer = self.create_timer(1, self.timer_callback)

        self.joints = ['joint_1','joint_2','joint_4']
        self.goal_positions1 = [0.00, -0.00, 0.00]
        
    def timer_callback(self):

        link1_state = input('LINK 1: Enter "left" or "right": ')
        link2_state = input('LINK 2: Enter "down" or "up": ')
        link3_state = input('LINK 3: Enter "down" or "up": ')

        if link1_state == 'right':
            for i in range(len(self.goal_positions1)):
                self.goal_positions1[0] -= 0.1
        elif link1_state == 'left':
            for i in range(len(self.goal_positions1)):
                self.goal_positions1[0] += 0.1

        if link2_state == 'down':
            for i in range(len(self.goal_positions1)):
                self.goal_positions1[1] -= 0.1
        elif link2_state == 'up':
            for i in range(len(self.goal_positions1)):
                self.goal_positions1[1] += 0.1


        if link3_state == 'up':
            for i in range(len(self.goal_positions1)):
                self.goal_positions1[2] -= 0.1
        elif link3_state == 'down':
            for i in range(len(self.goal_positions1)):
                self.goal_positions1[2] += 0.1
        
        print("the last positions of links: ", self.goal_positions1)

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions1
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