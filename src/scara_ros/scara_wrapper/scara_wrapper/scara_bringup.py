import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose

import time
from functools import partial
import numpy as np


class ScaraControl(Node):
    def  __init__(self):
        super().__init__('scara_control')

        self.publisher = self.create_publisher(Float64MultiArray, 'scara/target_positions', 10)
        self.joint_state_sub = self.create_subscription(JointState, 'scara/joint_states', self.update_joint_states, 10)
        self.pose_sub = self.create_subscription(Pose, 'scara/pose', self.update_pose, 10)

        self.goal_sub = self.create_subscription(Pose, 'supervisor/goal_pose', self.goal_callback, 10)

        self.end_effort_pose = Pose()
        self.joint_state = JointState()
        
    def update_joint_states(self, msg):
        self.joint_state = msg
    
    def update_pose(self, msg):
        self.end_effort_pose = msg
    
    def goal_callback(self, msg):
        x = msg.position.x
        y = msg.position.y
        x_test = 0.675
        y_test = -0.2301
        r = self.inverse_kinematic(x, y)
        self.get_logger().info("Angulos:"  + ' '+ str(r))
    
    def inverse_kinematic(self, x, y):
        a1 = 0.467  #link 1 size
        a2 = 0.4005 #link 2 size

        c2 = (x**2 + y**2 - a1**2 - a2**2)/(2*a1*a2)

        o21 = np.arctan2(-(1-c2**2)**(1/2), c2)

        b = np.arctan2(y, x)

        phi11 = np.arctan2(a2 * np.sin(o21), a1 + a2 * np.cos(o21))

        solution1 = [0, 0]
        solution1[1] = (o21 - np.pi * 50 / 180)# / np.pi * 180
        solution1[0] = (b - phi11)# / np.pi * 180

        return solution1

        

def main(args=None):
    rclpy.init(args=args)

    scaraController = ScaraControl()

    rclpy.spin(scaraController)

    scaraController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
