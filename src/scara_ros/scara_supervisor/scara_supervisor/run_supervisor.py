import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty

from custom_interfaces.srv import ArucoDetect
from custom_interfaces.srv import SetVel
from custom_interfaces.action import VelControl
from custom_interfaces.srv import GrabOrRelease

import numpy as np
import cv2
from functools import partial
import time
from scipy.spatial.transform import Rotation


Ts = 0.05
GoalPose = None
TargetJoint = None

from abc import ABC, abstractmethod

def inverse_kinematic(x, y):
    #compute
    return solution1

class SupervisorListener(Node):
    def __init__(self):
        super().__init__('supervisor_listener')
        self.group = MutuallyExclusiveCallbackGroup()
        self.goal_subscriber = self.create_subscription(Pose, 'supervisor/goal_pose', self.goal_callback, 10, callback_group=self.group)
    
    def goal_callback(self, msg):
        global GoalPose
        GoalPose = msg

class Command(ABC):
    @abstractmethod
    def execute(self):
        pass

class initial(Command):
    def __init__(self, node):
        self.node = node

        self._action_client = ActionClient(node, VelControl, 'conveyor/vel_controller')
        while not self._action_client.wait_for_server(1.0):
            node.get_logger().info("waiting for action service...")


    def execute(self):
        global GoalPose
        if GoalPose is not None:
            self.node.get_logger().info("\n Pose encontrada")
            x = GoalPose.position.x
            y = GoalPose.position.y
            if y > -0.3:
                self.node.get_logger().info("\n Parada solicitada")
                
                goal_msg = VelControl.Goal()
                goal_msg.vel = 0.0

                self.node.get_logger().info("\n Parada em andamento")
                future = self._action_client.send_goal_async(goal_msg)
                future.add_done_callback(self.get_result_callback)

                self.node.get_logger().info('Goal accepted :)')
                #self.node.get_logger().info(str(future.get_result_async()))
                time.sleep(6)
                #rclpy.spin_until_future_complete(self.node, future)
                self.node.get_logger().info("\n Parada concluida")
                return True
            else:
                self.node.get_logger().info("Pose não encontrada")
                return False
            
    def get_result_callback(self, future):
        result = future.result().status
        self.node.get_logger().info('\n Parada total:' + str(result) + '\n')
        
        
class MoveToGoal(Command):
    def __init__(self, node):
        self.node = node
        self.publisher = self.node.create_publisher(Float64MultiArray, 'scara/target_positions', 10)

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
    
    def execute(self):
        global GoalPose
        global TargetJoint

        if GoalPose is not None:
            
            x = GoalPose.position.x
            y = GoalPose.position.y
            s = inverse_kinematic(x, y)
            
            #self.node.get_logger().info("\n Posição buscada:" + str(GoalPose.position))
            msg = Float64MultiArray()
            z = 0.355 - GoalPose.position.z 

            quart = [GoalPose.orientation.x, GoalPose.orientation.y, GoalPose.orientation.z, GoalPose.orientation.w]
            re = Rotation.from_quat(quart)
            euler = re.as_euler('xyz', degrees=False) 
            w = euler[2] - np.pi * 50 /180

            self.node.get_logger().info("\n Angulos definidos:" + str(s) + " " + str(z) + " " + str(w))
            msg.data = [s[0], s[1], 0.0, w]
            TargetJoint = [s[0], s[1], z, w]
            self.publisher.publish(msg)
            time.sleep(8)
            return True
        else:
            return False
        
class GrabGoal(Command):
    def __init__(self, node):
        self.node = node
        self.publisher = self.node.create_publisher(Float64MultiArray, 'scara/target_positions', 10)

        self.grab_client = self.node.create_client(GrabOrRelease, 'scara/grab_release')

        while not self.grab_client.wait_for_service(1.0):
            self.node.get_logger().info("waiting for service GrabOrRelease...")
    
    def execute(self):
        global TargetJoint
        if TargetJoint is not None:
            self.node.get_logger().info("Grab goal\n")
            msg = Float64MultiArray()
            msg.data = TargetJoint
            self.node.get_logger().info("\n Posição buscada:" + str(TargetJoint))
            self.publisher.publish(msg)
            time.sleep(3)

            self.node.get_logger().info("\nGrab request\n")
            req = GrabOrRelease.Request()
            req.command = int(0)
            self.grab_client.call_async(req)

            t = TargetJoint.copy()
            t[2] = 0.0
            #t[3] = 0.0
            msg = Float64MultiArray()
            msg.data = t
            self.publisher.publish(msg)

            TargetJoint = None
            
            return True
        else:
            return False
        

class MoveToBox(Command):
    def __init__(self, node):
        self.node = node
        self.publisher = self.node.create_publisher(Float64MultiArray, 'scara/target_positions', 10)

        self.grab_client = self.node.create_client(GrabOrRelease, 'scara/grab_release')

        while not self.grab_client.wait_for_service(1.0):
            self.node.get_logger().info("waiting for service GrabOrRelease...")
    
    def execute(self):
        self.node.get_logger().info("MoveToBox\n")
        x = 0.046
        y = 0.557
        s = inverse_kinematic(x, y)
        
        msg = Float64MultiArray()
        z = 0.2

        msg.data = [s[0], s[1], 0.0, -np.pi * 50 /180]
        self.publisher.publish(msg)
        time.sleep(6)
        msg.data = [s[0], s[1], 0.2, -np.pi * 50 /180]
        self.publisher.publish(msg)

        time.sleep(5)
        self.node.get_logger().info("\nRelease request\n")
        req = GrabOrRelease.Request()
        req.command = int(1)
        self.grab_client.call_async(req)
        
        return True

class EndProcess(Command):
    def __init__(self, node):
        self.node = node
        self.publisher = self.node.create_publisher(Float64MultiArray, 'scara/target_positions', 10)
    
    def execute(self):
        self.node.get_logger().info("\n\nEnd\n")

        msg = Float64MultiArray()

        
        msg.data = [0.0, 0.0, 0.0, 0.0]
        self.publisher.publish(msg)
        return True
        

class DoNothing(Command):
    def __init__(self, node):
        self.node = node

    def execute(self):
        pass

class Context():
    def __init__(self, node):
        self.node = node
        self.state = initial(node)
        self.states = [DoNothing, EndProcess, MoveToBox, GrabGoal, MoveToGoal]

    def request(self):
        if(self.state.execute()):
            self.node.get_logger().info("Troca de estado\n")
            self.change_state()

    def change_state(self):
        state = self.states.pop()
        self.state = state(self.node)

class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        
        self.current_context = Context(self)
        
        self.timer = self.create_timer(Ts, self.control_loop)


    def control_loop(self):
        self.current_context.request()



def main(args=None):
    rclpy.init(args=args)

    listener = SupervisorListener()
    supervisor = Supervisor()

    executor = MultiThreadedExecutor()
    executor.add_node(listener)
    executor.add_node(supervisor)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        listener.destroy_node()
        supervisor.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
