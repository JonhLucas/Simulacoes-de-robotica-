import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose
from numpy import frombuffer, reshape, uint8
import numpy as np
import cv2
from cv_bridge import CvBridge
from cv_bridge.boost.cv_bridge_boost import getCvType

import threading
from functools import partial

class CameraSimul(Node):
    def __init__(self, camera_handle):
        super().__init__('camera_simul')
        self.camera_handle = camera_handle

        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None


        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, 'camera/image', 10)
        
        self.info_sub = self.create_subscription(CameraInfo, 'camera/camera_info', self.info_callback, 10)

        #f = sim.getObjectFloatParam(self.camera_handle, sim.visionfloatparam_pov_blur_distance)

        self.distCoeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
        self.cameraMatrix = np.array([[200, 0, 128],
                              [0, 200, 128],
                              [0,  0,  1]], dtype=np.float32)
        
        self.goal_pub = self.create_publisher(Pose, 'goal/pose', 10)
        

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(info_msg.k), (3, 3))
        self.distortion = np.array(info_msg.d)

        # Assume that camera parameters will remain the same
        self.destroy_subscription(self.info_sub)

 
    def goal_pose_publisher(self):
        aruco = sim.getObject('/arucoMarker')
        pose = sim.getObjectPose(aruco, -1)

        #pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg = Pose()
        msg.position.x = pose[0]
        msg.position.y = pose[1]
        msg.position.z = pose[2]
        msg.orientation.x = pose[3]
        msg.orientation.y = pose[4]
        msg.orientation.z = pose[5]
        msg.orientation.w = pose[6]
        
        # Publishing message
        self.goal_pub.publish(msg)

    def publish_image(self):

        image, resolution = sim.getVisionSensorImg(self.camera_handle)
        #self.get_logger().info(str(type(image)) + ' ' + str(len(image)))

        # Creating the Image structure
        msg = Image()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = resolution[0]
        msg.width = resolution[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * msg.width

        msg.data = image
        
        # Publishing message
        self.publisher.publish(msg)
    
def sysCall_init():
    sim = require('sim')

    camera_handle = sim.getObject('./sensor')

    rclpy.init()
    self.camera = CameraSimul(camera_handle)

def sysCall_sensing():
    self.camera.publish_image()
    self.camera.goal_pose_publisher()
    rclpy.spin_once(self.camera, timeout_sec=0)

def sysCall_cleanup():
    self.camera.destroy_node()
    rclpy.shutdown()
