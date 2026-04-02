"""
This node gets triggered when a contact has been detected. A residual is calculated and approximated to be the external torque.
It will return the estiamted external force on the arm, and the time interval of the duration of the contact.
This is an implementation of the momentum observer proposed by De Luca (2006).
"""

from xacrodoc import XacroDoc
import subprocess
import pinocchio as pin
import numpy as np
from numpy.linalg import pinv
from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory
import os

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool

class MomentumObserver(Node):
    def __init__(self):
        super().__init__('residual_node')
        self.publisher_ = self.create_publisher(WrenchStamped, 'External_Force_Topic', 10) ## what should the queue size be??? 

        self.contact_detected = None
        self.q = None
        self.qdot = None
        self.tau = None
        self.integral_term = None
        self.prev_time = None
        self.r = None
        self.K = 1000
        self.dt = dt = 0.001
        self.create_timer(self.dt, self.observer_step)

        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10) 
        self.create_subscription(Bool, 'contact_detection', self.contact_detection_callback, 10)

        # ---------------------------------------------------------------------
        # 1. Load the Franka Emika xacro from franka_ros2
        # ---------------------------------------------------------------------

        self.urdf_path = "panda.urdf"

        # Load model
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        for i, name in enumerate(self.model.names):
            print(i, name)


    def joint_state_callback(self, msg: JointState):

        self.q = np.array(msg.position)
        self.qdot = np.array(msg.velocity)
        self.tau = np.array(msg.effort)

    def contact_detection_callback(self, msg: Bool):
        if msg.data:
            self.compute_force()

    def observer_step(self):
        if self.q is None or self.qdot is None or self.tau is None: return # haven't received data yet

        # Inertia Matrix
        pin.crba(self.model, self.data, self.q)
        M = self.data.M

        # Gravity Vector
        pin.computeGeneralizedGravity(self.model, self.data, self.q)
        g = self.data.g

        # Non linear terms 
        pin.nonLinearEffects(self.model, self.data, self.q, self.qdot)
        b = self.data.nle

        # Momentum 
        p = M @ self.qdot

        # Initialization
        if self.integral_term is None: 
            self.integral_term = p.copy()
            self.r = np.zeros_like(p)
            #self.prev_time = self.get_clock().now()
            return self.r
        
        # Observer Update
        integrand = self.tau - b + self.r
        self.integral_term += self.dt* integrand

        self.r = self.K * (p - self.integral_term)
        self.get_logger().info(f"||r|| = {np.linalg.norm(self.r):.3f}")
        

    def compute_force(self):
        ee_frame_name = "peg"   # or whatever the actual frame is
        frame_id = self.model.getFrameId(ee_frame_name)
        body = self.data.body("peg").id

        # Compute Jacobian at the end-effector
        J = pin.computeFrameJacobian(self.model, self.data, self.q, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        F_ext = pinv(J.T) @ self.r

        # Publish
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ee_frame_name

        msg.wrench.force.x = F_ext[0]
        msg.wrench.force.y = F_ext[1]
        msg.wrench.force.z = F_ext[2]

        msg.wrench.torque.x = F_ext[3]
        msg.wrench.torque.y = F_ext[4]
        msg.wrench.torque.z = F_ext[5]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {F_ext}')
        


    

def main(args=None):
    rclpy.init(args=args)
    node = MomentumObserver()


    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

