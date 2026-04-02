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
import time

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
#from messages.msg import ForceEvent

class MomentumObserver(Node):
    def __init__(self):
        super().__init__('residual_node')
        self.publisher_ = self.create_publisher(WrenchStamped, 'ExternalForce', 10)
        #self.publisher_ = self.create_publisher(ForceEvent, 'force_data', 10)

        self.first_contact_time = None
        self.waiting_for_second_contact = False
        self.q = None
        self.qdot = None
        self.tau = None
        self.integral_term = None
        self.prev_time = None
        self.r = None
        self.K = 20
        self.dt = 0.0001
        self.contact_point = None # contact point in peg coordinates

        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10) 
        self.create_subscription(Bool, 'contact_detection', self.contact_detection_callback, 10)
        self.create_subscription(Point, 'contact_point', self.contact_point_callback, 10) 

        # ---------------------------------------------------------------------
        # 1. Load the Franka Emika Panda xacro
        # ---------------------------------------------------------------------

        package_share_dir = get_package_share_directory('trust_contact')
        self.urdf_path = os.path.join(package_share_dir, 'models', 'panda.urdf')

        # Load model
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        pin_map = {self.model.names[i]: i for i in range(len(self.model.names))}
        for name in pin_map:
            print(f"{name} -> Pin idx: {pin_map[name]}")


    def joint_state_callback(self, msg: JointState):
        # values received from mujoco model
        self.q = np.array(msg.position)
        self.qdot = np.array(msg.velocity)
        self.tau = np.array(msg.effort)

        self.observer_step()

    def contact_detection_callback(self, msg: Bool):
        current_time = time.time()
        if msg.data:
            # Case 1: First Contact
            if not self.waiting_for_second_contact:
                self.waiting_for_second_contact = True
                self.first_contact_time = current_time
               
                force1 = self.compute_force()
                self.results = [force1, [0, 0, 0]]  # place holder 
                #TODO: Add timestamps to force1 
                # TODO: publish as ForceEvent

            # Case 2: second contact within time period
            if current_time - self.first_contact_time <= 1.0:
                result2 = self.compute_force

                ## addd publication of force ---------------------------------

                self.waiting_for_second_contact = False

        # Case 3: no second contact
        if self.waiting_for_second_contact:
            if current_time - self.first_contact_time > 1.0:
                #publish results ---------------------------------------
                self.waiting_for_second_contact = False

            
    
    def contact_point_callback(self, msg: Point):
        if msg:
            self.contact_point = np.array([msg.x, msg.y, msg.z])

    def observer_step(self):
        if self.q is None or self.qdot is None or self.tau is None: return # haven't received data yet
        
        pin.forwardKinematics(self.model, self.data, self.q)

        # Inertia Matrix
        pin.computeAllTerms(self.model, self.data, self.q, self.qdot)
        M = self.data.M
        b = self.data.nle # Non linear terms 

        # Momentum 
        p = M @ self.qdot

        # Initialization
        if self.integral_term is None:
            self.p0 = p.copy()
            self.integral_term = np.zeros_like(p)
            self.r = np.zeros_like(p)
            return self.r
        
        # Observer Update
        alpha = 0.01
        integrand = self.tau - b + self.r
        self.integral_term += self.dt* integrand
        #self.integral_term *= (1 - alpha)

        self.r = self.K * (p - self.integral_term - self.p0) 
        self.get_logger().info(f"||r|| = {np.linalg.norm(self.r):.3f}")
        #self.get_logger().info(f"q pin: {self.q}")

        self.get_logger().info(f"tau: {self.tau}")
        self.get_logger().info(f"tau - b {(self.tau - b)}")
        

    def compute_force(self):

        ee_frame_name = "peg"   # or whatever the actual frame is
        frame_id = self.model.getFrameId(ee_frame_name)

        oMf = self.data.oMf[frame_id]
        p_frame = np.array(oMf.translation.reshape(3))
        offset = self.contact_point - p_frame

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        # Jacobian at peg
        J = pin.computeFrameJacobian(self.model, self.data, self.q, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        # transform jacobian
        X = np.block([ [np.eye(3), np.zeros((3,3))], [-self.skew(offset), np.eye(3)] ])

        # Jacobian at contact point
        J_point = X @ J

        F_ext = pinv(J_point.T) @ self.r

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
        self.get_logger().info(f'Publishing: External Force = {F_ext}')
        
        return np.linalg.norm(F_ext)

            

    def skew(self,v):
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])
    

def main(args=None):
    rclpy.init(args=args)
    node = MomentumObserver()


    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

