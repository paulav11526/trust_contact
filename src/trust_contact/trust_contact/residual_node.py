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
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from messages.msg import ForceEvent
from std_msgs.msg import Float64MultiArray

class MomentumObserver(Node):
    def __init__(self):
        super().__init__('residual_node')
        self.publisher_ = self.create_publisher(WrenchStamped, 'ExternalForce', 10)
        self.publisher1 = self.create_publisher(ForceEvent, 'force_data', 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10) 
        self.create_subscription(Bool, 'contact_detection', self.contact_detection_callback, 10)
        self.create_subscription(Point, 'contact_point', self.contact_point_callback, 10) 

        # DELETE - publish: p, tau, non-lin terms, r, 
        self.pub_p = self.create_publisher(Float64MultiArray, '/observer/measured_momentum', 10)
        self.pub_tau = self.create_publisher(Float64MultiArray, '/observer/commanded_torque', 10)
        self.pub_nle = self.create_publisher(Float64MultiArray, '/observer/nonlinear_terms', 10)
        self.pub_integrand = self.create_publisher(Float64MultiArray, '/observer/integrand', 10)
        self.pub_integral = self.create_publisher(Float64MultiArray, '/observer/integral_state', 10)
        self.pub_residual = self.create_publisher(Float64MultiArray, '/observer/residual', 10)
        self.pub_tau_minus_nle = self.create_publisher(Float64MultiArray, '/observer/tau_minus_nle', 10)
        self.pub_qdot = self.create_publisher(Float64MultiArray, '/observer/qdot', 10)
        self.pub_q = self.create_publisher(Float64MultiArray, '/observer/q', 10)


        # contact detection variables
        self.force = None
        self.last_contact_state = False
        self.contact_start_time = 0.0
        self.end_time = None
        self.waiting_for_second = False
        self.timer_start = 0.0
        self.double_tap_gap_limit = 1.0

        self.contact_point = np.zeros(3) # contact point in peg coordinates

        # Joint sensor + residual variables
        self.q = None
        self.qdot = None
        self.tau = None
        self.integral_term = None
        self.r = None
        self.r_rev = None
        self.K = 100
        self.dt = 0.0057

        # ---------------------------------------------------------------------
        # 1. Load the Franka Emika Panda xacro
        package_share_dir = get_package_share_directory('trust_contact')
        self.urdf_path = os.path.join(package_share_dir, 'models', 'panda.urdf')

        # Load model
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        pin_map = {self.model.names[i]: i for i in range(len(self.model.names))}
        for name in pin_map:
            print(f"{name} -> Pin idx: {pin_map[name]}")

        self.ee_frame_name = "peg"  
        peg_id = self.model.getFrameId(self.ee_frame_name)
        parent_joint_id = self.model.frames[peg_id].parentJoint
        self.contact_frame = pin.Frame("contact_frane", parent_joint_id, parent_joint_id, pin.SE3(np.eye(3), self.contact_point), pin.FrameType.OP_FRAME)
        self.frame_id = self.model.addFrame(self.contact_frame)


    def joint_state_callback(self, msg: JointState):
        # values received from mujoco model
        self.q = np.array(msg.position)
        self.qdot = np.array(msg.velocity)
        self.tau = np.array(msg.effort)

        self.observer_step()

    def contact_detection_callback(self, msg: Bool):
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Contact Started (False -> True)
        if msg.data and not self.last_contact_state:
            self.contact_start_time = current_time
            self.get_logger().info('Contact Started')
            self.force = self.compute_force()
            self.get_logger().info(f'External Force of: {self.force}')
        
        # Contact Ended (True -> False)
        elif not msg.data and self.last_contact_state:
            self.end_time = current_time
            self.get_logger().info("Contact Ended")

            # First Contact Ending
            if not self.waiting_for_second:
                self.results = [self.force, self.contact_start_time, self.end_time, 0.0 , 0.0 , 0.0]
                self.waiting_for_second = True 
                self.timer_start = current_time 
            # Second Contact Ending
            else:
                self.results[3:] = [self.force, self.contact_start_time, self.end_time]
                self.waiting_for_second = False
                self.publish_force()

        
        # Timeout Check
        if not msg.data and self.waiting_for_second:
            if (current_time - self.timer_start) > self.double_tap_gap_limit:
                # passed gap limit
                if self.results[2] == 0:
                    self.results[2] = self.end_time
                self.waiting_for_second = False
                self.publish_force()
        
        self.last_contact_state = msg.data



    def publish_force(self):
        msg = ForceEvent()
        msg.force_magnitude_1 = self.results[0]
        msg.timestamp_1_start = Time(seconds=self.results[1]).to_msg()
        msg.timestamp_1_end = Time(seconds=self.results[2]).to_msg()
        msg.force_magnitude_2 = self.results[3]
        msg.timestamp_2_start = Time(seconds=self.results[4]).to_msg()
        msg.timestamp_2_end = Time(seconds=self.results[5]).to_msg()

        self.publisher1.publish(msg)
        self.get_logger().info('Published force data')
            
    
    def contact_point_callback(self, msg: Point):
        if msg:
            self.contact_point = np.array([msg.x, msg.y, msg.z]) # peg coordinates
            self.model.frames[self.frame_id].placement = pin.SE3(np.eye(3), self.contact_point)

    def observer_step(self):
        if self.q is None or self.qdot is None or self.tau is None: return # haven't received data yet
        
        pin.forwardKinematics(self.model, self.data, self.q)

        # Inertia Matrix
        pin.computeAllTerms(self.model, self.data, self.q, self.qdot)
        M = self.data.M
        M = M.copy() + 0.1*np.eye(len(M)) # to match mujoco model with 0.1armature
        b = self.data.nle # Non linear ters 
        C = pin.computeCoriolisMatrix(self.model, self.data, self.q, self.qdot)
        g = pin.computeGeneralizedGravity(self.model, self.data, self.q)

        #self.get_logger().info(f"Coriolis matrix: {C}")

        # Momentum 
        p = M @ self.qdot

        # Initialization
        if self.integral_term is None or self.integral_term_rev is None:
            self.p0 = p.copy()
            self.integral_term = np.zeros_like(p)
            self.integral_term_rev = np.zeros_like(p)
            self.r = np.zeros_like(p)
            self.r_rev = np.zeros_like(p)
            return self.r
        
        # Observer Update
        integrand = self.tau - b + self.r
        integrand_rev = self.tau + C.T@self.qdot - g + self.r_rev
        self.integral_term += self.dt* integrand
        self.integral_term_rev += self.dt*integrand_rev
        tau_g = self.tau - g
        

        self.r = self.K * (p - self.integral_term - self.p0) 
        self.r_rev = self.K * (p - self.integral_term_rev - self.p0)

        self.get_logger().info(f"r_rev: {self.r_rev}")

        self.publish_vector(self.pub_p, p)
        self.publish_vector(self.pub_tau, self.tau)
        self.publish_vector(self.pub_nle, tau_g)
        self.publish_vector(self.pub_integrand, integrand_rev)
        self.publish_vector(self.pub_integral, self.integral_term_rev)
        self.publish_vector(self.pub_residual, self.r_rev)

        
    # for publishing signals    
    def publish_vector(self, publisher, vec):
        msg = Float64MultiArray()
        msg.data = np.asarray(vec, dtype=np.float64).flatten().tolist()
        publisher.publish(msg)

    def compute_force(self):

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        # Jacobian at peg
        Jc = pin.computeFrameJacobian(self.model, self.data, self.q, self.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        Jc_v=Jc[:3, :]

        F_ext = pinv(Jc_v.T) @ self.r_rev
        #F_ext[3:] = [0.0, 0.0, 0.0]

        # Publish
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.ee_frame_name

        msg.wrench.force.x = F_ext[0]
        msg.wrench.force.y = F_ext[1]
        msg.wrench.force.z = F_ext[2]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: External Force = {F_ext}!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        
        return np.linalg.norm(F_ext)

            

def main(args=None):
    rclpy.init(args=args)
    node = MomentumObserver()


    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

