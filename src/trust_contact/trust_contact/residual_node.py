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
import threading

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
from std_msgs.msg import Float64

class MomentumObserver(Node):
    def __init__(self):
        super().__init__('residual_node')
        self.publisher1 = self.create_publisher(ForceEvent, 'force_data', 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10) 
        self.create_subscription(Bool, 'contact_detection', self.contact_detection_callback, 10)
        self.create_subscription(Point, 'contact_point', self.contact_point_callback, 10) 
        self.sim_time_sub = self.create_subscription(
            Float64,
            '/sim_time',
            self.sim_time_callback,
            10
        )

        # For Debugging - publish: p, tau, non-lin terms, r, 
        self.pub_p = self.create_publisher(Float64MultiArray, '/observer/measured_momentum', 10)
        self.pub_tau = self.create_publisher(Float64MultiArray, '/observer/commanded_torque', 10)
        self.pub_nle = self.create_publisher(Float64MultiArray, '/observer/nonlinear_terms', 10)
        self.pub_integrand = self.create_publisher(Float64MultiArray, '/observer/integrand', 10)
        self.pub_integral = self.create_publisher(Float64MultiArray, '/observer/integral_state', 10)
        self.pub_residual = self.create_publisher(Float64MultiArray, '/observer/residual', 10)
        self.pub_tau_minus_nle = self.create_publisher(Float64MultiArray, '/observer/tau_minus_nle', 10)
        self.pub_qdot = self.create_publisher(Float64MultiArray, '/observer/qdot', 10)
        self.pub_q = self.create_publisher(Float64MultiArray, '/observer/q', 10)
        self.pub_mj_tau = self.create_publisher(Float64MultiArray, '/observer/mj_tau', 10)
        self.create_subscription(Float64MultiArray, '/observer/force_world', self.mujoco_force_callback, 10) 
        self.jacobian_sub = self.create_subscription(Float64MultiArray,'/debug/contact_jacobian',self.jacobian_callback,10)

        # contact detection variables
        self.force = None
        self.last_contact_state = False
        self.contact_start_time = 0.0
        self.end_time = None
        self.waiting_for_second = False
        self.timer_start = 0.0
        self.double_tap_gap_limit = 1.0
        # flag to wait a bit before calculating force so that residual has time to stabilize
        self.sim_time = None
        self.contact_pending = False 
        self.contact_pending_time = 0.0
        self.force = 0.0
        self.force_estimation_delay = 0.01  # seconds (10 ms)

        self.contact_point = np.zeros(3) # contact point in peg coordinates
        self.mujoco_force =None # externally applied force from mujoco for debugging
        
        #lock
        self.state_lock = threading.Lock()
        # Joint sensor + residual variables
        self.q = None
        self.qdot = None
        self.tau = None
        self.r_rev = None
        self.integral_term_rev = None
        self.K = 100
        self.dt = 0.0057

        # ---------------------------------------------------------------------
        # 1. Load the Franka Emika Panda xacro
        package_share_dir = get_package_share_directory('trust_contact')
        self.urdf_path = os.path.join(package_share_dir, 'models', 'panda.urdf')

        # Load model
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.Jc_v = None
        pin_map = {self.model.names[i]: i for i in range(len(self.model.names))}
        for name in pin_map:
            print(f"{name} -> Pin idx: {pin_map[name]}")

        self.ee_frame_name = "peg"  
        peg_id = self.model.getFrameId(self.ee_frame_name)
        parent_joint_id = self.model.frames[peg_id].parentJoint
        self.contact_frame = pin.Frame("contact_frame", parent_joint_id, peg_id, pin.SE3(np.eye(3), self.contact_point), pin.FrameType.OP_FRAME)
        self.frame_id = self.model.addFrame(self.contact_frame)
    

    def mujoco_force_callback(self, msg: Float64MultiArray):
        if msg.data:
            self.mujoco_force = np.array(msg.data)

    def sim_time_callback(self, msg: Float64):
        with self.state_lock:
            self.sim_time = float(msg.data)
    
    # gets contact point 
    def contact_point_callback(self, msg: Point):
        if msg:
            cp = np.array([msg.x, msg.y, msg.z]) # peg coordinates
            with self.state_lock:
                self.contact_point = cp
                self.model.frames[self.frame_id].placement = pin.SE3(np.eye(3), self.contact_point)

    # receives q, qdot, tau, then triggers residual calculation
    def joint_state_callback(self, msg: JointState):
        # values received from mujoco model
        q_new = np.array(msg.position)
        qdot_new = np.array(msg.velocity)
        tau_new = np.array(msg.effort)

        # compute r using these values
        r_new = self.observer_step(q_new, qdot_new, tau_new)

        # store all related state together under one lock
        with self.state_lock:
            self.q = q_new.copy()
            self.qdot = qdot_new.copy()
            self.tau = tau_new.copy()
            self.r = r_new.copy()
            sim_time = self.sim_time
        
        if self.contact_pending:
            if sim_time is None:
                return

            if (sim_time - self.contact_pending_time) > self.force_estimation_delay:
                snapshot = self.get_force_snapshot()
                if snapshot is not None:
                    self.force = self.compute_force(snapshot)
                    self.get_logger().info(f'External Force of: {self.force}')
                self.contact_pending = False        

    # receives contact jacobian
    def jacobian_callback(self, msg: Float64MultiArray):
        J = np.array(msg.data, dtype=np.float64)

        if J.size != 21:
            self.get_logger().warning(f"Jacobian size wrong: expected 21, got {J.size}")
            return

        self.Jc_v = J.reshape(3, 7)

    # recieves bool for contact - triggers force calculation
    def contact_detection_callback(self, msg: Bool):
        with self.state_lock:
            sim_time = self.sim_time

        if sim_time is None:
            return

        current_time = sim_time

        # Contact Started (False -> True)
        if msg.data and not self.last_contact_state:
            self.contact_start_time = current_time
            self.get_logger().info('Contact Started')
            
            # trigger delayed force computation
            self.contact_pending = True
            self.contact_pending_time = current_time          

        
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

    # residual calculation
    def observer_step(self, q, qdot, tau):
        #if self.q is None or self.qdot is None or self.tau is None: return # haven't received data yet
        
        pin.forwardKinematics(self.model, self.data, q)

        # Inertia Matrix
        pin.computeAllTerms(self.model, self.data, q, qdot)
        M = self.data.M
        M = M.copy() + 0.1*np.eye(self.data.M.shape[0]) # to match mujoco model with 0.1armature
        b = self.data.nle.copy() # Non linear ters 
        C = pin.computeCoriolisMatrix(self.model, self.data, q, qdot).copy()
        g = pin.computeGeneralizedGravity(self.model, self.data, q).copy()

        #self.get_logger().info(f"Coriolis matrix: {C}")

        # Momentum 
        p = M @ qdot

        # Initialization
        if self.integral_term_rev is None:
            self.p0 = p.copy()
            self.integral_term_rev = np.zeros_like(p)
            self.r_rev = np.zeros_like(p)
            return self.r_rev.copy()
        
        # Observer Update using passed in state
        # integral term
        integrand_rev = tau + C.T @ qdot - g + self.r_rev
        self.integral_term_rev += self.dt * integrand_rev
        
        # r = K[p_meas - p_pred]
        self.r_rev = self.K * (p - self.integral_term_rev - self.p0)
        r_new = self.r_rev.copy()

        #self.get_logger().info(f"r_rev: {self.r_rev}")

        self.publish_vector(self.pub_p, p)
        self.publish_vector(self.pub_tau, tau)
        #self.publish_vector(self.pub_nle, )
        self.publish_vector(self.pub_integrand, integrand_rev)
        self.publish_vector(self.pub_integral, self.integral_term_rev)
        self.publish_vector(self.pub_residual, self.r_rev)

        return r_new

    # external force calculation    
    def compute_force(self, snapshot):
        q = snapshot["q"]
        r = snapshot["r"]
        contact_point = snapshot["contact_point"]

        # update contact point frame if point changes
        self.model.frames[self.frame_id].placement = pin.SE3(np.eye(3), contact_point)

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data, q)

        # Jacobian at peg
        Jc = pin.getFrameJacobian(self.model, self.data, self.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        Jc_v = Jc[:3, :]

        ## using CONTACT JACOBIAN from mujocoo 
        if self.Jc_v is None:
            self.get_logger().warning("Jacobian not received yet")
            return 0.0

        F_ext = pinv(self.Jc_v.T) @ r

        #F_ext = pinv(Jc_v.T) @ r

        # debugging check
        ## subscribe to mujoco_force and compare to qfrc_applied 
        mj_tau = self.Jc_v.T @ self.mujoco_force
        #publish
        tau_check = self.Jc_v.T @ F_ext
        self.get_logger().info(f'Publishing: External Force = {F_ext}')
        #self.get_logger().info(f'tau_check = {tau_check}')
        #self.get_logger().info(f'tau_check_mjforce = {mj_tau}')
        #self.get_logger().info(f'r_used = {r}')
        
        return np.linalg.norm(F_ext)

    # publishes force for RF
    def publish_force(self):
        msg = ForceEvent()
        msg.force_magnitude_1 = self.results[0]
        msg.timestamp_1_start = Time(seconds=self.results[1]).to_msg()
        msg.timestamp_1_end = Time(seconds=self.results[2]).to_msg()
        msg.force_magnitude_2 = self.results[3]
        msg.timestamp_2_start = Time(seconds=self.results[4]).to_msg()
        msg.timestamp_2_end = Time(seconds=self.results[5]).to_msg()

        self.publisher1.publish(msg)
        self.get_logger().info(f'Published force data: {msg}')
   
    # helper function to get snapshot of states 
    def get_force_snapshot(self):
        with self.state_lock:
            if self.q is None or self.r is None or self.contact_point is None:
                return None

            return {
                "q": self.q.copy(),
                "r": self.r.copy(),
                "contact_point": self.contact_point.copy(),
            }    


    # for publishing debugging signals    
    def publish_vector(self, publisher, vec):
        msg = Float64MultiArray()
        msg.data = np.asarray(vec, dtype=np.float64).flatten().tolist()
        publisher.publish(msg)

            

def main(args=None):
    rclpy.init(args=args)
    node = MomentumObserver()


    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

