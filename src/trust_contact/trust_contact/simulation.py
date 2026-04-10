# mujoco + other dependencies
import mujoco
import mujoco.viewer
import numpy as np
import time
from ament_index_python.packages import get_package_share_directory
import os
from numpy import random 
import matplotlib.pyplot as plt
import pandas as pd
import threading

# ros dependencies
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray, Float64, Float32MultiArray



class ApplyContactForce:
    def __init__(self, model, data, df, random_row, contact_pub, point_pub, param_pub ,node):
        self.m = model
        self.d = data
        self.node = node

        # simulation details
        self.force_body = self.d.body("peg").id
        self.force_limit = 15  # maximum magnitude of each force component
        self.force_starttime = [500] # timesteps when force application begins
        self.force_windows = []
        self.prev_contact_active = False
        self.current_force_index = None
        self.active_force_world = None
        self.active_force_local = None
        self.active_force_point_local = None
        self.active_globalpoint = None

        self.logging_start = self.force_starttime[0] # timestep when sensor data logging starts
        self.logging_end = self.logging_start + 1000 # timestep when sensor data logging ends
        self.contact_detected = False

        # reading coordinates of body points from file and selecting random row of data
        self.df = df
        self.random_row = random_row

        self.publisher1 = contact_pub
        self.publisher2 = point_pub
        self.param_pub = param_pub

        # trust parameter
        self.C1 = None

    # setting random force vector
    def force_generation(self):
        # random point on peg
        point = [self.df.iloc[self.random_row,0], self.df.iloc[self.random_row,1], self.df.iloc[self.random_row,2]] #random point of force application

        # vector normal to point
        normal_vector = [self.df.iloc[self.random_row,3], self.df.iloc[self.random_row,4], self.df.iloc[self.random_row,5]]  #normal vector at point of force application, read from csv file
        # ensuring normal vector will always point outwards: point's x value and normal's x value must have the same sign etc. 
        if (point[0]<=0 and normal_vector[0]>=0) or (point[0]>=0 and normal_vector[0]<=0):
            normal_vector[0] *= -1
        if (point[1]<=0 and normal_vector[1]>=0) or (point[1]>=0 and normal_vector[1]<=0):
            normal_vector[1] *= -1
        if (point[2]<=0 and normal_vector[2]>=0) or (point[2]>=0 and normal_vector[2]<=0):
            normal_vector[2] *= -1
        
        # normalize
        normal_local = normal_vector / np.linalg.norm(normal_vector)

        # choose random magnitude
        magnitude = random.uniform(0.0, self.force_limit)
        self.node.get_logger().info(f'Magnitude: {magnitude}')

        # inward pushing force
        force_local = -magnitude * normal_local

        # Get contact_type for tap
        C2 = self.contact_type()

        return force_local, point, C2


    def apply_force(self, timestep, viewer, event):
        self.d.qfrc_applied[:] = 0.0
        contact_msg = Bool()

        active_event_index = self.get_active_event_index(timestep)
        event_active = (active_event_index is not None)
        
        if (not self.prev_contact_active) and event_active:
            start, end, tap_type, logical_event_id = self.event_windows[active_event_index]
            self.node.start_contact_event(self.tap_type_to_string(tap_type), logical_event_id)

        elif self.prev_contact_active and (not event_active):
            self.node.end_contact_event()

        active_index = self.get_active_force_index(timestep)
        if active_index is None:   # if we're not in a force window
            self.contact_detected = False
            self.current_force_index = None
            self.active_force_world = None
            self.active_force_local = None
            self.active_force_point_local = None
            self.active_globalpoint = None
            self.prev_contact_active = event_active

            # clear custom geoms
            viewer.user_scn.ngeom = 0
            viewer.sync()

            #publish contact detection
            contact_msg.data = self.contact_detected
            self.publisher1.publish(contact_msg)
            return None, None
        else:
            self.contact_detected = True
            start, end, tap_type, logical_event_id = self.force_windows[active_index]



        # entering a new force window -> generate new force/point once
        if self.current_force_index != active_index:
            self.current_force_index = active_index
            
            # generate local point + local normal force 
            force_local, force_point, C2 = self.force_generation()
            self.active_force_point_local = np.array(force_point, dtype=float)
            self.active_force_local = np.array(force_local, dtype=float)

            c_msg = Point()
            c_msg.x = float(self.active_force_point_local[0])
            c_msg.y = float(self.active_force_point_local[1])
            c_msg.z = float(self.active_force_point_local[2])
            self.publisher2.publish(c_msg)
            self.node.get_logger().info(f"Contact point at: {c_msg} in peg coordinates")

            # publish trust parameters
            trust_param_msg = Float32MultiArray()
            trust_param_msg.data = np.asarray( np.array([self.C1, C2]), dtype=float).flatten().tolist()
            self.param_pub.publish(trust_param_msg)

        # get global position and rotation of desired body
        xpos = self.d.xpos[self.force_body]
        xmat = self.d.xmat[self.force_body].reshape(3, 3)

        # convert point of force application to global coordinates
        self.active_globalpoint = xmat @ self.active_force_point_local + xpos
        self.active_force_world = xmat @ self.active_force_local

        mujoco.mj_applyFT(self.m, self.d, self.active_force_world, [0,0,0], self.active_globalpoint, self.force_body, self.d.qfrc_applied)
        
        # for visualization -------------------------------------------------------------------------------------------------
        # scaling force vector for visualization so arrow geom is not giant
        scaled_force_vector = self.active_force_world / 10.0
        arrow_start = self.active_globalpoint - scaled_force_vector # finding starting point for force vector's arrow 
        
        viewer.user_scn.ngeom = 0
        i=0
        # creating arrow to visualize direction of force applied
        mujoco.mjv_initGeom(viewer.user_scn.geoms[i], type=mujoco.mjtGeom.mjGEOM_ARROW, size=[0.01, 0.01, 0.01], pos= arrow_start, mat=np.eye(3).flatten(), rgba=[255, 0, 0, 1])
        mujoco.mjv_connector(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_ARROW, 0.008, arrow_start, self.active_globalpoint)
        i += 1
            
        # creating sphere to visualize point of force application
        mujoco.mjv_initGeom(viewer.user_scn.geoms[i], type=2, size=[0.008, 0.008, 0.008], pos= self.active_globalpoint, mat=np.eye(3).flatten(), rgba=[0, 0, 0, 1])
        i += 1
        viewer.user_scn.ngeom = i
        viewer.sync()
        # -----------------------------------------------------------------------------------------------------------------

        # logging
        self.node.get_logger().info(f"Force world: {self.active_force_world} | Norm:  {np.linalg.norm(self.active_force_world)}")
        self.node.get_logger().info(f"qfrc_applied: {self.d.qfrc_applied}")
        self.node.get_logger().info(f"Event type: {event}")
        self.node.get_logger().info(f"Current Event: {tap_type}")
        

        # publish bool for contact detection
        contact_msg.data = self.contact_detected
        self.publisher1.publish(contact_msg)

        self.prev_contact_active = event_active
        return self.active_globalpoint.copy(), self.active_force_world.copy()

    # trust parameters
    def contact_parameters(self):
        self.C1 = random.random() # trust parameter

    def contact_type(self):
        C2 = random.choice([0,1]) # palm or finger
        return C2

    # defining force windows and tap types
    
    def force_event_type(self):
        events = []
        num_events = 3
        for _ in range(num_events):
            event = np.random.choice([0, 1, 2]) #0 - long tap , 1 = double tap, 2 - single tap
            events.append(event)  
        return events

    def get_active_force_index(self, timestep):
        for i, (start,end, _, id) in enumerate(self.force_windows):
            if start <= timestep < end:
                return i
        return None

    def get_active_event_index(self, timestep):
        for i, (start, end, tap_type, logical_event_id) in enumerate(self.event_windows):
            if start <= timestep <= end:
                return i
        return None

    def force_application_time(self, events):
        self.force_windows = []
        self.event_windows = [] # for evaluation

        current_time = np.random.randint(2000, 2500)
        logical_event_id = 0

        for event in events:
            logical_event_id += 1
            if event == 1: # double tap
                tap_duration = 100
                gap = np.random.randint(50, 100)

                start1 = current_time
                end1 = start1 + tap_duration

                start2 = end1 + gap
                end2 = start2 + tap_duration

                # force windows
                self.force_windows.append((start1, end1, 1, logical_event_id))
                self.force_windows.append((start2, end2, 1, logical_event_id))

                # event windows
                self.event_windows.append((start1, end2, 1, logical_event_id))

                current_time = end2 + np.random.randint(1000, 2000)
            elif event == 2: # single tap
                tap_duration = 200

                start = current_time
                end = start + tap_duration

                self.force_windows.append((start, end, 2, logical_event_id))
                self.event_windows.append((start, end, 2, logical_event_id))
                current_time = end + np.random.randint(1000, 2000)
            else:
                tap_duration = 700 # long tap  

                start = current_time
                end = start + tap_duration

                self.force_windows.append((start, end, 0, logical_event_id))
                self.event_windows.append((start, end, 0, logical_event_id))
                current_time = end + np.random.randint(1000, 2000)

        self.node.get_logger().info(f'force window: {self.force_windows}')
        self.node.get_logger().info(f'event windows: {self.event_windows}')

    def tap_type_to_string(self, tap_type):
        mapping = {
            0: "Long_Tap",
            1: "Double_Tap",
            2: "Single_Tap"
        }
        return mapping[tap_type]

class RobotController:
    def __init__(self, model, data, qhome, node):
        self.model = model
        self.data = data
        self.node = node
        self.n_arm_joints = 7

        self.q_target = qhome
        self.speed_scale = 1.0
        self.motion_active = False

        # controller tuning
        self.Kp = np.array([2.0, 2.0, 2.0, 2.0, 1.5, 1.5, 1.5])   # position -> velocity gain
        self.max_joint_vel = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0])  # rad/s
        self.pos_tol = 0.05   # rad

        self.pending_q_target = None
        self.pending_speed = None

    def compute_velocity_command(self):
        if self.q_target is None:
            return np.zeros(self.n_arm_joints)
        
        q = self.data.qpos[:self.n_arm_joints].copy()
        q_error = self.q_target - q

        # stop if close enough
        if self.motion_active and np.linalg.norm(q_error) < self.pos_tol:
            self.motion_active = False
            #self.node.get_logger().info("Target reached")
            return np.zeros(self.n_arm_joints)
        
        # proportional position error -> desired velocity
        qdot_cmd = self.Kp * q_error
        
        # speed scaling
        vel_limit = self.speed_scale * self.max_joint_vel
        qdot_cmd = np.clip(qdot_cmd, -vel_limit, vel_limit)

        """         # minimum command to avoid creeping forever
        min_vel = 0.02 
        for i in range(self.n_arm_joints):
            if abs(q_error[i]) > self.pos_tol and abs(qdot_cmd[i]) < min_vel:
                qdot_cmd[i] = np.sign(qdot_cmd[i]) * min_vel """

        return qdot_cmd
    
    def apply_control(self):
        with self.node.state_lock:
            qdot_cmd = self.compute_velocity_command()

            # arm velocity actuators
            self.data.ctrl[:self.n_arm_joints] = qdot_cmd

    def try_activate_command(self):
        if self.pending_q_target is None or self.pending_speed is None:
            return

        self.q_target = self.pending_q_target.copy()
        self.speed_scale = self.pending_speed
        self.motion_active = True

        self.pending_q_target = None
        self.pending_speed_scale = None

        #self.node.get_logger().info(
        #    f"Activated new target with speed_scale={self.speed_scale:.2f}")        

class MujocoSimulatorNode(Node):
    def __init__(self):
        super().__init__('mujoco_simulator_node')
        # Load model
        # load robot model and data
        package_share_dir = get_package_share_directory('trust_contact')
        model_path = os.path.join(package_share_dir, 'models', 'scene.xml')
        pointcloud_path = os.path.join(package_share_dir, 'pointcloud', 'peg_points.asc')
        self.m = mujoco.MjModel.from_xml_path(model_path)
        self.d = mujoco.MjData(self.m)

        # set robot to home position
        self.q_home = [8.94154e-21, -0.566287, 0.00014964, -0.850776, -9.77076e-05, 1.79119, -1.53133e-05]
        mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
        #self.d.qpos[:7] = q_home
       # self.d.qvel[:] = [1.07488e-18, -2.65678e-14 ,-5.71869e-18, 2.687e-14 ,-5.14054e-18 ,-5.34417e-14 ,-1.40062e-18]
        #self.d.ctrl[:7] = [8.94154e-21, -0.566287 ,0.00014964, -0.850776 ,-9.77076e-05 ,1.79119 ,-1.53133e-05]
        mujoco.mj_forward(self.m, self.d)

        # reading coordinates of body points from file and selecting random row of data
        self.df = pd.read_csv(pointcloud_path, usecols = ["X", "Y", "Z", "Nx", "Ny", "Nz"])
        self.random_row = random.randint(len(self.df.index))

        # Publishers and Subscribers
        self.publisher1 = self.create_publisher(Bool, 'contact_detection', 10)
        self.publisher2 = self.create_publisher(Point, 'contact_point', 10)
        self.publisher3 = self.create_publisher(JointState, 'joint_states', 10)
        self.publisher4 = self.create_publisher(Float64MultiArray, '/observer/qfrc_applied', 10)
        self.force_pub = self.create_publisher(Float64MultiArray, '/observer/force_world', 10)
        self.jacobian_pub = self.create_publisher(Float64MultiArray, '/debug/contact_jacobian', 10)
        self.sim_time_pub = self.create_publisher(Float64, '/sim_time', 10)
        self.cont_param_pub = self.create_publisher(Float32MultiArray, 'contact_parameters', 10)
        self.create_subscription(JointState, 'q_target', self.target_callback, 10)
        self.create_subscription(Float64, 'predicted_speed', self.speed_callback, 10)

        # publishers for evaluation
        self.trial_pub = self.create_publisher(Int32, '/eval/trial_id', 10)
        self.event_pub = self.create_publisher(Int32, '/eval/event_id', 10)
        self.true_tap_pub = self.create_publisher(String, '/eval/true_tap', 10)
        self.event_active_pub = self.create_publisher(Bool, '/eval/event_active', 10)

        self.trial_id = 5
        self.event_id = 0
        self.current_true_tap = ""
        self.event_active = False

        
        # lock
        self.state_lock = threading.Lock()

        #Instantiate helper classes
        self.force_manager = ApplyContactForce(self.m, self.d, self.df, self.random_row, self.publisher1, self.publisher2, self.cont_param_pub , self)
        self.controller = RobotController(self.m, self.d, self.q_home, self)

    def publish_eval_metadata(self):
        trial_msg = Int32()
        trial_msg.data = self.trial_id
        self.trial_pub.publish(trial_msg)

        event_msg = Int32()
        event_msg.data = self.event_id
        self.event_pub.publish(event_msg)

        tap_msg = String()
        tap_msg.data = self.current_true_tap
        self.true_tap_pub.publish(tap_msg)
        
        active_msg = Bool()
        active_msg.data = self.event_active
        self.event_active_pub.publish(active_msg)

    def start_contact_event(self, tap_type: str, event_id):
        self.event_id = event_id
        self.current_true_tap = tap_type
        self.event_active = True
        self.publish_eval_metadata()
        self.get_logger().info(
            f"Started contact event: trial={self.trial_id}, event={self.event_id}, tap={tap_type}"
        )
        
    def end_contact_event(self):
        self.event_active = False
        self.publish_eval_metadata()
        self.get_logger().info(
            f"Ended contact event: trial={self.trial_id}, event={self.event_id}"
        )        

    # subscribes to target position
    def target_callback(self, msg:JointState):
        with self.state_lock:
            self.controller.pending_q_target = np.array(msg.position)
            self.controller.try_activate_command()
    
    # subscribes to target speed
    def speed_callback(self, msg:Float64 ):
        with self.state_lock:
            self.controller.pending_speed = msg.data
            self.controller.try_activate_command()

    def publish_jacobian(self, J):
        msg = Float64MultiArray()
        msg.data = np.asarray(J, dtype=np.float64).reshape(-1).tolist()
        self.jacobian_pub.publish(msg)

    def calculate_jacobian(self, globalpoint):
        jacp = np.zeros((3, self.m.nv))
        jacr = np.zeros((3, self.m.nv))

        # globalpoint = world point where force is applied
        mujoco.mj_jac(self.m, self.d, jacp, jacr, globalpoint, self.force_manager.force_body)

        Jc_v = jacp[:, :7].copy()
        self.publish_jacobian(Jc_v)
            

def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimulatorNode()
    msg = JointState()

    # determine what type of contact event this is
    contact_event = node.force_manager.force_event_type()
    node.force_manager.force_application_time(contact_event)

    # generate randoom trust parameter for duration of simulation
    node.force_manager.contact_parameters()

    # Launch the simulation
    with mujoco.viewer.launch_passive(node.m, node.d) as viewer:
        start_time = time.time()
        timestep = 0
        sensor_id = node.force_manager.force_body - 1
        sensor_readings  = pd.DataFrame(columns=['fx', 'fy', 'fz', 'tz']) # creating data frame to store sensor readings at a single joint

        # finding indexes of desired sensor readings in list of all sensor readings 'data.sensordata'
        fx = (sensor_id*6)-6
        fy = (sensor_id*6)-5
        fz = (sensor_id*6)-4
        tz = (sensor_id*6)-1 # torque sensor automatically reads torque in x,y,z but we only need tz since all joints only rotate in z direction

        while viewer.is_running():
            
            rclpy.spin_once(node, timeout_sec=0.001)
            # publish sim time
            msg_t = Float64()
            msg_t.data = float(node.d.time)
            node.sim_time_pub.publish(msg_t)

            node.controller.apply_control()

            timestep+=1
            dt = node.m.opt.timestep
            viewer.user_scn.ngeom = 0

            # Apply force
            globalpoint, force_world = node.force_manager.apply_force(timestep, viewer, contact_event)
            if globalpoint is not None:
                node.calculate_jacobian(globalpoint)

            if node.force_manager.logging_start < timestep <= node.force_manager.logging_end: # only recording sensor data at certain timesteps
                sensor_readings.loc[timestep] = [node.d.sensordata[fx], node.d.sensordata[fy], node.d.sensordata[fz], node.d.sensordata[tz]] # logging sensor readings in data frame

            # change to True to plot sensor readings
            if (False) and timestep == logging_end : 
                fx_list = sensor_readings['fx'].tolist()
                fy_list = sensor_readings['fy'].tolist()
                fz_list = sensor_readings['fz'].tolist()
                tz_list = sensor_readings['tz'].tolist()

                t = np.linspace(node.logging_start, node.logging_end, node.logging_end-node.logging_start)         
                fig, axs = plt.subplots(2, 2)
                axs[0, 0].plot(t, fx_list)
                axs[0, 1].plot(t, fy_list)
                axs[1, 0].plot(t, fz_list)
                axs[1, 1].plot(t, tz_list)

                axs[0, 0].set_title('Forces in x-axis in joint over time')
                axs[0, 1].set_title('Forces in y-axis in joint over time')
                axs[1, 0].set_title('Forces in z-axis in joint over time')
                axs[1, 1].set_title('Torque in joint over time')
                
                plt.show()

            # Step simulation
            mujoco.mj_step(node.m, node.d)

            # Publish joint state
            gen_torque = node.d.qfrc_actuator[:7] + node.d.qfrc_passive[:7]
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = [node.m.joint(i).name for i in range(node.m.njnt)]
            msg.position = node.d.qpos.tolist()
            msg.velocity = node.d.qvel.tolist()
            msg.effort = (gen_torque).tolist()

            node.publisher3.publish(msg)

            # publish qfrc applied
            msg1= Float64MultiArray()
            msg1.data = np.asarray(node.d.qfrc_applied[:7], dtype=np.float64).flatten().tolist()
            node.publisher4.publish(msg1)

            # publish force world
            for_msg = Float64MultiArray()
            for_msg.data = np.asarray(force_world, dtype=np.float64).flatten().tolist()
            node.force_pub.publish(for_msg)
            

            # Update viewer
            viewer.sync()
            time.sleep(dt)
            
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()