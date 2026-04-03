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

# ros dependencies
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray

class ApplyContactForce:
    def __init__(self, model, data, df, random_row, contact_pub, point_pub, node):
        self.m = model
        self.d = data
        self.node = node

        # simulation details
        self.force_body = self.d.body("peg").id
        self.force_limit = 15  # maximum magnitude of each force component
        self.application_time = 100 # how many timesteps force is applied for
        self.force_starttimes = [1000,5000] # timesteps when force application begins
        self.current_force_index = None
        self.active_force_world = None
        self.active_force_point_local = None
        self.active_globalpoint = None

        # for sensor logging
        self.logging_start = self.force_starttimes[0] # timestep when sensor data logging starts
        self.logging_end = self.logging_start + 1000 # timestep when sensor data logging ends
        self.contact_detected = False

        # reading coordinates of body points from file and selecting random row of data
        self.df = df
        self.random_row = random_row

        self.publisher1 = contact_pub
        self.publisher2 = point_pub

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

        # EDIT LATER

        return force_local, point

    def get_active_force_index(self, timestep):
        for i, start in enumerate(self.force_starttimes):
            if start < timestep < (start + self.application_time):
                return i
        return None

    def apply_force(self, timestep, viewer):
        self.d.qfrc_applied[:] = 0.0

        active_index = self.get_active_force_index(timestep)

        if active_index is None:
            self.contact_detected = False
            self.current_force_index = None
            self.active_force_world = None
            self.active_force_point_local = None
            self.active_globalpoint = None
            return None, None

        self.contact_detected = True

        # entering a new force window -> generate new force/point once
        if self.current_force_index != active_index:
            self.current_force_index = active_index

            # generate local point + local normal force
            force_local, force_point_local = self.force_generation()

            # scaling force vector for visualization so arrow geom is not giant
            scaled_force_vector = [force_local[0]/50, force_local[1]/50, force_local[2]/50]
            # finding starting point for force vector's arrow 
            local_arrow_start = [force_point_local[0]-scaled_force_vector[0], force_point_local[1]-scaled_force_vector[1], force_point_local[2]-scaled_force_vector[2]]

            self.active_force_point_local = np.array(force_point_local, dtype=float)

            xpos = self.d.xpos[self.force_body]
            xmat = self.d.xmat[self.force_body].reshape(3, 3)

            # transform to world
            self.active_force_world = xmat @ np.array(force_local, dtype=float)
            self.active_globalpoint = xmat @ self.active_force_point_local + xpos
            arrow_start = xmat @ local_arrow_start + xpos

            c_msg = Point()
            c_msg.x = float(self.active_force_point_local[0])
            c_msg.y = float(self.active_force_point_local[1])
            c_msg.z = float(self.active_force_point_local[2])
            self.publisher2.publish(c_msg)

        # apply force at every timestep
        mujoco.mj_applyFT(
            self.m,
            self.d,
            self.active_force_world,
            np.zeros(3),
            self.active_globalpoint,
            self.force_body,
            self.d.qfrc_applied
        )

        # visualization
        fn = np.linalg.norm(self.active_force_world)
        if fn > 1e-9:
            scale = 0.5 / fn
            arrow_vec = self.active_force_world * scale
            start = self.active_globalpoint - arrow_vec
            i = viewer.user_scn.ngeom

            if i < len(viewer.user_scn.geoms):
                mujoco.mjv_initGeom(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_ARROW,
                                    np.array([0.01, 0.01, 0.01]), np.zeros(3), np.eye(3).flatten(),
                                    np.array([1.0, 0.0, 0.0, 1.0]))
                mujoco.mjv_connector(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_ARROW, 0.008,
                                    np.array(start, dtype=np.float64),
                                    np.array(self.active_globalpoint, dtype=np.float64))
                i += 1

            if i < len(viewer.user_scn.geoms):
                mujoco.mjv_initGeom(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_SPHERE,
                                    np.array([0.008, 0.008, 0.008]), self.active_globalpoint,
                                    np.eye(3).flatten(), np.array([0.0, 1.0, 0.0, 1.0]))
                i += 1

            viewer.user_scn.ngeom = i +1    
        
        self.node.get_logger().info(f"Applied force: {self.active_force_world}")
        msg = Bool()
        msg.data = self.contact_detected
        self.publisher1.publish(msg)
        self.node.get_logger().info(f'Publishing: {self.contact_detected}')

        return self.active_globalpoint.copy(), self.active_force_world.copy()

    def contact_parameters(self):
        C1 = random.random() # trust parameter
        C2 = random.choice([0,1]) # palm or finger
        X = [[C1, C2]]
        return X

    def force_event_type(self):
        event = np.random.choice([0,1]) #1 = double tap
        return event

class RobotController:
    def __init__(self, model, data, qhome):
        self.model = model
        self.data = data
        
        self.q_des = qhome
        self.q_target = qhome

    def update(self):
        #Initialize desired pose
        if self.q_des is None:
            self.q_des = self.data.qpos.copy()

        self.q_des = self.q_target.copy()
        # apply control
        self.data.ctrl[:] = self.q_des
        
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
        q_home = [8.94154e-21, -0.566287, 0.00014964, -0.850776, -9.77076e-05, 1.79119, -1.53133e-05]
        self.d.qpos[:7] = q_home
        self.d.qvel[:] = 0
        self.d.ctrl[:7] = [8.94154e-21, -0.566287 ,0.00014964, -0.850776 ,-9.77076e-05 ,1.79119 ,-1.53133e-05]
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
        self.create_subscription(JointState, 'q_target', self.target_callback, 10)

        #Instantiate helper classes
        self.force_manager = ApplyContactForce(self.m, self.d, self.df, self.random_row, self.publisher1, self.publisher2, self)
        self.controller = RobotController(self.m, self.d, q_home)

    def target_callback(self, msg:JointState):
        self.controller.q_target = np.array(msg.position)
        self.controller.update()


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

    # generate random force on random point
    #force_vector, force_point = node.force_manager.force_generation()
    event = node.force_manager.force_event_type()

    # generate randoom trust parameter and contact type
    X = node.force_manager.contact_parameters()
    node.get_logger().info(f'Contact: {X}')

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

            timestep+=1
            dt = node.m.opt.timestep
            viewer.user_scn.ngeom = 0

            # set robot to home position
            #node.d.ctrl[:7] = q_home

            # Apply force
            globalpoint, force_world = node.force_manager.apply_force(timestep, viewer)
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