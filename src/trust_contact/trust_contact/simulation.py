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

class ApplyContactForce:
    def __init__(self, model, data, df, random_row, contact_pub, point_pub, node):
        self.m = model
        self.d = data
        self.node = node

        # simulation details
        self.force_body = self.d.body("peg").id
        self.force_limit = 5  # maximum magnitude of each force component
        self.application_time = 1000 # how many timesteps force is applied for
        self.force_starttime = 2000 # timesteps when force application begins
        self.logging_start = self.force_starttime # timestep when sensor data logging starts
        self.logging_end = self.logging_start + 1000 # timestep when sensor data logging ends
        self.contact_detected = False

        # reading coordinates of body points from file and selecting random row of data
        self.df = df
        self.random_row = random_row

        self.publisher1 = contact_pub
        self.publisher2 = point_pub

    # setting random force vector
    def force_generation(self):
        fx = random.uniform(-self.force_limit,self.force_limit)
        fy = random.uniform(-self.force_limit,self.force_limit)
        fz = random.uniform(-self.force_limit,self.force_limit)
        force_applied = [fx, fy, fz]
        force_point = [self.df.iloc[self.random_row,0], self.df.iloc[self.random_row,1], self.df.iloc[self.random_row,2]] #random point of force application
        return force_applied, force_point
 
    def normal2point(self, point):
        normal_vector = [self.df.iloc[self.random_row,3], self.df.iloc[self.random_row,4], self.df.iloc[self.random_row,5]]  #normal vector at point of force application, read from csv file
        # ensuring normal vector will always point outwards: point's x value and normal's x value must have the same sign etc. 
        if (point[0]<=0 and normal_vector[0]>=0) or (point[0]>=0 and normal_vector[0]<=0):
            normal_vector[0] *= -1
        if (point[1]<=0 and normal_vector[1]>=0) or (point[1]>=0 and normal_vector[1]<=0):
            normal_vector[1] *= -1
        if (point[2]<=0 and normal_vector[2]>=0) or (point[2]>=0 and normal_vector[2]<=0):
            normal_vector[2] *= -1
        return normal_vector

    def angle_btwn_vectors(self, A, B):
        dot_product = np.dot(A, B)
        magnitude_A = np.linalg.norm(A)
        magnitude_B = np.linalg.norm(B)
        angle_radians = np.arccos(dot_product / (magnitude_A * magnitude_B))
        angle_degrees = np.degrees(angle_radians)
        return angle_degrees

    def apply_force(self, force_point, force_vector, timestep, viewer):
        self.d.qfrc_applied[:] = 0.0
        
        # scaling force vector for visualization so arrow geom is not giant
        scaled_force_vector = [force_vector[0]/50, force_vector[1]/50, force_vector[2]/50]
        # finding starting point for force vector's arrow 
        local_arrow_start = [force_point[0]-scaled_force_vector[0], force_point[1]-scaled_force_vector[1], force_point[2]-scaled_force_vector[2]]
        
        # get global position and rotation of desired body
        xpos = self.d.xpos[self.force_body]
        xmat = self.d.xmat[self.force_body].reshape(3, 3)

        # convert point of force application to global coordinates
        globalpoint = xmat @ force_point + xpos
        arrow_start = xmat @ local_arrow_start + xpos
        c_msg = Point()
        c_msg.x = float(force_point[0])
        c_msg.y = float(force_point[1])
        c_msg.z = float(force_point[2])
        self.publisher2.publish(c_msg)




        if self.force_starttime <timestep< (self.force_starttime + self.application_time): # apply force at specified time interval
            mujoco.mj_applyFT(self.m, self.d, force_vector, [0,0,0], globalpoint, self.force_body, self.d.qfrc_applied)
            self.contact_detected = True
            i = viewer.user_scn.ngeom
            # creating arrow to visualize direction of force applied
            mujoco.mjv_initGeom(viewer.user_scn.geoms[i], type=mujoco.mjtGeom.mjGEOM_ARROW, size=[0.01, 0.01, 0.01], pos= arrow_start, mat=np.eye(3).flatten(), rgba=[255, 0, 0, 1])
            mujoco.mjv_connector(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_ARROW, 0.008, arrow_start, globalpoint)
            i += 1
                
            # creating sphere to visualize point of force application
            mujoco.mjv_initGeom(viewer.user_scn.geoms[i], type=2, size=[0.008, 0.008, 0.008], pos= globalpoint, mat=np.eye(3).flatten(), rgba=[0, 0, 0, 1])
            i += 1
            viewer.user_scn.ngeom = i
            viewer.sync()

            self.node.get_logger().info(f"Exerting a force of: {force_vector}")
            self.node.get_logger().info(f"qfrc_applied: {self.d.qfrc_applied}")
            self.node.get_logger().info(f"Contact point at: {c_msg} in peg coordinates")
        else:
            self.contact_detected = False
        
        msg = Bool()
        msg.data = self.contact_detected
        self.publisher1.publish(msg)
        self.node.get_logger().info(f'Publishing: {self.contact_detected}')
        #self.get_logger().info(f"q mujoco: {self.d.qpos}")

class RobotController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.q_des = None
        self.q_target = np.zeros(model.nu)
        #self.q_target = np.array([-5.57811e-22, 0.00096614, -2.6869e-07, -0.0695319, -2.45295e-05, 1.6, -1.53293e-05])

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
        # reading coordinates of body points from file and selecting random row of data
        self.df = pd.read_csv(pointcloud_path, usecols = ["X", "Y", "Z", "Nx", "Ny", "Nz"])
        self.random_row = random.randint(len(self.df.index))

        # Publishers and Subscribers
        self.publisher1 = self.create_publisher(Bool, 'contact_detection', 10)
        self.publisher2 = self.create_publisher(Point, 'contact_point', 10)
        self.publisher3 = self.create_publisher(JointState, 'joint_states', 10)
        self.publisher4 = self.create_publisher(Float64MultiArray, '/observer/qfrc_constraint', 10)
        self.create_subscription(JointState, 'q_target', self.target_callback, 10)

        #Instantiate helper classes
        self.force_manager = ApplyContactForce(self.m, self.d, self.df, self.random_row, self.publisher1, self.publisher2, self)
        self.controller = RobotController(self.m, self.d)

    def target_callback(self, msg:JointState):
        self.controller.q_target = np.array(msg.position)
        self.controller.update()


    

def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimulatorNode()
    force= [-2.3310970026575504, -2, 3]
    point= [np.float64(0.025166), np.float64(-0.000428), np.float64(0.094485)]
    msg = JointState()

    q_home = [8.94154e-21, -0.566287, 0.00014964, -0.850776, -9.77076e-05, 1.79119, -1.53133e-05]


    force_vector, force_point = node.force_manager.force_generation()
    normal_vector = node.force_manager.normal2point(force_point)
    angle = node.force_manager.angle_btwn_vectors(force_vector, normal_vector)
    # ensuring random force generated is a pushing force by checking angle between force vector and normal vector to the link's surface
    if angle <= 90 or angle >= 270: 
        while angle <= 90 or angle >= 270:
            force_vector, force_point = node.force_manager.force_generation() # if force is not a pushing force, a new force is generated until requirement is fulfilled
            normal_vector = node.force_manager.normal2point(force_point)
            angle = node.force_manager.angle_btwn_vectors(force_vector, normal_vector)

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

            timestep+=1
            dt = node.m.opt.timestep
            viewer.user_scn.ngeom = 0

            # set robot to home position
            node.d.ctrl[:7] = q_home

            # Apply force
            node.force_manager.apply_force(force_point, force_vector, timestep, viewer)
            
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

            msg1= Float64MultiArray()
            msg1.data = np.asarray(node.d.qfrc_applied[:7], dtype=np.float64).flatten().tolist()
            node.publisher4.publish(msg1)

            # Update viewer
            viewer.sync()
            time.sleep(dt)
            
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()