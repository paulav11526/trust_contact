import mujoco
import mujoco.viewer
import numpy as np
import time

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool

class ApplyContactForce(Node):
    def __init__(self):
        super().__init__('contact_force_node')
        self.publisher_ = self.create_publisher(Bool, 'contact_detection', 10)

        # Load model
        # load robot model and data
        import os

        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, "models", "scene.xml")
        self.m = mujoco.MjModel.from_xml_path(model_path)
        self.d = mujoco.MjData(self.m)

        self.body = self.d.body("peg").id
        for i in range(self.m.njnt):
            name = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_JOINT, i)
            print(i, name)


    def apply_force(self, force_point, force_vector, timestep, viewer, wait):
        
        # scaling force vector for visualization so arrow geom is not giant
        scaled_force_vector = [force_vector[0]/100, force_vector[1]/100, force_vector[2]/100]
        # finding starting point for force vector's arrow 
        local_arrow_start = [force_point[0]-scaled_force_vector[0], force_point[1]-scaled_force_vector[1], force_point[2]-scaled_force_vector[2]]
        
        # get global position and rotation of desired body
        xpos = self.d.xpos[self.body]
        xmat = self.d.xmat[self.body].reshape(3, 3)

        # convert point of force application to global coordinates
        globalpoint = xmat @ force_point + xpos
        arrow_start = xmat @ local_arrow_start + xpos

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

        in_contact = (wait < timestep < wait + 10)

        if in_contact:
            mujoco.mj_applyFT(self.m, self.d, force_vector, [0,0,0], globalpoint, self.body, self.d.qfrc_applied)

        msg = Bool()
        msg.data = in_contact
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {in_contact}')



def main(args=None):
    rclpy.init(args=args)
    node = ApplyContactForce()
    force= [-2.3310970026575504, -24.92942268797891, 27.449059574015948]
    point= [np.float64(0.025166), np.float64(-0.000428), np.float64(0.094485)]

    # Launch the simulation
    with mujoco.viewer.launch_passive(node.m, node.d) as viewer:
        start_time = time.time()
        timestep = 0

        while viewer.is_running():

            timestep+=1
            dt = node.m.opt.timestep
            viewer.user_scn.ngeom = 0
            
            node.apply_force(point, force, timestep, viewer, wait=2000)
                    
            # Step simulation
            mujoco.mj_step(node.m, node.d)
            
            # Update viewer
            viewer.sync()
            time.sleep(dt)
            rclpy.spin_once(node, timeout_sec=0.001)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()