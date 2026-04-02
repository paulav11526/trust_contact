import mujoco
import mujoco.viewer
import numpy as np
from numpy import random
import matplotlib.pyplot as plt
import time
import pandas as pd

# load panda arm model and data, set simulation details
model = mujoco.MjModel.from_xml_path("models/scene.xml")
data = mujoco.MjData(model)
force_body = data.body("peg").id # link where force is going to be applied
force_limit = 20 # maximum magnitude of each force component
application_time = 10 # how many timesteps force is applied for
force_starttime = 2000 # timesteps when force application begins
logging_start = 2000 # timestep when sensor data logging starts
logging_end = 3000 # timestep when sensor data logging ends

# reading coordinates of body points from file and selecting random row of data
df = pd.read_csv('pointcloud/peg_points.asc', usecols = ["X", "Y", "Z", "Nx", "Ny", "Nz"])
random_row = random.randint(len(df.index))

# setting random force vector
def force_generation(row):
    fx = random.uniform(-force_limit,force_limit)
    fy = random.uniform(-force_limit,force_limit)
    fz = random.uniform(-force_limit,force_limit)
    force_applied = [fx, fy, fz]
    force_point = [df.iloc[row,0], df.iloc[row,1], df.iloc[row,2]] #random point of force application
    return force_applied, force_point

def normal2point(point, row):
    normal_vector = [df.iloc[row,3], df.iloc[row,4], df.iloc[row,5]]  #normal vector at point of force application, read from csv file
    # ensuring normal vector will always point outwards: point's x value and normal's x value must have the same sign etc. 
    if (point[0]<=0 and normal_vector[0]>=0) or (point[0]>=0 and normal_vector[0]<=0):
        normal_vector[0] *= -1
    if (point[1]<=0 and normal_vector[1]>=0) or (point[1]>=0 and normal_vector[1]<=0):
        normal_vector[1] *= -1
    if (point[2]<=0 and normal_vector[2]>=0) or (point[2]>=0 and normal_vector[2]<=0):
        normal_vector[2] *= -1
    return normal_vector

def angle_btwn_vectors(A, B):
    dot_product = np.dot(A, B)
    magnitude_A = np.linalg.norm(A)
    magnitude_B = np.linalg.norm(B)
    angle_radians = np.arccos(dot_product / (magnitude_A * magnitude_B))
    angle_degrees = np.degrees(angle_radians)
    return angle_degrees

def apply_force(model, data, force_point, force_body, force_vector, timestep, start, force_time):
    
    # scaling force vector for visualization so arrow geom is not giant
    scaled_force_vector = [force_vector[0]/100, force_vector[1]/100, force_vector[2]/100]
    # finding starting point for force vector's arrow 
    local_arrow_start = [force_point[0]-scaled_force_vector[0], force_point[1]-scaled_force_vector[1], force_point[2]-scaled_force_vector[2]]
    
    # get global position and rotation of desired body
    xpos = data.xpos[force_body]
    xmat = data.xmat[force_body].reshape(3, 3)

    # convert point of force application from link frame coordinates to global coordinates
    globalpoint = xmat @ force_point + xpos
    arrow_start = xmat @ local_arrow_start + xpos

    i = viewer.user_scn.ngeom
    # creating arrow to visualize direction of force applied
    mujoco.mjv_initGeom(viewer.user_scn.geoms[i], type=mujoco.mjtGeom.mjGEOM_ARROW, size=[0.01, 0.01, 0.01], pos= arrow_start, mat=np.eye(3).flatten(), rgba=[255, 0, 0, 1])
    mujoco.mjv_connector(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_ARROW, 0.008, arrow_start, globalpoint)
    i += 1
        
    # creating sphere to visualize point of force application
    mujoco.mjv_initGeom(viewer.user_scn.geoms[i], type=2, size=[0.008, 0.008, 0.008], pos= globalpoint, mat=np.eye(3).flatten(), rgba=[0, 255, 0, 1])
    i += 1
    
    viewer.user_scn.ngeom = i
    viewer.sync()

    if start <timestep< (start + force_time): # apply force at specified time interval
      mujoco.mj_applyFT(model, data, force_vector, [0,0,0], globalpoint, force_body, data.qfrc_applied)

force_vector, force_point = force_generation(random_row)
normal_vector = normal2point(force_point, random_row)
angle = angle_btwn_vectors(force_vector, normal_vector)

# ensuring random force generated is a pushing force by checking angle between force vector and normal vector to the link's surface
if angle <= 90 or angle >= 270: 
    while angle <= 90 or angle >= 270:
        force_vector, force_point = force_generation(random_row) # if force is not a pushing force, a new force is generated until requirement is fulfilled
        angle = angle_btwn_vectors(force_vector, normal_vector)

print("force", force_vector)
print("point", force_point)

# Launch the simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    timestep = 0
    
    sensor_id = force_body - 1
    sensor_readings  = pd.DataFrame(columns=['fx', 'fy', 'fz', 'tz']) # creating data frame to store sensor readings at a single joint

    # finding indexes of desired sensor readings in list of all sensor readings 'data.sensordata'
    fx = (sensor_id*6)-6
    fy = (sensor_id*6)-5
    fz = (sensor_id*6)-4
    tz = (sensor_id*6)-1 # torque sensor automatically reads torque in x,y,z but we only need tz since all joints only rotate in z direction

    while viewer.is_running():

        dt = model.opt.timestep
        viewer.user_scn.ngeom = 0
        
        apply_force(model, data, force_point, force_body, force_vector, timestep, force_starttime, application_time)

        if logging_start < timestep <= logging_end: # only recording sensor data at certain timesteps
            sensor_readings.loc[timestep] = [data.sensordata[fx], data.sensordata[fy], data.sensordata[fz], data.sensordata[tz]] # logging sensor readings in data frame

        # change to True to plot sensor readings
        if (False) and timestep == logging_end : 
           fx_list = sensor_readings['fx'].tolist()
           fy_list = sensor_readings['fy'].tolist()
           fz_list = sensor_readings['fz'].tolist()
           tz_list = sensor_readings['tz'].tolist()

           t = np.linspace(logging_start, logging_end, logging_end-logging_start)         
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
        mujoco.mj_step(model, data)
        timestep+=1

        # Update viewer
        viewer.sync()
        time.sleep(dt)

