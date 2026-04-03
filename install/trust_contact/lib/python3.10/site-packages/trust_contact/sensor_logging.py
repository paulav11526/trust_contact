import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import time
import pandas as pd

"""
for context this is how sensor readings are stored in the list:

f1x_reading = data.sensordata[0]        f2x_reading = data.sensordata[6]        f3x_reading = data.sensordata[12]       f4x_reading = data.sensordata[18]
f1y_reading = data.sensordata[1]        f2y_reading = data.sensordata[7]        f3y_reading = data.sensordata[13]       f4y_reading = data.sensordata[19]
f1z_reading = data.sensordata[2]        f2z_reading = data.sensordata[8]        f3z_reading = data.sensordata[14]       f4z_reading = data.sensordata[20]
t1x_reading = data.sensordata[3]        t2x_reading = data.sensordata[9]        t3x_reading = data.sensordata[15]       t4x_reading = data.sensordata[21]
t1y_reading = data.sensordata[4]        t2y_reading = data.sensordata[10]       t3y_reading = data.sensordata[16]       t4y_reading = data.sensordata[22]
t1z_reading = data.sensordata[5]        t2z_reading = data.sensordata[11]       t3z_reading = data.sensordata[17]       t4z_reading = data.sensordata[23]
        
f5x_reading = data.sensordata[24]       f6x_reading = data.sensordata[30]       f7x_reading = data.sensordata[36]       f8x_reading = data.sensordata[42]
f5y_reading = data.sensordata[25]       f6y_reading = data.sensordata[31]       f7y_reading = data.sensordata[37]       f8y_reading = data.sensordata[43]
f5z_reading = data.sensordata[26]       f6z_reading = data.sensordata[32]       f7z_reading = data.sensordata[38]       f8z_reading = data.sensordata[44]
t5x_reading = data.sensordata[27]       t6x_reading = data.sensordata[33]       t7x_reading = data.sensordata[39]       t8x_reading = data.sensordata[45]
t5y_reading = data.sensordata[28]       t6y_reading = data.sensordata[34]       t7y_reading = data.sensordata[40]       t8y_reading = data.sensordata[46]
t5z_reading = data.sensordata[29]       t6z_reading = data.sensordata[35]       t7z_reading = data.sensordata[41]       t8z_reading = data.sensordata[47]

"""

# load panda model and data
model = mujoco.MjModel.from_xml_path("models/scene.xml")
data = mujoco.MjData(model)


# Launch the simulation
with mujoco.viewer.launch_passive(model, data) as viewer:

    timestep = 0
    
    # creating data frames to store sensor readings for each joint
    sensor1 = pd.DataFrame(columns=['fx', 'fy', 'fz', 'tz'])
    sensor2 = pd.DataFrame(columns=['fx', 'fy', 'fz', 'tz'])
    sensor3 = pd.DataFrame(columns=['fx', 'fy', 'fz', 'tz'])
    sensor4 = pd.DataFrame(columns=['fx', 'fy', 'fz', 'tz'])
    sensor5 = pd.DataFrame(columns=['fx', 'fy', 'fz', 'tz'])
    sensor6 = pd.DataFrame(columns=['fx', 'fy', 'fz', 'tz'])
    sensor7 = pd.DataFrame(columns=['fx', 'fy', 'fz', 'tz'])
    sensor8 = pd.DataFrame(columns=['fx', 'fy', 'fz', 'tz'])

    while viewer.is_running():
        
        dt = model.opt.timestep
        
        if  0 < timestep <= 2000: # recording sensor data at certain timesteps
            sensor1.loc[timestep] = [data.sensordata[0], data.sensordata[1], data.sensordata[2], data.sensordata[5]]
            sensor2.loc[timestep] = [data.sensordata[6], data.sensordata[7], data.sensordata[8], data.sensordata[11]]
            sensor3.loc[timestep] = [data.sensordata[12], data.sensordata[13], data.sensordata[14], data.sensordata[17]]
            sensor4.loc[timestep] = [data.sensordata[18], data.sensordata[19], data.sensordata[20], data.sensordata[23]]
            sensor5.loc[timestep] = [data.sensordata[24], data.sensordata[25], data.sensordata[26], data.sensordata[29]]
            sensor6.loc[timestep] = [data.sensordata[30], data.sensordata[31], data.sensordata[32], data.sensordata[35]]
            sensor7.loc[timestep] = [data.sensordata[36], data.sensordata[37], data.sensordata[38], data.sensordata[41]]
            sensor8.loc[timestep] = [data.sensordata[42], data.sensordata[43], data.sensordata[44], data.sensordata[47]]
                
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Update viewer
        viewer.sync()
        time.sleep(dt)
        timestep += 1