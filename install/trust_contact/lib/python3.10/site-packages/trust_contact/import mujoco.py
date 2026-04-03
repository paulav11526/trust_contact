import mujoco
import pinocchio as pin
import numpy as np

# 1. Load Models
m_mj = mujoco.MjModel.from_xml_path("/home/paulav/contact_ws/src/trust_contact/trust_contact/models/scene.xml")
d_mj = mujoco.MjData(m_mj)

m_pin = pin.buildModelFromUrdf("/home/paulav/contact_ws/src/trust_contact/trust_contact/models/panda.urdf")
d_pin = m_pin.createData()

# 2. Set identical configuration
# Using a non-zero pose is better for catching axis flips
q_test = np.array([0.1, -0.7, 0.2, -2.3, -0.1, 1.5, 0.7]) 

# --- MUJOCO CALCULATION ---
d_mj.qpos[:7] = q_test
mujoco.mj_forward(m_mj, d_mj)

# Get Jacobian for the 'peg' body
# jacp = 3xNV (translation), jacr = 3xNV (rotation)
jacp = np.zeros((3, m_mj.nv))
jacr = np.zeros((3, m_mj.nv))
# Assuming 'peg' is the name in your XML
body_id = mujoco.mj_name2id(m_mj, mujoco.mjtObj.mjOBJ_BODY, 'peg')
mujoco.mj_jacBody(m_mj, d_mj, jacp, jacr, body_id)
mj_jacobian = np.vstack([jacp, jacr])

# --- PINOCCHIO CALCULATION ---
# 1. Get the placement of the robot base in MuJoCo
# (If your robot is at 0,0,0 in MuJoCo, this is just Identity)
# But let's assume there's an offset:
base_pos = d_mj.body('link0').xpos 
base_quat = d_mj.body('link0').xquat # MuJoCo is [w, x, y, z]

# 2. In Pinocchio, ensure you are using the WORLD frame 
# and that the forward kinematics are updated:
pin.forwardKinematics(m_pin, d_pin, q_test)
pin.updateFramePlacements(m_pin, d_pin)
frame_id = m_pin.getFrameId("peg")

# Use ReferenceFrame.WORLD explicitly
pin.computeJointJacobians(m_pin, d_pin, q_test)
pin_jacobian = pin.getFrameJacobian(m_pin, d_pin, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

# --- RESULTS ---
print("--- POSITION CHECK ---")
print(f"MuJoCo Peg Pos:    {d_mj.body('peg').xpos}")
# Note: Pinocchio uses oMf[id].translation for world position
pin.updateFramePlacements(m_pin, d_pin)
print(f"Pinocchio Peg Pos: {d_pin.oMf[frame_id].translation}")

print("\n--- JACOBIAN CHECK (First 3 rows - Translation) ---")
print("MuJoCo:\n", mj_jacobian[:3, :7])
print("Pinocchio:\n", pin_jacobian[:3, :7])

print("\n--- ERROR MAGNITUDE ---")
print(np.linalg.norm(mj_jacobian[:3, :7] - pin_jacobian[:3, :7]))