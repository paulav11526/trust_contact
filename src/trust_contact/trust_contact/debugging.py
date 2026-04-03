import mujoco
import pinocchio as pin
import numpy as np
from numpy.linalg import pinv

# 1. Load Models
m_mj = mujoco.MjModel.from_xml_path("/home/paulav/contact_ws/src/trust_contact/trust_contact/models/scene.xml")
d_mj = mujoco.MjData(m_mj)

m_pin = pin.buildModelFromUrdf("/home/paulav/contact_ws/src/trust_contact/trust_contact/models/panda.urdf")
d_pin = m_pin.createData()

# 2. Set identical configuration
# Using a non-zero pose is better for catching axis flips
q_test = np.array([8.94154e-21, -0.566287, 0.00014964, -0.850776, -9.77076e-05, 1.79119, -1.53133e-05])

# -----------------------------
# MUJOCO CALCULATION
# -----------------------------
d_mj.qpos[:7] = q_test
d_mj.qvel[:7] = 0.0
if hasattr(d_mj, "qacc"):
    d_mj.qacc[:7] = 0.0

mujoco.mj_forward(m_mj, d_mj)

# Jacobian for the 'peg' body
jacp = np.zeros((3, m_mj.nv))
jacr = np.zeros((3, m_mj.nv))
body_id = mujoco.mj_name2id(m_mj, mujoco.mjtObj.mjOBJ_BODY, 'peg')
mujoco.mj_jacBody(m_mj, d_mj, jacp, jacr, body_id)
mj_jacobian = np.vstack([jacp, jacr])

# MuJoCo gravity/bias term
# qfrc_bias contains gravity + velocity-dependent effects.
# Since qvel = 0 here, this should reduce essentially to gravity.
mj_bias = d_mj.qfrc_bias[:7].copy()

# MuJoCo actuator generalized torque
mj_tau_actuator = d_mj.qfrc_actuator[:7].copy()



# -----------------------------
# PINOCCHIO CALCULATION
# -----------------------------
pin.forwardKinematics(m_pin, d_pin, q_test)
pin.updateFramePlacements(m_pin, d_pin)
pin.computeJointJacobians(m_pin, d_pin, q_test)

frame_id = m_pin.getFrameId("peg")
pin_jacobian = pin.getFrameJacobian(
    m_pin, d_pin, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
)

# Pinocchio gravity
pin_g = pin.computeGeneralizedGravity(m_pin, d_pin, q_test).copy()

# Pinocchio nonlinear effects at zero velocity
qdot_zero = np.zeros(7)
pin.computeAllTerms(m_pin, d_pin, q_test, qdot_zero)
pin_nle = d_pin.nle.copy()

# -----------------------------
# RESULTS
# -----------------------------
print("\n--- POSITION CHECK ---")
print(f"MuJoCo Peg Pos:    {d_mj.body('peg').xpos}")
print(f"Pinocchio Peg Pos: {d_pin.oMf[frame_id].translation}")

print("\n--- JACOBIAN CHECK (First 3 rows - Translation) ---")
print("MuJoCo:\n", mj_jacobian[:3, :7])
print("Pinocchio:\n", pin_jacobian[:3, :7])

print("\n--- JACOBIAN ERROR MAGNITUDE ---")
print(np.linalg.norm(mj_jacobian[:3, :7] - pin_jacobian[:3, :7]))

print("\n--- GRAVITY / TORQUE CHECK ---")
print("MuJoCo qfrc_bias (gravity at qvel=0):")
print(mj_bias)

print("\nPinocchio g(q):")
print(pin_g)

print("\nPinocchio nle(q, qdot=0):")
print(pin_nle)

print("\nMuJoCo actuator torque qfrc_actuator:")
print(mj_tau_actuator)

print("\nDifference: Pinocchio g - MuJoCo bias")
print(pin_g - mj_bias)

print("\nDifference: actuator torque - MuJoCo bias")
print(mj_tau_actuator - mj_bias)

print("\nDifference: actuator torque - Pinocchio g")
print(mj_tau_actuator - pin_g)

print("\n--- MuJoCo Body Masses ---")
for i in range(m_mj.nbody):
    name = mujoco.mj_id2name(m_mj, mujoco.mjtObj.mjOBJ_BODY, i)
    print(name, m_mj.body_mass[i])
    

print("\n--- Pinocchio Body Masses ---")
for i, inertia in enumerate(m_pin.inertias):
    print(i, inertia.mass)

print("Total MJ mass:", np.sum(m_mj.body_mass))
print("Total Pin mass:", sum([i.mass for i in m_pin.inertias]))

# -----------------------------
# INERTIA MATRIX COMPARISON
# -----------------------------

# --- MuJoCo inertia matrix ---
M_mj = np.zeros((m_mj.nv, m_mj.nv))
mujoco.mj_fullM(m_mj, M_mj, d_mj.qM)   # expands internal representation
M_mj = M_mj[:7, :7]  # only the 7 arm joints

# --- Pinocchio inertia matrix ---
pin.computeAllTerms(m_pin, d_pin, q_test, np.zeros(7))
M_pin = d_pin.M.copy()
M_pin = M_pin.copy() + 0.1*np.eye(len(M_pin))

# --- Print ---
print("\n--- INERTIA MATRIX CHECK ---")
print("MuJoCo M:\n", M_mj)
print("\nPinocchio M:\n", M_pin)

# --- Error ---
diff = M_pin - M_mj
print("\nDifference (Pin - MJ):\n", diff)

print("\nFrobenius norm of difference:",
      np.linalg.norm(diff))
d_mj.qpos[:7] = q_test
d_mj.qvel[:7] = 0.0
d_mj.ctrl[:7] = q_test
mujoco.mj_forward(m_mj, d_mj)

print("qfrc_actuator:", d_mj.qfrc_actuator[:7])
print("qfrc_passive: ", d_mj.qfrc_passive[:7])
print("qfrc_bias:    ", d_mj.qfrc_bias[:7])
print(f'actuator - bias: {d_mj.qfrc_actuator[:7] - d_mj.qfrc_bias[:7]}')
print("qvel:", d_mj.qvel[:7])
print("qfrc_constraint:", d_mj.qfrc_constraint[:7])

print("ncon:", d_mj.ncon)
for i in range(d_mj.ncon):
    con = d_mj.contact[i]
    geom1 = mujoco.mj_id2name(m_mj, mujoco.mjtObj.mjOBJ_GEOM, con.geom1)
    geom2 = mujoco.mj_id2name(m_mj, mujoco.mjtObj.mjOBJ_GEOM, con.geom2)
    print(i, geom1, geom2, "dist =", con.dist)


print("q4:", d_mj.qpos[3])
print("q4 range:", m_mj.jnt_range[3])
print("ctrl4:", d_mj.ctrl[3])
print("qfrc_constraint:", d_mj.qfrc_constraint[:7])
print("qfrc_actuator:", d_mj.qfrc_actuator[:7])

print("neq:", m_mj.neq)
for i in range(m_mj.neq):
    print(i, m_mj.eq_type[i], m_mj.eq_obj1id[i], m_mj.eq_obj2id[i])

### testing point
point_peg = [-0.021978, -0.016132, 0.054675]
force_world = [0.09497044, -10.33624113 ,  2.04209867]
force_magnitude = 10.536463692853902

ee_frame_name = "peg"  
peg_id = m_pin.getFrameId(ee_frame_name)
parent_joint_id = m_pin.frames[peg_id].parentJoint
contact_frame = pin.Frame("contact_frame", parent_joint_id, peg_id, pin.SE3(np.eye(3), point_peg), pin.FrameType.OP_FRAME)
frame_id = m_pin.addFrame(contact_frame)
q = [8.94154e-21, -0.566287, 0.00014964, -0.850776, -9.77076e-05, 1.79119, -1.53133e-05]
# update contact point frame if point changes
m_pin.frames[frame_id].placement = pin.SE3(np.eye(3), point_peg)

pin.forwardKinematics(m_pin, d_pin.data, q)
pin.updateFramePlacements(m_pin, d_pin)

# Jacobian at peg
Jc = pin.computeFrameJacobian(m_pin, d_pin, q, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
Jc_v=Jc[:3, :]

F_ext = pinv(Jc_v.T) @ d_mj.qfrc_applied

print(f'Jc: {Jc}')
print(f'Fext = {F_ext}')