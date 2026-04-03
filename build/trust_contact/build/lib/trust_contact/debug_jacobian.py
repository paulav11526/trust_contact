import mujoco
import pinocchio as pin
import numpy as np

# -----------------------------
# USER-SET VALUES
# -----------------------------
q_test = np.array([0.1, -0.7, 0.2, -1.1, -0.1, 1.5, 0.7])   # use your actual pose
point_local = np.array([0.00158, 0.015746, 0.048112])       # point on peg, in peg frame
force_world = np.array([-0.01825065, 14.29795358, -0.01722611])  # actual world force used by MuJoCo

# -----------------------------
# LOAD MODELS
# -----------------------------
m_mj = mujoco.MjModel.from_xml_path("/home/paulav/contact_ws/src/trust_contact/trust_contact/models/scene.xml")
d_mj = mujoco.MjData(m_mj)

m_pin = pin.buildModelFromUrdf("/home/paulav/contact_ws/src/trust_contact/trust_contact/models/panda.urdf")
d_pin = m_pin.createData()

# -----------------------------
# MUJOCO STATE SETUP
# -----------------------------
d_mj.qpos[:7] = q_test
d_mj.qvel[:7] = 0.0
d_mj.ctrl[:7] = q_test
d_mj.qfrc_applied[:] = 0.0
mujoco.mj_forward(m_mj, d_mj)

# body info
body_id = mujoco.mj_name2id(m_mj, mujoco.mjtObj.mjOBJ_BODY, "peg")
xpos = d_mj.xpos[body_id].copy()
xmat = d_mj.xmat[body_id].reshape(3, 3).copy()

# convert local point -> world point
point_world = xmat @ point_local + xpos

# apply force
mujoco.mj_applyFT(
    m_mj,
    d_mj,
    force_world,
    np.zeros(3),
    point_world,
    body_id,
    d_mj.qfrc_applied
)

# after applying force, you may want forward/inverse recomputation depending on your workflow
mujoco.mj_forward(m_mj, d_mj)

qfrc_applied = d_mj.qfrc_applied[:7].copy()

# -----------------------------
# MUJOCO POINT JACOBIAN
# -----------------------------
jacp_mj = np.zeros((3, m_mj.nv))
jacr_mj = np.zeros((3, m_mj.nv))

# Jacobian of arbitrary world point attached to body
mujoco.mj_jac(m_mj, d_mj, jacp_mj, jacr_mj, point_world, body_id)
J_mj = jacp_mj[:, :7].copy()

tau_from_mj = J_mj.T @ force_world

# -----------------------------
# PINOCCHIO POINT JACOBIAN
# -----------------------------
pin.forwardKinematics(m_pin, d_pin, q_test)
pin.updateFramePlacements(m_pin, d_pin)
pin.computeJointJacobians(m_pin, d_pin, q_test)

peg_frame_id = m_pin.getFrameId("peg")
parent_joint_id = m_pin.frames[peg_frame_id].parentJoint

# add a temporary contact frame at the same local point on peg
contact_frame = pin.Frame(
    "contact_frame_debug",
    parent_joint_id,
    peg_frame_id,
    pin.SE3(np.eye(3), point_local),
    pin.FrameType.OP_FRAME
)
contact_frame_id = m_pin.addFrame(contact_frame)

pin.updateFramePlacements(m_pin, d_pin)

J_pin_6 = pin.getFrameJacobian(
    m_pin,
    d_pin,
    contact_frame_id,
    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
)
J_pin = J_pin_6[:3, :].copy()

tau_from_pin = J_pin.T @ force_world

# -----------------------------
# PRINT COMPARISONS
# -----------------------------
np.set_printoptions(precision=8, suppress=True)

print("\n--- INPUTS ---")
print("q_test:", q_test)
print("point_local:", point_local)
print("point_world:", point_world)
print("force_world:", force_world)
print("||force_world||:", np.linalg.norm(force_world))

print("\n--- MUJOCO RESULTS ---")
print("qfrc_applied:", qfrc_applied)
print("J_mj:\n", J_mj)
print("tau_from_mj = J_mj.T @ force_world:", tau_from_mj)
print("error_mj:", qfrc_applied - tau_from_mj)
print("||error_mj||:", np.linalg.norm(qfrc_applied - tau_from_mj))

print("\n--- PINOCCHIO RESULTS ---")
print("J_pin:\n", J_pin)
print("tau_from_pin = J_pin.T @ force_world:", tau_from_pin)
print("error_pin:", qfrc_applied - tau_from_pin)
print("||error_pin||:", np.linalg.norm(qfrc_applied - tau_from_pin))

print("\n--- JACOBIAN DIFFERENCE ---")
print("J_pin - J_mj:\n", J_pin - J_mj)
print("||J_pin - J_mj||:", np.linalg.norm(J_pin - J_mj))

# Inverse test
f_est_from_mj_tau = np.linalg.pinv(J_mj.T) @ qfrc_applied
f_est_from_pin_tau = np.linalg.pinv(J_pin.T) @ qfrc_applied

print("\n--- INVERSE TEST ---")
print("f_est_from_mj_tau:", f_est_from_mj_tau)
print("f_est_from_pin_tau:", f_est_from_pin_tau)
print("actual force_world:", force_world)