import pybullet as p
import pybullet_data
import numpy as np
import time
import matplotlib.pyplot as plt
import pdb

# Connect to PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, True)

plane = p.loadURDF("plane.urdf")

# Load a quadruped robot (Example: Laikago)
urdfFlags = p.URDF_USE_SELF_COLLISION
quadruped = p.loadURDF("laikago/laikago_toes.urdf",[0,0,.5],[0,0.5,0.5,0], flags = urdfFlags,useFixedBase=False)
cam_info = p.getDebugVisualizerCamera()


#  0 FR_hip_motor_2_chassis_joint
#  1 FR_upper_leg_2_hip_motor_joint
#  2 FR_lower_leg_2_upper_leg_joint
#  3 jtoeFR
#  4 FL_hip_motor_2_chassis_joint
#  5 FL_upper_leg_2_hip_motor_joint
#  6 FL_lower_leg_2_upper_leg_joint
#  7 jtoeFL
#  8 RR_hip_motor_2_chassis_joint
#  9 RR_upper_leg_2_hip_motor_joint
# 10 RR_lower_leg_2_upper_leg_joint
# 11 jtoeRR
# 12 RL_hip_motor_2_chassis_joint
# 13 RL_upper_leg_2_hip_motor_joint
# 14 RL_lower_leg_2_upper_leg_joint
# 15 jtoeRL
#n_joints = p.getNumJoints(quadruped)
#for i in range(n_joints): print(p.getJointInfo(quadruped, i))

joint_IDs = [1, 2, 5, 6, 9, 10, 13, 14]
n_joints = 8

#pdb.set_trace()

# yaw, pitch, dist
n_legs = 4
cam_dist = 2.0
cam_roll = 0.0
cam_yaw = -104.0
cam_pitch = -23.0

# simulation
dt = 0.01
T = 700
timerange = np.arange(0, T, dt)

X = np.zeros((T, n_legs))
Y = np.zeros((T, n_legs))
X[0, :] = np.random.rand(n_legs) - 0.5
Y[0, :] = np.random.rand(n_legs) - 0.5
X[0,:] = np.array([0.1, 0.2, 0.3, 0.4])
Y[0,:] = np.array([0.0, 0.1, 0.2, 0.3])
joint_angles = np.zeros((T, n_joints))


# cpg
omega = np.zeros((T, n_legs))
phase_offsets = [0, np.pi/2, np.pi, 3*np.pi/2]  # Trot gait
K_nuncoupled = np.zeros((n_legs, n_legs))
K_rand = np.random.rand(n_legs, n_legs)
K_trot = np.array([[0, -1, -1, 1], [-1, 0, 1, -1], [-1, 1, 0, -1], [1, -1, -1, 0]])
K_pace = np.array([[0, -1, 1, -1], [-1, 0, -1, 1], [1, -1, 0, -1], [-1, 1, -1, 0]])
K_bound = np.array([[0, 1, -1, -1], [1, 0, -1, -1], [-1, -1, 0, 1], [-1, -1, 1, 0]])
K_walk = np.array([[0, -1, 1, -1], [-1, 0, -1, 1], [-1, 1, 0, -1], [1, -1, -1, 0]])
K = K_trot

mu_walk = 60*60
mu = mu_walk
mu = 1.5
alpha = 5.0
beta = 3.0
omega_stance = 8  # rad/s
omega_swing = 3  # rad/s
b = 100
delta = 1.0

for t in range(1, T):
    #pdb.set_trace()
    for i in np.arange(n_legs):
        omega[t,i] = omega_stance / (np.exp(-b*Y[t-1,i]) + 1) + omega_swing / (np.exp(b*Y[t-1,i]) + 1)
        r2 = np.sqrt(X[t-1,i]**2 + Y[t-1,i]**2)
        dx_i = alpha*(mu - r2)*X[t-1,i] - omega[t,i]*Y[t-1,i]
        dy_i =  beta*(mu - r2)*Y[t-1,i] + omega[t,i]*X[t-1,i] + np.dot(K[i,:], Y[t-1,:])

        X[t,i] = X[t-1,i] + dt*dx_i
        Y[t,i] = Y[t-1,i] + dt*dy_i

        joint_hip_ID, joint_knee_ID = joint_IDs[(i*2):(i*2)+2]
        if t > 500: joint_hip_angle = (X[t,i] + delta) * 0.5
        elif t > 350 and t <= 500: joint_hip_angle = (X[t,i] + delta) * 0.2
        else: joint_hip_angle = 0
        p.setJointMotorControl2(quadruped, joint_hip_ID, p.POSITION_CONTROL, joint_hip_angle)
        joint_knee_angle = 1.0
        if Y[t,i] >= 0: joint_knee_angle = 0.0
        if Y[t,i] < 0 and np.abs(X[t,i]) > 0.9: joint_knee_angle = 10*(1 - np.abs(X[t,i]))
        if t > 500: joint_knee_angle = joint_knee_angle * 0.5
        elif t > 350 and t <= 500: joint_knee_angle = joint_knee_angle * 0.2
        else: joint_knee_angle = 0
        p.setJointMotorControl2(quadruped, joint_knee_ID, p.POSITION_CONTROL, joint_knee_angle)

    p.stepSimulation()
    cam_pos, _ = (p.getBasePositionAndOrientation(quadruped))
    cam_info = p.getDebugVisualizerCamera()
    #cam_yaw, cam_pitch, cam_dist = cam_info[8], cam_info[9], cam_info[10]
    p.resetDebugVisualizerCamera(cam_dist, cam_yaw, cam_pitch, cam_pos)
    time.sleep(dt)

# Plot all oscillators over time
colors = ['#D62728', '#1F77B4', '#2CA02C', '#FF7F0E']  # Dark Red, Deep Blue, Green, Orange
linestyles = ['.', '-', '.', '-']   # for visibility when overlap
labels = [f'Leg {i+1}' for i in range(n_legs)]

plt.figure(figsize=(15, 5))

tsteps = range(0, T)
plt.subplot(121)
for i in range(n_legs):
    plt.plot(tsteps, X[:,i], color=colors[i], linestyle='-' if linestyles[i] == '-' else 'None', marker='.' if linestyles[i] == '.' else None, markersize=3, label=labels[i])
plt.xlabel('Time (s)')
plt.ylabel('x_i(t)')

plt.subplot(122)
for i in range(n_legs):
    plt.plot(tsteps, Y[:,i], color=colors[i], linestyle='-' if linestyles[i] == '-' else 'None', marker='.' if linestyles[i] == '.' else None, markersize=3)
plt.xlabel('Time (s)')
plt.ylabel('y_i(t)')


plt.figlegend(labels, loc='upper center', ncol=n_legs, fontsize=12)
plt.show()

# Plot each oscillator over space
plt.figure(2, figsize=(10, 8))

for i in range(n_legs):
    plt.subplot(2, 2, i + 1)
    plt.scatter(X[:,i], Y[:,i], color='k', s=1)
    plt.xlabel(f'x_{i+1} (s)')
    plt.ylabel(f'y_{i+1} (s)')

plt.tight_layout()
plt.show()
