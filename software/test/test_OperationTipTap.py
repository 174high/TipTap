import pybullet as p
import pybullet_data
import time
import atexit
import os
import importlib
import numpy as np
import enum


class joint(enum.Enum):
    RightServo = 0
    RightHip = 1
    RightKnee = 2
    LeftServo = 4
    LeftHip = 5
    LeftKnee = 6


enum2urdf_id = {
    joint.RightServo: 0,
    joint.RightHip: 1,
    joint.RightKnee: 2,
    joint.LeftServo: 4,
    joint.LeftHip: 5,
    joint.LeftKnee: 6
}

enum2pyds_id = {
    joint.RightServo: None,
    joint.RightHip: 0,
    joint.RightKnee: 1,
    joint.LeftServo: None,
    joint.LeftHip: 2,
    joint.LeftKnee: 3
}

PositionControlable = [joint.RightServo, joint.LeftServo]
TorqueControlable = \
    [joint.RightHip, joint.RightKnee, joint.LeftHip, joint.LeftKnee]
last_sevo_angles = [0, 0]


#
# Initialize robot and sim
#
GRAVITY = -9.81
atexit.register(p.disconnect)
try:
    p.disconnect()
except Exception:
    pass
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.resetDebugVisualizerCamera(cameraDistance=0.6,
                             cameraYaw=20,
                             cameraPitch=-40,
                             cameraTargetPosition=[0, 0, 0.1])
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
p.setRealTimeSimulation(0)
p.setPhysicsEngineParameter(numSubSteps=4)
p.setPhysicsEngineParameter(numSolverIterations=10)

p.setGravity(0, 0, GRAVITY)
StartPos = [0, 0, 0.3]
StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
planeId = p.loadURDF("plane.urdf")
botId = p.loadURDF("tiptap.urdf", StartPos, StartOrientation)

# Disable the motors for torque control:
p.setJointMotorControlArray(
    botId,
    [j for j in range(p.getNumJoints(botId))],
    p.VELOCITY_CONTROL,
    forces=[0.0 for j in range(p.getNumJoints(botId))]
)

# regenerate urdf if needed
if not os.path.isfile("tiptap.urdf"):
    print("xacro success= ",
          0 == os.system(
              'rosrun xacro xacro tiptap_xacro.xml > tiptap.urdf'
          ))

dt_actual = 1. / 240.
last_time = time.time()
start_time = time.time()
step_counter = 0
joint_pos = [0.0 for _ in range(p.getNumJoints(botId))]
last_pos = [0.0 for _ in range(p.getNumJoints(botId))]
joint_vel = [0.0 for _ in range(p.getNumJoints(botId))]
joint_accel = [0.0 for _ in range(p.getNumJoints(botId))]

def GetTimeStep(TargetDT=None, MinDT=1./240.):
    global last_time
    global dt_actual
    global step_counter
    global joint_pos
    global joint_vel
    global joint_accel
    step_counter += 1
    dt_actual = 0.0
    t0 = time.time()
    dt_actual = t0 - last_time

    # enforce a minumum dt limit and/or a user provided time limit
    if TargetDT is not None or dt_actual < MinDT:
        if TargetDT is None:
            TargetDT = MinDT
        remaining_time = TargetDT - dt_actual
        if remaining_time > 0.00001:
            time.sleep(remaining_time)
            t0 = time.time()
            dt_actual = t0 - last_time
    last_time = t0

    p.setPhysicsEngineParameter(fixedTimeStep=dt_actual)
    p.stepSimulation()
    for jnt_indx in range(p.getNumJoints(botId)):
        jointPosition, jointVelocity, _, _ = p.getJointState(
            botId, jnt_indx)
        joint_pos[jnt_indx] = jointPosition
        joint_accel[jnt_indx] = (
            jointVelocity - joint_vel[jnt_indx])/dt_actual
        joint_vel[jnt_indx] = jointVelocity
    elapsed_time = time.time() - start_time
    return dt_actual, elapsed_time

BodyCount = 9
state_snapshot = [[{}, 0.1234] for _ in range(BodyCount)]
def _anglevec(u, v): return np.arccos(
    np.array(u).dot(v)/np.linalg.norm(u)/np.linalg.norm(v))

torso_index = -1

class TipTap():

        @staticmethod
        def OnHardware():
            return False

        @staticmethod
        def GetBodyState(body_indx=-1):
            global step_counter, dt_actual, state_snapshot
            state_indx = body_indx + 1
            if step_counter != state_snapshot[state_indx][1] and dt_actual > 0:
                if body_indx == -1:


                    pos_linkcom_world, world_rot_linkcom = \
                        p.getBasePositionAndOrientation(botId)
                     
                    print("pos_linkcom_world",pos_linkcom_world,"world_rot_linkcom",world_rot_linkcom)

                    linVel_linkcom_world, rotVel_linkcom_world = \
                        p.getBaseVelocity(botId)

                    print("linVel_linkcom_world",linVel_linkcom_world,"rotVel_linkcom_world",rotVel_linkcom_world)

                else:
                    pos_linkcom_world, world_rot_linkcom, _, _, _, _, \
                        linVel_linkcom_world, rotVel_linkcom_world \
                        = p.getLinkState(botId, body_indx,
                                         computeLinkVelocity=1,
                                         computeForwardKinematics=1,
                                         physicsClientId=0)
                old_vel = state_snapshot[state_indx][0].get(
                    "pos_vel", linVel_linkcom_world)
                position_accel = [0, 0, 0]
                position_accel[0] = (
                    linVel_linkcom_world[0] - old_vel[0])/dt_actual
                position_accel[1] = (
                    linVel_linkcom_world[1] - old_vel[1])/dt_actual
                position_accel[2] = (
                    linVel_linkcom_world[2] - old_vel[2])/dt_actual
                mat = np.array(p.getMatrixFromQuaternion(
                    world_rot_linkcom)).reshape((3, 3))
                T = np.array([
                    [mat[0][0], mat[0][1], mat[0][2], pos_linkcom_world[0]],
                    [mat[1][0], mat[1][1], mat[1][2], pos_linkcom_world[1]],
                    [mat[2][0], mat[2][1], mat[2][2], pos_linkcom_world[2]],
                    [0, 0, 0, 1]
                ])
                state_snapshot[state_indx][0] = {
                    "pos": pos_linkcom_world,
                    "quat": world_rot_linkcom,
                    "mat": mat,
                    "T": T,
                    "ang": p.getEulerFromQuaternion(world_rot_linkcom),
                    "pos_vel": linVel_linkcom_world,
                    "ang_vel": rotVel_linkcom_world,
                    "accel": position_accel
                }
                state_snapshot[state_indx][1] = step_counter
            return state_snapshot[state_indx][0]

        @staticmethod
        def setTorque(joint_enum, force, debug=False):
            global joint_pos, joint_vel, joint_accel
            if not debug and joint_enum not in TorqueControlable:
                raise ValueError(
                    f"joint {joint_enum} is not torque controlable")
            joint_id = enum2urdf_id[joint_enum]
            p.setJointMotorControl2(
                botId, joint_id, p.TORQUE_CONTROL,
                force=force)
            return {"Angle": joint_pos[joint_id],
                    "Vel": joint_vel[joint_id],
                    "Accel": joint_accel[joint_id],
                    "Code": "OK",
                    "ID": enum2pyds_id[joint_enum]}

        @staticmethod
        def setPosition(joint_enum, angle, force=30, debug=False):
            global joint_vel, joint_accel
            if not debug and joint_enum not in PositionControlable:
                raise ValueError(
                    f"joint {joint_enum} is not position controlable")
            joint_id = enum2urdf_id[joint_enum]
            p.setJointMotorControl2(
                botId, joint_id, p.POSITION_CONTROL,
                targetPosition=angle, force=force)
            return {"Angle": angle,
                    "Vel": joint_vel[joint_id],
                    "Accel": joint_accel[joint_id],
                    "Code": "OK",
                    "ID": enum2pyds_id[joint_enum]}

        @staticmethod
        def getState(joint_enum):
            global joint_pos, joint_vel, joint_accel
            joint_id = enum2urdf_id[joint_enum]
            return {"Angle": joint_pos[joint_id],
                    "Vel": joint_vel[joint_id],
                    "Accel": joint_accel[joint_id],
                    "Code": "OK",
                    "ID": enum2pyds_id[joint_enum]}

        @staticmethod
        def getIMU():
            base_state = TipTap.GetBodyState(torso_index)
            mat = base_state["mat"]
            u = mat.dot([0, 0, 1])
            pitch = _anglevec(u, [u[0], u[1], 0])*np.sign(u[2])
            u = mat.dot([1, 0, 0])
            roll = _anglevec(u, [u[0], u[1], 0])*np.sign(u[2])
            u = mat.dot([0, 0, -1])
            yaw = np.arctan2(u[1], u[0])
            return {
                "quat": base_state["quat"],
                "yaw": yaw,
                "pitch": pitch,
                "roll": roll,
                "accel": base_state["accel"]}

        @staticmethod
        def setStates(qin):
            RS_state = TipTap.setPosition(joint.RightServo, qin[0])
            RH_state = TipTap.setTorque(joint.RightHip, qin[1])
            RK_state = TipTap.setTorque(joint.RightKnee, qin[2])
            LS_state = TipTap.setPosition(joint.LeftServo, qin[3])
            LH_state = TipTap.setTorque(joint.LeftHip, qin[4])
            LK_state = TipTap.setTorque(joint.LeftKnee, qin[5])
            q = [RS_state["Angle"], RH_state["Angle"], RK_state["Angle"],
                 LS_state["Angle"], LH_state["Angle"], LK_state["Angle"]]
            dq = [RS_state["Vel"], RH_state["Vel"], RK_state["Vel"],
                  LS_state["Vel"], LH_state["Vel"], LK_state["Vel"]]
            ddq = [RS_state["Accel"], RH_state["Accel"], RK_state["Accel"],
                   LS_state["Accel"], LH_state["Accel"], LK_state["Accel"]]
            return q, dq, ddq

        @staticmethod
        def getStates():
            RS_state = TipTap.getState(joint.RightServo)
            RH_state = TipTap.getState(joint.RightHip)
            RK_state = TipTap.getState(joint.RightKnee)
            LS_state = TipTap.getState(joint.LeftServo)
            LH_state = TipTap.getState(joint.LeftHip)
            LK_state = TipTap.getState(joint.LeftKnee)
            q = [RS_state["Angle"], RH_state["Angle"], RK_state["Angle"],
                 LS_state["Angle"], LH_state["Angle"], LK_state["Angle"]]
            dq = [RS_state["Vel"], RH_state["Vel"], RK_state["Vel"],
                  LS_state["Vel"], LH_state["Vel"], LK_state["Vel"]]
            ddq = [RS_state["Accel"], RH_state["Accel"], RK_state["Accel"],
                   LS_state["Accel"], LH_state["Accel"], LK_state["Accel"]]
            return q, dq, ddq

        @staticmethod
        def calibrate(FailoverTorqueRatio=0,
                      SearchingTorqueRatio=0.35,
                      FailAtLimitOffsetDeg=15*8,
                      Timeout=15):
            pass

        @staticmethod
        def setZeroAngles():
            pass

        @staticmethod
        def GetTimeStep(TargetDT=None, MinDT=1./240.):
            return GetTimeStep(TargetDT, MinDT)


ControlSignal = [0. for _ in range(6)]

# You can use the TipTap.OnHardware() query to
# add in simulation specific functions
if not TipTap.OnHardware():
    line_ids = [p.addUserDebugLine([0, 0, 0], [0, 0, 0]) for _ in range(3)]

# main sim/control loop
while True:

    # get a dictionary of the current IMU states
    imu_data = TipTap.getIMU()

    # feel free to do simulation specific things, using not TipTap.OnHardware()
    if not TipTap.OnHardware():
        bs = TipTap.GetBodyState()  # (only in sim)
        pos = np.array(bs["pos"])
        mat = bs["mat"]
        # draw some debugging lines (+x = red,+y = green,+z = blue)
        for bvi in range(3):
            base_vect = [bvi == 0, bvi == 1, bvi == 2]
            p.addUserDebugLine(pos, pos+mat.dot(base_vect)*0.3,
                               lineColorRGB=base_vect,
                               replaceItemUniqueId=line_ids[bvi])

    # TODO:
    # Put your controller here, which uses "imu_data" and
    # crafts a new "ControlSignal" list

    # apply controller torques and positions, and get the new joint states
    q, dq, ddq = TipTap.setStates(ControlSignal)

    # step the simulation if not on hw, or just gets the elapsed time info
    dt, elapsed_time = TipTap.GetTimeStep()

