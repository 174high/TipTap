import pybullet as p
import pybullet_data
import os
import time 


# physics parameter setting
fixedTimeStep = 1./1000
numSolverIterations = 200


physicsClient = p.connect(p.GUI)
p.setTimeStep(fixedTimeStep)
p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load plane.urdf


p.setGravity(0, 0, -9.81)
p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=10, cameraPitch=-5, cameraTargetPosition=[0.3, 0.5, 0.1])

StartPos = [0, 0, 0.3]
StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
# samurai.urdf  plane.urdf
planeId = p.loadURDF("plane.urdf")


robot = p.loadURDF("test.urdf", StartPos, StartOrientation)
p.setRealTimeSimulation(1) # real-time instead of step control


while 1:

#    p.stepSimulation()
#    time.sleep(0.01)

    pass 