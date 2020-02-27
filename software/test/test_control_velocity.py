import pybullet as p
import time
import numpy as np

## Setup the gui and connect to physic's engine
p.connect(p.GUI)
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)
################################################

## Create the objects (inverted rotary pendulum)
motor=p.createCollisionShape(p.GEOM_CYLINDER,radius=0.15,height=.4)
link=p.createCollisionShape(p.GEOM_CYLINDER,radius=0.05,height=.5)
pendulum= p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.005, 0.02, 0.2])
################################################

## Setup the properties of the chapes including how they are connected/friction/mass/orientation/position/etc
mass = 1
visualShapeId = -1

link_Masses = [.5,.1]
linkCollisionShapeIndices = [link,pendulum]
linkVisualShapeIndices = [-1,-1]

linkPositions = [[0.0, 0., .25],[0.00, 0.0, 0.25]]
linkOrientations = [[0, 1, 0, 1],[0, 1, 0, 1]]
linkInertialFramePositions = [[0, 0, 0],[0, 0, 0]]
linkInertialFrameOrientations = [[0, 0, 0, 1],[0, 0, 0, 1]]
indices = [0,1]
jointTypes = [p.JOINT_REVOLUTE,p.JOINT_REVOLUTE]
axis = [[1, 0, 0],[1,0,0]]
basePosition = [-.067,.0893,0.19999]
baseOrientation = [0,0,1,.1455]
################################################

## Create the multibody entity
sphereUid = p.createMultiBody(mass,
                                      motor,
                                      visualShapeId,
                                      basePosition,
                                      baseOrientation,
                                      linkMasses=link_Masses,
                                      linkCollisionShapeIndices=linkCollisionShapeIndices,
                                      linkVisualShapeIndices=linkVisualShapeIndices,
                                      linkPositions=linkPositions,
                                      linkOrientations=linkOrientations,
                                      linkInertialFramePositions=linkInertialFramePositions,
                                      linkInertialFrameOrientations=linkInertialFrameOrientations,
                                      linkParentIndices=indices,
                                      linkJointTypes=jointTypes,
                                      linkJointAxis=axis)

#######################


p.setGravity(0, 0, -9.8) # set gravity
p.setJointMotorControl2(sphereUid, 1, p.VELOCITY_CONTROL, targetVelocity=5, force=0) # want the pendulum to not be motor control but move freely
p.setRealTimeSimulation(1) # real-time instead of step control

## to make sure the simulation always starts in the same camera angle/perpective etc  
p.resetDebugVisualizerCamera( cameraDistance=1.5, cameraYaw=-30, cameraPitch=-30, cameraTargetPosition=[0.0, 0.0, 0.25])

## this is the main loop for control/etc
while (1):
  rnd=(.5-np.random.rand(1))  
  p.setJointMotorControl2(sphereUid, 1, p.VELOCITY_CONTROL, targetVelocity=10, force=rnd) # just apply a random motor command
  p.setJointMotorControl2(sphereUid, 0, p.VELOCITY_CONTROL, targetVelocity=10, force=rnd) # just apply a random motor command

time.sleep(0.01)