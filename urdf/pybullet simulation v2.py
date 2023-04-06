import pybullet as p
import time
import pybullet_data
from math import *

#Variables that control movement
Z = 300
Y = 0
X = 0

#Initial Variables
LegOffset = 73.81
LowerLimb = 250
UpperLimb = 250

def legik(x,y,z):
    AbsoluteHeight = z
    SidewaysStep = y + LegOffset
    StepLength = x
    HipAngle = atan((SidewaysStep - LegOffset) / AbsoluteHeight)
    Hypotenuse = sqrt(AbsoluteHeight**2 + SidewaysStep**2)
    CurrentHeight = sqrt(Hypotenuse**2 - LegOffset**2)
    ShoulderAngle1 = atan(StepLength / CurrentHeight)
    LegLength = sqrt(CurrentHeight**2 + StepLength**2)
    KneeAngle = acos((UpperLimb**2 + LowerLimb**2 - LegLength**2)/(2*UpperLimb*LowerLimb))
    ShoulderAngle = (pi - KneeAngle)/2
    TotalShoulderAngle = ShoulderAngle + ShoulderAngle1
    # print ("Knee Angle = ",KneeAngle)
    # print ("ShoulderAngle = ", ShoulderAngle*180/pi)
    # print ("ShoulderAngle1 = ", ShoulderAngle1*180/pi)
    return HipAngle,TotalShoulderAngle,(pi - KneeAngle)*-1
 
fl =  [0,1,2]
fr = [3,4,5]
bl = [6,7,8]
br = [9,10,11]

a,b,c = legik(X,Y,Z)
print(a*180/pi,b*180/pi,c*180/pi)
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.setGravity(0,0,-10)
p.setPhysicsEngineParameter(enableFileCaching=0)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,2]
startOrientation = p.getQuaternionFromEuler([0,0,0])
quad = p.loadURDF("quadf.urdf",startPos, startOrientation,useFixedBase=False)
no = p.getNumJoints(quad)

for k in range(no):
    info = p.getJointInfo(quad,k)
    print(info[0],": ",info[1])

# positions = [0,pi/6,-pi/3,0,-pi/6,pi/3,0,pi/6,-pi/3,0,-pi/6,pi/3]
positions = [a,b,c,a,-b,-c,a,b,c,a,-b,-c]
# force = [6,6,6,6,6,6,6,6,6,6,6,6]
force = [9,9,9,9,9,9,9,9,9,9,9,9]
velocity = [1,1,1,1,1,1,1,1,1,1,1,1]
indices = [0,1,2,3,4,5,6,7,8,9,10,11]

p.setRealTimeSimulation(1)
# p.setTimeStep(0.002)
time.sleep(2)
p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)

while True:
    # print("first move",positions)
    # p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
    # time.sleep(1)
    
    # # Standing up
    # for i in range(200,500):
        # Z = i
        # a,b,c = legik(X,Y,Z)
        # positions = [a,b,c,a,-b,-c,a,b,c,a,-b,-c]
        # p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        # time.sleep(.02)
        
    # stepping sideways
    for i in range(0,200):
        Y = i
        a,b,c = legik(X,Y,Z)
        positions = [-a,b,c,a,-b,-c,-a,b,c,a,-b,-c]
        p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        time.sleep(.02)
    
    # positions = [0,0,0,0,0,0,0,0,0,0,0,0]
    # print("second move",positions)
    # p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
    # positions = [0,pi/4,-pi/2,0,-pi/4,pi/2,0,pi/4,-pi/2,0,-pi/4,pi/2]
    # time.sleep(1.5)

cubePos, cubeOrn = p.getBasePositionAndOrientation(quad)
print(cubePos,cubeOrn)
p.disconnect()
