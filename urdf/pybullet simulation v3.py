import pybullet as p
import time
import pybullet_data
from math import *

"""
Created by Aditya Srinivas Manohar ; May 4th 2021
Nomenclature-
Z axis is perpendicular to the plane of movement
Y axis is along the length of the robot
Z axis is along the width of the robot

For each leg X,Y and Z has a different coordinate system
"""

## The below code basically controls only for a single leg

#Variables that control movement
Z = 400
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
 
# Leg joint indices for different legs
# legname = [hip joint, upper limb joint, lower limb joint]
fl =  [0,1,2]
fr = [3,4,5]
bl = [6,7,8]
br = [9,10,11]

##Rotation stuff
## coordinate = [left front leg, right front leg, left back leg, right back leg]
Xc = [0,0,0,0]
Yc = [0,0,0,0]
Zc = [300,300,300,300]

#yaw manipulation
##Rotation stuff
## coordinate = [left front leg, right front leg, left back leg, right back leg]
Xc = [0,0,0,0]
Yc = [0,0,0,0]
Zc = [300,300,300,300]
BodyLength = 593.3
BodyWidth = 250
def Rotation(Xc,Yc,Zc,yawangle):
    YawAngle = yawangle
    YawAngle = YawAngle*(pi/180) 
    Yuc = [Yc[0] + (BodyLength/2), Yc[1] + (BodyLength/2), Yc[2] - (BodyLength/2), Yc[3] - (BodyLength/2)]
    Xuc = [Xc[0] - (BodyWidth/2), Xc[1] + (BodyWidth/2), Xc[2] + (BodyWidth/2), Xc[3] - (BodyWidth/2)]
    ExistingAngle = []
    DemandYaw = []
    Radius = []
    for i in range(4):
        ExistingAngle += [atan(Xuc[i]/Yuc[i])]
    for i in range(4):
        DemandYaw += [ExistingAngle[i] + YawAngle]
        Radius += [Xuc[i]/sin(ExistingAngle[i])]
    Xnew = []
    Ynew = []
    for i in range(4):
        Xnew += [Radius[i]*sin(DemandYaw[i])]
        Ynew += [Radius[i]*cos(DemandYaw[i])]
    Ynewu = [Ynew[0] - (BodyLength/2), Ynew[1] - (BodyLength/2), Ynew[2] + (BodyLength/2), Ynew[3] + (BodyLength/2)]
    Xnewu = [Xnew[0] + (BodyWidth/2), Xnew[1] - (BodyWidth/2), Xnew[2] - (BodyWidth/2), Xnew[3] + (BodyWidth/2)] 
    return Ynewu, Xnewu

# Yc, Xc = Rotation(Xc,Yc,Zc,30)
# a1,b1,c1 = legik(Xc[0],Yc[0],Zc[0])
# a2,b2,c2 = legik(Xc[1],Yc[1],Zc[1])
# a3,b3,c3 = legik(Xc[2],Yc[2],Zc[2])
# a4,b4,c4 = legik(Xc[3],Yc[3],Zc[3])

a,b,c = legik(X,Y,Z)
print(a*180/pi,b*180/pi,c*180/pi)
# pybullet initialization
# in the pybullet environment all units are in metres
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.setGravity(0,0,-10)
p.setPhysicsEngineParameter(enableFileCaching=0)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,.55]
startOrientation = p.getQuaternionFromEuler([0,0,0])
quad = p.loadURDF("quadf.urdf",startPos, startOrientation,useFixedBase=False)
no = p.getNumJoints(quad)

for k in range(no):
    info = p.getJointInfo(quad,k)
    print(info[0],": ",info[1])

##initialization of various joint parameters
# positions = [0,pi/6,-pi/3,0,-pi/6,pi/3,0,pi/6,-pi/3,0,-pi/6,pi/3]
positions = [a,b,c,a,-b,-c,a,b,c,a,-b,-c]
# positions = [a1,b1,c1,a2,-b2,-c2,a3,b3,c3,a4,-b4,-c4]
# force = [6,6,6,6,6,6,6,6,6,6,6,6]
force = [9,9,9,9,9,9,9,9,9,9,9,9]
velocity = [1,1,1,1,1,1,1,1,1,1,1,1]
indices = [0,1,2,3,4,5,6,7,8,9,10,11]

p.setRealTimeSimulation(1)
p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
# p.setTimeStep(0.002)
time.sleep(3)


counter = 0
X = 0
Y = 0
Z = 350
X1 = 0
Y1 = 0

while True:
    counter +=1
    print("counter = ",counter)
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
    # for i in range(0,200):
        # Y = i
        # a,b,c = legik(X,Y,Z)
        # positions = [-a,b,c,a,-b,-c,-a,b,c,a,-b,-c]
        # p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        # time.sleep(.02)
    
    Z1 = 300
    Xi = X
    #forward moving
    d,e,f = legik(X1,Y1,Z1)
    p.setJointMotorControlArray(quad,jointIndices=[0,1,2,9,10,11],controlMode=p.POSITION_CONTROL,targetPositions= [-d,e,f,d,-e,-f],forces=[9,9,9,9,9,9],targetVelocities=[1,1,1,1,1,1])
    for i in range(X,101):
        X = i
        X1 = -i
        Z1 = Z1 + (50/(101-Xi))
        a,b,c = legik(X,Y,Z)
        d,e,f = legik(X1,Y1,Z1)
        positions = [-d,e,f,a,-b,-c,-a,b,c,d,-e,-f]
        p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        time.sleep(.001)
    Z = 300
    Xi = X
    a,b,c = legik(X,Y,Z)
    p.setJointMotorControlArray(quad,jointIndices=[3,4,5,6,7,8],controlMode=p.POSITION_CONTROL,targetPositions= [a,-b,-c,-a,b,c],forces=[9,9,9,9,9,9],targetVelocities=[1,1,1,1,1,1])
    for i in range(X,-101,-1):
        X = i
        X1 = -i
        Z = Z + (50/(101+Xi))
        a,b,c = legik(X,Y,Z)
        d,e,f = legik(X1,Y1,Z1)
        positions = [-d,e,f,a,-b,-c,-a,b,c,d,-e,-f]
        p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        time.sleep(.001)
    print(X,Y,Z)
    print(X1,Y1,Z1) 
    # time.sleep(1)
    #rotating
    # for i in range(30):
        # print(i)
        # print(Xc,Yc)
        # Yc, Xc = Rotation(Xc,Yc,Zc,i)
        # a1,b1,c1 = legik(Xc[0],Yc[0],Zc[0])
        # a2,b2,c2 = legik(Xc[1],Yc[1],Zc[1])
        # a3,b3,c3 = legik(Xc[2],Yc[2],Zc[2])
        # a4,b4,c4 = legik(Xc[3],Yc[3],Zc[3])
        # positions = [a1,b1,c1,a2,-b2,-c2,a3,b3,c3,a4,-b4,-c4]
        # p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        # time.sleep(1)
        
    # positions = [0,0,0,0,0,0,0,0,0,0,0,0]
    # print("second move",positions)
    # p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
    # positions = [0,pi/4,-pi/2,0,-pi/4,pi/2,0,pi/4,-pi/2,0,-pi/4,pi/2]
    # time.sleep(1)

cubePos, cubeOrn = p.getBasePositionAndOrientation(quad)
print(cubePos,cubeOrn)
p.disconnect()
