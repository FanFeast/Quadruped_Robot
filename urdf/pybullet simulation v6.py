"""
Created by Aditya Srinivas Manohar ; May 4th 2021
Nomenclature-
Z axis is perpendicular to the plane of movement
Y axis is along the width of the robot
X axis is along the length of the robot

Z +ve is more height
Y +ve is left
X +ve is forward

For each leg X,Y and Z has a different coordinate system
"""

#Modules to import
import pybullet as p
import time
import pybullet_data
from math import *

##Robot Parameters
LegOffset = 73.81
LowerLimb = 250
UpperLimb = 250
BodyLength = 593.3
BodyWidth = 250

##############################################################################################################################

## The below code basically controls only for a single leg -- IK for leg
def legik(x,y,z,leg):
    if (leg == 1) or (leg == 3):
        y = y*-1
    AbsoluteHeight = z
    SidewaysStep = y + LegOffset
    StepLength = x
    HipAngle = atan((SidewaysStep) / AbsoluteHeight)
    Hypotenuse = sqrt(AbsoluteHeight**2 + SidewaysStep**2)
    HipAngle1 = asin(LegOffset/Hypotenuse)
    TotalHipAngle = (pi/2 - HipAngle1) + HipAngle
    TotalHipAngle = TotalHipAngle - pi/2
    CurrentHeight = sqrt(Hypotenuse**2 - LegOffset**2)
    ShoulderAngle1 = atan(StepLength / CurrentHeight)
    LegLength = sqrt(CurrentHeight**2 + StepLength**2)
    KneeAngle = acos((UpperLimb**2 + LowerLimb**2 - LegLength**2)/(2*UpperLimb*LowerLimb))
    ShoulderAngle = (pi - KneeAngle)/2
    TotalShoulderAngle = ShoulderAngle + ShoulderAngle1
    return TotalHipAngle,TotalShoulderAngle,(pi - KneeAngle)*-1

##############################################################################################################################
##Rotation stuff

def Rotation(Xc,Yc,Zc,yawangle):
    YawAngle = yawangle
    YawAngle = YawAngle*(pi/180) 
    Yuc = [Yc[0] - ((BodyWidth/2)+LegOffset), Yc[1] + (BodyWidth/2)+LegOffset, Yc[2] - ((BodyWidth/2)+LegOffset), Yc[3] + (BodyWidth/2)+LegOffset]
    Xuc = [(Xc[0] - (BodyLength/2)), Xc[1] - (BodyLength/2), (Xc[2] + (BodyLength/2)), Xc[3] + (BodyLength/2)]
    # Ytest = [Yuc[0] - (BodyLength/2), Yuc[1] - (BodyLength/2), Yuc[2]*-1 - (BodyLength/2), Yuc[3]*-1 - (BodyLength/2)]
    # Xtest = [Xuc[0]*-1 - (BodyWidth/2), Xuc[1] - (BodyWidth/2), Xuc[2]*-1 - (BodyWidth/2), Xuc[3] - (BodyWidth/2)] 
    ExistingAngle = []
    DemandYaw = []
    Radius = []
    for i in range(4):
        ExistingAngle += [atan(Yuc[i]/Xuc[i])]
    for i in range(4):
        DemandYaw += [ExistingAngle[i] + YawAngle]
        Radius += [Yuc[i]/sin(ExistingAngle[i])]
    Xnew = []
    Ynew = []
    for i in range(4):
        Xnew += [Radius[i]*cos(DemandYaw[i])]
        Ynew += [Radius[i]*sin(DemandYaw[i])]
    Ynewu = [Ynew[0] + (BodyWidth/2)+LegOffset, Ynew[1] - (BodyWidth/2)-LegOffset, Ynew[2] + (BodyWidth/2)+LegOffset, Ynew[3] - (BodyWidth/2)-LegOffset]
    Xnewu = [Xnew[0]+(BodyLength/2), Xnew[1] + (BodyLength/2), Xnew[2] - (BodyLength/2), Xnew[3] - (BodyLength/2)] 
    return Ynewu, Xnewu

##############################################################################################################################

# pybullet initialization
## in the pybullet environment all units are in metres
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.setGravity(0,0,-10)
p.setPhysicsEngineParameter(enableFileCaching=0)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,.55]
# startPos = [0,0,2]
startOrientation = p.getQuaternionFromEuler([0,0,0])
quad = p.loadURDF("quadf.urdf",startPos, startOrientation,useFixedBase=False)
no = p.getNumJoints(quad)

for k in range(no):
    info = p.getJointInfo(quad,k)
    print(info[0],": ",info[1])

##initialization of various joint parameters

# positions = [a1,b1,c1,a2,-b2,-c2,a3,b3,c3,a4,-b4,-c4]
force = [9,9,9,9,9,9,9,9,9,9,9,9]
velocity = [1,1,1,1,1,1,1,1,1,1,1,1]
indices = [0,1,2,3,4,5,6,7,8,9,10,11]


##Initial Standing position
Z = 350
Y = 0
X = 0
a,b,c = legik(X,Y,Z,1)
print (a*180/pi,b*180/pi,c*180/pi)
positions = [a,b,c,a,-b,-c,a,b,c,a,-b,-c]
p.setRealTimeSimulation(1)
p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
time.sleep(2)

#Variables for walking
counter = 0
X1 = 0
Y1 = 0

##Coordinate = [left front leg, right front leg, left back leg, right back leg]
Xc = [0,0,0,0]
Yc = [0,0,0,0]
Zc = [350,350,350,350]
Ynewu, Xnewu = Rotation(Xc,Yc,Zc,0)

# for i in range(4):
    # print (i+1, Xnewu[i],Ynewu[i])

a1,b1,c1 = legik(Xnewu[0],Ynewu[0],Zc[0],1)
a2,b2,c2 = legik(Xnewu[1],Ynewu[1],Zc[1],2)
a3,b3,c3 = legik(Xnewu[2],Ynewu[2],Zc[2],3)
a4,b4,c4 = legik(Xnewu[3],Ynewu[3],Zc[3],4)

print(-a1*180/pi,b1*180/pi,c1*180/pi)
print(-a2*180/pi,-b2*180/pi,-c2*180/pi)
print(-a3*180/pi,b3*180/pi,c3*180/pi)
print(-a4*180/pi,-b4*180/pi,-c4*180/pi)

Y1 = 0
X1 = 0
while True:
    
    counter +=1
    
    # sideways Translation
    # for i in range(Y1,150):
        # Y = i
        # a,b,c = legik(X,Y,Z,1)
        # a1,b1,c1 = legik(X,Y,Z,2)
        # positions = [-a,b,c,-a1,-b1,-c1,-a,b,c,-a1,-b1,-c1]
        # p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        # time.sleep(.02)
    # time.sleep(.5)
    # Y1 = 150
    # for i in range(Y1,-150,-1):
        # Y = i
        # a,b,c = legik(X,Y,Z,1)
        # a1,b1,c1 = legik(X,Y,Z,2)
        # positions = [-a,b,c,-a1,-b1,-c1,-a,b,c,-a1,-b1,-c1]
        # p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        # time.sleep(.02)
        
    #stepwise Translation
    # for i in range(X1,150):
        # X = i
        # a,b,c = legik(X,Y,Z,1)
        # a1,b1,c1 = legik(X,Y,Z,2)
        # positions = [-a,b,c,-a1,-b1,-c1,-a,b,c,-a1,-b1,-c1]
        # p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        # time.sleep(.02)
    # time.sleep(.5)
    # X1 = 150
    # for i in range(X1,-150,-1):
        # X = i
        # a,b,c = legik(X,Y,Z,1)
        # a1,b1,c1 = legik(X,Y,Z,2)
        # positions = [-a,b,c,-a1,-b1,-c1,-a,b,c,-a1,-b1,-c1]
        # p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        # time.sleep(.02)
    # X1 = -150
    # time.sleep(.5)
    
    for i in range(0,20):
        Ynewu, Xnewu = Rotation(Xc,Yc,Zc,i)
        a1,b1,c1 = legik(Xnewu[0],Ynewu[0],Zc[0],1)
        a2,b2,c2 = legik(Xnewu[1],Ynewu[1],Zc[1],2)
        a3,b3,c3 = legik(Xnewu[2],Ynewu[2],Zc[2],3)
        a4,b4,c4 = legik(Xnewu[3],Ynewu[3],Zc[3],4)
        positions = [-a1,b1,c1,-a2,-b2,-c2,-a3,b3,c3,-a4,-b4,-c4]
        p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        time.sleep(.02)
    time.sleep(.5)
    for i in range(20,-20,-1):
        Ynewu, Xnewu = Rotation(Xc,Yc,Zc,i)
        a1,b1,c1 = legik(Xnewu[0],Ynewu[0],Zc[0],1)
        a2,b2,c2 = legik(Xnewu[1],Ynewu[1],Zc[1],2)
        a3,b3,c3 = legik(Xnewu[2],Ynewu[2],Zc[2],3)
        a4,b4,c4 = legik(Xnewu[3],Ynewu[3],Zc[3],4)
        positions = [-a1,b1,c1,-a2,-b2,-c2,-a3,b3,c3,-a4,-b4,-c4]
        p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        time.sleep(.02)
    time.sleep(.5)
    for i in range(-20,0):
        Ynewu, Xnewu = Rotation(Xc,Yc,Zc,i)
        a1,b1,c1 = legik(Xnewu[0],Ynewu[0],Zc[0],1)
        a2,b2,c2 = legik(Xnewu[1],Ynewu[1],Zc[1],2)
        a3,b3,c3 = legik(Xnewu[2],Ynewu[2],Zc[2],3)
        a4,b4,c4 = legik(Xnewu[3],Ynewu[3],Zc[3],4)
        positions = [-a1,b1,c1,-a2,-b2,-c2,-a3,b3,c3,-a4,-b4,-c4]
        p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        time.sleep(.02)

cubePos, cubeOrn = p.getBasePositionAndOrientation(quad)
print(cubePos,cubeOrn)
p.disconnect()
