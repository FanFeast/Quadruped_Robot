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

def Rotation(Xc,Yc,Zc,yawangle,pitchangle,rollangle):
    #Yaw Calculation
    YawAngle = yawangle
    YawAngle = YawAngle*(pi/180) 
    Yuc = [Yc[0] - ((BodyWidth/2)+LegOffset), Yc[1] + (BodyWidth/2)+LegOffset, Yc[2] - ((BodyWidth/2)+LegOffset), Yc[3] + (BodyWidth/2)+LegOffset]
    Xuc = [(Xc[0] - (BodyLength/2)), Xc[1] - (BodyLength/2), (Xc[2] + (BodyLength/2)), Xc[3] + (BodyLength/2)]
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
    Ynewu = [-(Ynew[0] + (BodyWidth/2)+LegOffset), -(Ynew[1] - (BodyWidth/2)-LegOffset), -(Ynew[2] + (BodyWidth/2)+LegOffset), -(Ynew[3] - (BodyWidth/2)-LegOffset)]
    Xnewu = [Xnew[0]+(BodyLength/2), Xnew[1] + (BodyLength/2), -(Xnew[2] - (BodyLength/2)), -(Xnew[3] - (BodyLength/2))] 
    
    #Pitch calculation
    Xu = []
    Yu = []
    Zu = Zc
    Pitch = pitchangle*pi/180
    for i in range(4):
        if (i== 2 or i == 3):
            Xu += [Xnewu[i]*-1]
        else:
            Xu += [Xnewu[i]]
        Yu += [Ynewu[i]]
        
    LegDiffPitch = [sin(-Pitch)*BodyLength/2, sin(-Pitch)*BodyLength/2,sin(Pitch)*BodyLength/2,sin(Pitch)*BodyLength/2]
    BodyDiffPitch = [cos(-Pitch)*BodyLength/2,cos(-Pitch)*BodyLength/2,cos(Pitch)*BodyLength/2,cos(Pitch)*BodyLength/2]
    LDP = []
    FDP = []
    FDAP = []
    Zu1 = []
    FWAP = []
    Znu = []
    Xnu = []
    for i in range(4):
        LDP += [Zu[i] - LegDiffPitch[i]]
        FDP += [((BodyDiffPitch[i]-(BodyLength/2))*-1) + Xu[i]]
        FDAP += [atan(FDP[i]/LDP[i])]
        Zu1 += [LDP[i]/cos(FDAP[i])]
        FWAP += [FDAP[i]+Pitch]
        Znu += [cos(FWAP[i])*Zu1[i]]
        Xnu += [sin(FWAP[i])*Zu1[i]]
    Xnu2 = [Xnu[0],Xnu[1],Xnu[2],Xnu[3]]   
    
    #Roll calculations
    Xn = Xnu2
    Yn = []
    Zn = Znu
    for i in range(4):
        if (i== 0 or i == 2):
            Yn += [Yu[i]*-1]
        else:
            Yn += [Yu[i]]
    Roll = rollangle*pi/180
    LegDiffRoll = [sin(-Roll)*BodyWidth/2,sin(Roll)*BodyWidth/2,sin(-Roll)*BodyWidth/2,sin(Roll)*BodyWidth/2] 
    BodyDiffRoll = [cos(-Roll)*BodyWidth/2,cos(Roll)*BodyWidth/2,cos(-Roll)*BodyWidth/2,cos(Roll)*BodyWidth/2]
    LDR = []
    FDR = []
    FDAR = []
    Zn1 = []
    FWAR = []
    Zn2 = []
    Yn1 = []
    for i in range(4):
        LDR += [Zn[i] - LegDiffRoll[i]]
        FDR += [((BodyDiffRoll[i]-BodyWidth/2)*-1)+LegOffset - Yn[i]]
        FDAR += [atan(FDR[i]/LDR[i])]
        Zn1 += [LDR[i]/cos(FDAR[i])]
    FWAR = [FDAR[0]-Roll,FDAR[1]+Roll,FDAR[2]-Roll,FDAR[3]+Roll]
    for i in range(4):
        Zn2 += [cos(FWAR[i])*Zn1[i]]
        Yn1 += [sin(FWAR[i])*Zn1[i] - LegOffset]
    Yn2 = [-Yn1[0],Yn1[1],-Yn1[2],Yn1[3]]
    return Yn2,Xn,Zn2

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

##Debug sliders
pitch = p.addUserDebugParameter("pitch" , -20 , 20 , 0.)
roll = p.addUserDebugParameter("roll" , -20 , 20 , 0.)
yaw = p.addUserDebugParameter("yaw" , -20 , 20 , 0.)
Absheight = p.addUserDebugParameter("Absolute Height",100,500,350.0)
xtrans = p.addUserDebugParameter("X translation",-200,200,0.0)
ytrans = p.addUserDebugParameter("Y translation",-150,150,0.0)

#Variables for walking
counter = 0
X1 = 0
Y1 = 0
Xc = [0,0,0,0]
Yc = [0,0,0,0]
Zc = [350,350,350,350]

while True:
    counter +=1
    pit = p.readUserDebugParameter(pitch)
    r = p.readUserDebugParameter(roll)
    y = p.readUserDebugParameter(yaw)
    l = p.readUserDebugParameter(xtrans)
    m = p.readUserDebugParameter(ytrans)
    n = p.readUserDebugParameter(Absheight)
    Xc = [l,l,l,l]
    Yc = [m,m,m,m]
    Zc = [n,n,n,n]
    Ynewu, Xnewu, Znewu = Rotation(Xc,Yc,Zc,y,pit,r)
    a1,b1,c1 = legik(Xnewu[0],Ynewu[0],Znewu[0],1)
    a2,b2,c2 = legik(Xnewu[1],Ynewu[1],Znewu[1],2)
    a3,b3,c3 = legik(Xnewu[2],Ynewu[2],Znewu[2],3)
    a4,b4,c4 = legik(Xnewu[3],Ynewu[3],Znewu[3],4)
    positions = [-a1,b1,c1,-a2,-b2,-c2,-a3,b3,c3,-a4,-b4,-c4]
    p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
    time.sleep(.02)
    
    #forward moving
    # Z1 = 300
    # Xi = X
    # d,e,f = legik(X1,Y1,Z1,1)
    # p.setJointMotorControlArray(quad,jointIndices=[0,1,2,9,10,11],controlMode=p.POSITION_CONTROL,targetPositions= [-d,e,f,d,-e,-f],forces=[9,9,9,9,9,9],targetVelocities=[1,1,1,1,1,1])
    # for i in range(X,101):
        # X = i
        # X1 = -i
        # Z1 = Z1 + (50/(101-Xi))
        # a,b,c = legik(X,Y,Z,1)
        # d,e,f = legik(X1,Y1,Z1,1)
        # positions = [-d,e,f,a,-b,-c,-a,b,c,d,-e,-f]
        # p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        # time.sleep(.001)
    # Z = 300
    # Xi = X
    # a,b,c = legik(X,Y,Z,1)
    # p.setJointMotorControlArray(quad,jointIndices=[3,4,5,6,7,8],controlMode=p.POSITION_CONTROL,targetPositions= [a,-b,-c,-a,b,c],forces=[9,9,9,9,9,9],targetVelocities=[1,1,1,1,1,1])
    # for i in range(X,-101,-1):
        # X = i
        # X1 = -i
        # Z = Z + (50/(101+Xi))
        # a,b,c = legik(X,Y,Z,1)
        # d,e,f = legik(X1,Y1,Z1,1)
        # positions = [-d,e,f,a,-b,-c,-a,b,c,d,-e,-f]
        # p.setJointMotorControlArray(quad,jointIndices=indices,controlMode=p.POSITION_CONTROL,targetPositions=positions,forces=force,targetVelocities=velocity)
        # time.sleep(.001)
p.disconnect()
