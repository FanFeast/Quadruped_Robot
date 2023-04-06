from math import *

#Variables that control movement
Z = 250
y = 0
X = 0

#Initial Variables 
LegOffset = 73.81
LowerLimb = 250
UpperLimb = 250

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
    
##Rotation stuff
## coordinate = [left front leg, right front leg, left back leg, right back leg]
Xc = [0,0,0,0]
Yc = [0,0,0,0]
Zc = [350,350,350,350]
BodyLength = 593.3
BodyWidth = 250

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
    # return Ynewu, Xnewu
    for i in range(4):
        print(i+1, Xnewu[i],Ynewu[i])
    
    #Pitch calculation
    Xu = []
    Yu = []
    Zu = Zc
    Pitch = pitchangle*pi/180
    # if (leg == 1 or leg == 2):
        # Pitch = -Pitch
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
    # return Yu,Xnu2,Znu
    
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
    
Ynewu, Xnewu, Znewu = Rotation(Xc,Yc,Zc,10,0,0)

# print("LegDiffRoll ",LegDiffRoll)
    # print("BodyDiffRoll ",BodyDiffRoll)
    # print("LegDiffRoll ",LDR)
    # print("FootDisplacementRoll ", FDR)
    # print("FootDisplacementangleRoll ", FDAR)
    # print("Hypotenuse/new height ",Zn1)
    # print("Footwholeangleroll ",FWAR)
    # print("Zn2 ",Zn2)
    # print("Yn1 ",Yn1)
print("")
for i in range(4):
    print (i+1, Xnewu[i],Ynewu[i],Znewu[i])



a1,b1,c1 = legik(Xnewu[0],Ynewu[0],Znewu[0],1)
a2,b2,c2 = legik(Xnewu[1],Ynewu[1],Znewu[1],2)
a3,b3,c3 = legik(Xnewu[2],Ynewu[2],Znewu[2],3)
a4,b4,c4 = legik(Xnewu[3],Ynewu[3],Znewu[3],4)

# print ("")
# print(-a1*180/pi,b1*180/pi,c1*180/pi)
# print(a2*180/pi,-b2*180/pi,-c2*180/pi)
# print(-a3*180/pi,b3*180/pi,c3*180/pi)
# print(a4*180/pi,-b4*180/pi,-c4*180/pi)

# a,b,c = legik(0,0,250)
# print(a*180/pi,b*180/pi,c*180/pi)

