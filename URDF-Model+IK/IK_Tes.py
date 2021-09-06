import pybullet as p
import time
import serial
coordinate=[.5,.9,0.3]
rawdata=[]
file_name = "URDF.urdf"
p.connect(p.GUI)
arm = p.loadURDF(file_name, useFixedBase=1)


numJoints = p.getNumJoints(arm)
for i in range(numJoints):
    print(p.getJointInfo(arm,i))
    
    
angle1, angle2, angle3, angle4, angle5,angle6,=p.calculateInverseKinematics(arm,5,coordinate)
p.setJointMotorControl2(bodyIndex=arm,
                        jointIndex=0,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angle1,
                        force=20)

p.setJointMotorControl2(bodyIndex=arm,
                        jointIndex=1,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angle2,
                        force=20)

p.setJointMotorControl2(bodyIndex=arm,
                        jointIndex=2,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angle3,
                        force=20)

p.setJointMotorControl2(bodyIndex=arm,
                        jointIndex=3,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angle4,
                        force=20)


p.setJointMotorControl2(bodyIndex=arm,
                        jointIndex=4,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angle5,
                        force=20)

p.setJointMotorControl2(bodyIndex=arm,
                        jointIndex=5,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angle6,
                        force=20)


print(angle1, angle2, angle3, angle4, angle5,angle6)
previoustime=0
interval=0.1
previoustime2=0
interval2=0.5
while True:
    
    if (time.time()>previoustime+interval):
        p.stepSimulation()
        previoustime=time.time()
        print(p.getLinkState(arm,5,computeForwardKinematics=1))
        

    if(time.time()>previoustime2+interval2): 
         previoustime2=time.time()       
         #print(coordinate)

    
    