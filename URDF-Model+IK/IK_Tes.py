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
    
angles=[]

angles=p.calculateInverseKinematics(arm,5,coordinate)
print("Angles:")
print(angles)
p.setJointMotorControl2(bodyIndex=arm,
                        jointIndex=0,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angles[0],
                        force=200)

p.setJointMotorControl2(bodyIndex=arm,
                        jointIndex=1,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angles[1],
                        force=200)

p.setJointMotorControl2(bodyIndex=arm,
                        jointIndex=2,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angles[2],
                        force=200)

p.setJointMotorControl2(bodyIndex=arm,
                        jointIndex=3,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angles[3],
                        force=200)


p.setJointMotorControl2(bodyIndex=arm,
                        jointIndex=4,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angles[4],
                        force=200)

p.setJointMotorControl2(bodyIndex=arm,
                        jointIndex=5,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angles[5],
                        force=200)



previoustime=0
interval=0.1
previoustime2=0
interval2=2
while True:
    
    if (time.time()>previoustime+interval):
        p.stepSimulation()
        previoustime=time.time()
        #print(p.getLinkState(arm,5,computeForwardKinematics=1))
        

    if(time.time()>previoustime2+interval2): 
         previoustime2=time.time()       
         #coordinate=str(esp.readline().strip().decode("utf-8")).split(',')
         #print(coordinate)
         print(p.getLinkState(arm,5,computeForwardKinematics=1))

    
    