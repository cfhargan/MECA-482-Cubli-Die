import numpy as np
import sim
import sys
import time

def rotate(rot_vel, joint):
    err_code = sim.simxSetJointTargetVelocity (clientID, joint,rot_vel,sim.simx_opmode_streaming)

def stop(joint):
    err_code = sim.simxSetJointTargetVelocity (clientID, joint,0,sim.simx_opmode_streaming)

#state space map of flipping cube to the desired side

states = (((0,0,0),(0,-100,0),(100,0,0),(-100,0,0),(0,100,0),(-100,0,0))#side 1 map
        ,((0,100,0),(0,0,0),(0,0,100),(0,0,-100),(0,100,0),(0,-100,0))#side 2 map
        ,((-100,0,0),(0,0,-100),(0,0,0),(-100,0,0),(0,0,100),(100,0,0))#side 3 map
        ,((100,0,0),(0,0,100),(100,0,0),(0,0,0),(0,0,100),(-100,0,0))#side 4 map
        ,((0,-100,0),(0,100,0),(0,0,100),(0,0,-100),(0,0,0),(0,100,0))#side 5 map
        ,((100,0,0),(0,100,0),(-100,0,0),(100,0,0),(0,-100,0),(0,0,0)))#side 6 map


#-----Try to connect---------------
sim.simxFinish(-1)
your_IP='10.0.0.56'
clientID = sim.simxStart(your_IP,19999,True,True,10000,5)
if clientID != -1:
    print("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit ("could not connect")

#-----Start the Paused Simulation
err_code = sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot)
#-----Initialize Joint Handles---------
err_code,jx = sim.simxGetObjectHandle(clientID,"Joint_x",sim.simx_opmode_blocking)
err_code,jy = sim.simxGetObjectHandle(clientID,"Joint_y",sim.simx_opmode_blocking)
err_code,jz = sim.simxGetObjectHandle(clientID,"Joint_z",sim.simx_opmode_blocking)
err_code,accel = sim.simxGetObjectHandle(clientID,"Accelerometer_forceSensor",sim.simx_opmode_blocking)
err_code,accel_mass = sim.simxGetObjectHandle(clientID,"Accelerometer_mass",sim.simx_opmode_blocking)

chosenside = 4 #this is the side it will try to flip to

while True:
    #read accelerometer
    err_code, something, force, something1 = sim.simxReadForceSensor(clientID, accel, sim.simx_opmode_blocking)
    acc = (force[0]*1000,force[1]*1000,force[2]*1000)
    side = 0
    #figure out which side it is on with accelerometer values
    if(acc[2] < -9): 
        side = 1
    if(acc[2] > 9):
        side = 6
    if(acc[0] < -9): 
        side = 5
    if(acc[0] > 9):
        side = 2
    if(acc[1] < -9): 
        side = 4
    if(acc[1] > 9):
        side = 3
    print(side)
    rotate(states[side-1][chosenside-1][0],jx) #use newton simulation
    rotate(states[side-1][chosenside-1][1],jy)
    rotate(states[side-1][chosenside-1][2],jz)
    print("Rotating")
    time.sleep(2)
    stop(jx)
    stop(jy)
    stop(jz)
    print("Stopped")
    time.sleep(5)       
            
        



 
