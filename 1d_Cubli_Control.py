import numpy as np
import sim
import sys
import time
import scipy.linalg as sp

def rotate(rot_vel, joint):
    err_code = sim.simxSetJointTargetVelocity (clientID, joint,rot_vel,sim.simx_opmode_streaming)

def stop(joint):
    err_code = sim.simxSetJointTargetVelocity (clientID, joint,0,sim.simx_opmode_streaming)

#-----Try to connect---------------
sim.simxFinish(-1)
your_IP='10.0.0.57'
clientID = sim.simxStart(your_IP,19999,True,True,10000,5)
if clientID != -1:
    print("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit ("could not connect")

#-----Start the Paused Simulation
err_code = sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot)
#-----Initialize Joint Handles---------
err_code,j_c = sim.simxGetObjectHandle(clientID,"Joint",sim.simx_opmode_blocking)
err_code,motor = sim.simxGetObjectHandle(clientID,"Motor",sim.simx_opmode_blocking)

l = 0.085
lb = 0.075
mb = 0.419
mw = 0.204
Ib = 3.34/(10*10*10)
Iw = 0.57/(10*10*10)
Cb = 0# 1.02/(10*10*10) no friction in coppeliasim between the wheels and body
Cw = 0# 0.05/(10*10*10)
g = 9.8
Km = 1# 0.0251          no current control in coppeliasim, torque is direct


#continuous time state matricies
A = np.array([[0,1,0],
             [(mb*lb+mw*l)*g/(Ib+mw*l*l),-Cb/(Ib+mw*l*l),Cw/(Ib+mw*l*l)],
             [-1*(mb*lb+mw*l)*g/(Ib+mw*l*l),Cb/(Ib+mw*l*l),-Cw*(Ib+Iw+mw*l*l)/(Iw*(Ib+mw*l*l))]])

print(A)

B = np.array([[0,-Km/(Ib+mw*l*l),Km*(Ib+Iw+mw*l*l)/(Iw*(Ib+mw*l*l))]])
             
print(B)

#initializations of some variables

I = [[1,0,0],[0,1,0],[0,0,1]]              #3x3 identity matrix
x=[[0],[0],[0]]                            #state matrix
K = np.array([[-39.1481,-4.9963,-0.0057]]) #LQR controller coefficients computed in matlab
first_run = True
T=0
newT=0

#jump up from rest
rotate(-50,motor)
time.sleep(2)         #let motor speed up for a bit
stop(motor)           #coppeliasim motor is set to lock when velocity is equal to zero just like the real cubli
time.sleep(0.3)       #wait a little before trying to control

while True:
    #get states
    err_code, position = sim.simxGetJointPosition(clientID,j_c,sim.simx_opmode_blocking) 
    err_code, linear, angular_c = sim.simxGetObjectVelocity(clientID,j_c,sim.simx_opmode_blocking)
    err_code, linear, angular_m = sim.simxGetObjectVelocity(clientID,motor,sim.simx_opmode_blocking)
    x=np.array([[position+np.pi/4,angular_c[0],angular_m[0]]])

    #define LQR input
    u = -1*np.matmul(K.T,x)

    #measure time since last loop
    oldT=newT
    newT=time.time()
    T = newT - oldT
    if(first_run == True):
        T=0
        first_run = False
        x = np.matmul(np.array(I+T*A),x.T) + T*B
    
    #define new states
    x = np.array([[x[0,0]],[x[1,0]],[x[2,0]]])
    print(x[2])
    #change motor velocity
    rotate(x[2],motor)
    
        
            
        



 
