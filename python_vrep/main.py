# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import sim
import sys
#import cv2
import numpy as np
from scipy.linalg import expm, logm
import time
import matplotlib.pyplot as mlp
from matplotlib.pyplot import imread
PI = np.pi
"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
    #M = np.eye(4)
	#S = np.zeros((6,6))
    S = np.array([[0,0,1,150,150,0],[0,1,0,-162,0,-150],[0,1,0,-162,0,94],[0,1,0,-162,0,307],[1,0,0,0,162,-260],[0,1,0,-162,0,390]])
    #S = np.array([[0,0,0,0,1,0],[0,1,1,1,0,1],[1,0,0,0,0,0],[150,-162,-162,-162,0,-162],[1,0,0,0,162,-260],[0,1,0,-162,0,390]])
    M = np.array([[0,-1,0,390],[0,0,-1,401],[1,0,0,215.5],[0,0,0,1]])
	#S = S.T


	
	# ==============================================================#
    return M, S


"""
Function that calculates encoder numbers for each motor
"""
def expand_S(S,i):
	S_expanded = np.array([[0,-S[i][2],S[i][1],S[i][3]],[S[i][2],0,-S[i][0],S[i][4]],[-S[i][1],S[i][0],0,S[i][5]],[0,0,0,0]])
	return S_expanded
 
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	#theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	#T = np.eye(4)

	M, S = Get_MS()
	inter = np.linalg.multi_dot([expm(expand_S(S,0)*theta1),expm(expand_S(S,1)*theta2),expm(expand_S(S,2)*theta3),expm(expand_S(S,3)*theta4),expm(expand_S(S,4)*theta5),expm(expand_S(S,5)*theta6)])
	T = np.dot(inter,M)




	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
    xgrip = xWgrip 
    ygrip = yWgrip 
    zgrip = zWgrip 
    theta=[0,0,0,0,0,0]
    l01 = 152 
    l02 = 120
    l03 = 244
    l04 = 93
    l05 = 213
    l06 = 83
    l07 = 83
    l08 = 82
    l09 = 0
    l10 = 59 

    xcen = xgrip - l09 * np.cos(np.radians(yaw_WgripDegree))
	
    ycen = ygrip - l09 * np.sin(np.radians(yaw_WgripDegree))
	
    zcen = zgrip 
	
    distcen = np.sqrt(xcen**2 + ycen**2)
	
    theta1 = np.arctan2(ycen,xcen) - np.arcsin((l06+27)/(np.sqrt(xcen**2 + ycen**2)))
	
    theta6 = PI / 2 + theta1 - np.radians(yaw_WgripDegree)
	
    x3end = (np.sqrt(distcen**2 - 110**2)-83)*np.cos(theta1)
	
    y3end = (np.sqrt(distcen**2 - 110**2)-83)*np.sin(theta1)
	
    z3end = zcen + l08 + l10
	
    distend = np.sqrt(x3end**2 + y3end**2)

    theta3 = PI - np.arccos((l03**2 + l05**2 - distend**2 - (z3end-l01)**2)/(2*l03*l05))
	
    theta2 = -np.arcsin(np.sin(PI - theta3)*l05 / np.sqrt(distend**2+(z3end-l01)**2))-np.arctan2((z3end - l01),distend)

    theta4 = - theta2 -theta3 

    theta5 = -PI/2

    #theta=[theta1+ PI,theta2,theta3,theta4- (0.5*PI),theta5,theta6]
    theta=[theta1+PI,theta2+85*PI/180,theta3,-5*PI/180+theta4,theta5,theta6]
    print(theta1/PI*180.0,theta2/PI*180,theta3/PI*180,theta4/PI*180,theta5/PI*180,theta6/PI*180)
    return theta






	# ==============================================================#
    pass

########################################################################################
print ('Program started')
sim.simxFinish(-1)
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)
if clientID!=-1:
    print ('Connected to remote API server')
    
else:
    print ('Failed connecting to remote API server')
    sys.exit()

binary_img = imread("/home/ur3/Desktop/binary_apple.png")
row,col = binary_img.shape 
ur3_handle = np.zeros(6)
for i in range(0,6):
    errorCode, ur3_handle[i] = sim.simxGetObjectHandle(clientID,"UR3_joint"+str(i+1),sim.simx_opmode_oneshot_wait)

errorCode, vision_sensor_handle = sim.simxGetObjectHandle(clientID,"Vision_sensor",sim.simx_opmode_oneshot_wait)
errorCode,resolution,image=sim.simxGetVisionSensorImage(clientID,vision_sensor_handle,1, sim.simx_opmode_blocking)
im = np.array(image,dtype =np.uint8)
im.resize(resolution[0],resolution[1])
mlp.imshow(im)
#print(np.mean(im))
#im = im > np.mean(im)
#go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]
binary_img[binary_img>np.mean(binary_img)] = 255
for i in range(0,row):
    for j in range(0,col):
        if binary_img[i][j] == 255:
            theta = lab_invk(400+(i-15)*10,(j-15)*10,0,0)  
            print(500+(i-15)*10,(j-15)*10,0)
            for k in range(0,6):
                #time.sleep(1)   
                errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[k]),theta[k],sim.simx_opmode_oneshot_wait)
                
            errorCode=sim.simxSetIntegerSignal(clientID,'paint_enable',1,sim.simx_opmode_oneshot)
            time.sleep(1)
        else:
            errorCode=sim.simxSetIntegerSignal(clientID,'paint_enable',0,sim.simx_opmode_oneshot)
#target_loc = [[400,300,0],[400,280,0],[400,260,0],[400,240,0],[400,220,0],[400,200,0],[400,180,0],[400,160,0],[400,140,0],[400,120,0],[400,100,0]]#,[200,90,10]]
#target_loc =[[300,300,0]]
#theta = np.zeros((len(target_loc),6))

#for i in range(0,len(target_loc)):
 #   theta[i] = lab_invk(target_loc[i][0],target_loc[i][1],target_loc[i][2]-5,0)
#for j in range(0,len(target_loc)):
 #   for i in range(0,6):
  #      time.sleep(1)
   #     errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[i]),theta[j][i],sim.simx_opmode_oneshot_wait)


#else: im = 1

mlp.imshow(im)



#sim.simxFinish(clientID)