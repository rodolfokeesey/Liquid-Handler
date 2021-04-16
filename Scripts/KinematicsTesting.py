#Importing functions

import math
import matplotlib.pyplot as plt
import kinematicsfunctions

#%% CALLING THE END COORDINATE -> SERVO POSITION FUNCTION
L1 = 7
L2 = 3
L3 = 10
origin = [0,0]
p4 = [12.45,-1.65]
solutions = kinematicsfunctions.inverse_kinematics(L1,L2,L3,origin,p4)

servo1theta = [0]
servo2theta = [1]
p1 = solutions[2]
p2 = solutions[3]
p3 = solutions[4]                    

#%% Plotting the results

fig, ax = plt.subplots()
ax.plot([p1[0],origin[0]],[p1[1],origin[1]])
ax.plot([p2[0],origin[0]],[p2[1],origin[1]])
ax.plot([p2[0],p3[0]],[p2[1],p3[1]])
ax.plot([p1[0],p3[0]],[p1[1],p3[1]])
ax.plot([p1[0],p4[0]],[p1[1],p4[1]])
ax.set_xlim(-7,14)
ax.set_ylim(-7,14)

#%% CALLING THE WELL ID -> SERVO POSITION FUNCTION

solutions2 = kinematicsfunctions.angle_lookup("a12")
servo1theta = solutions2[0]
servo2theta = solutions2[1]