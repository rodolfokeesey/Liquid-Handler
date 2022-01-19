## Import libraries

import math

#%% All measurements are in cm and degrees
#%% Lookup table, converts wells formatted as 'a1' ... 'h12' to joint angles

def angle_lookup(well,wellids,alltheta1,alltheta2):
    #wellids = ['a12','a11','a10','a9','a8','a7','a6','a5','a4','a3','a2','a1','b12','b11','b10','b9','b8','b7','b6','b5','b4','b3','b2','b1','c12','c11','c10','c9','c8','c7','c6','c5','c4','c3','c2','c1','d12','d11','d10','d9','d8','d7','d6','d5','d4','d3','d2','d1','e12','e11','e10','e9','e8','e7','e6','e5','e4','e3','e2','e1','f12','f11','f10','f9','f8','f7','f6','f5','f4','f3','f2','f1','g12','g11','g10','g9','g8','g7','g6','g5','g4','g3','g2','g1','h12','h11','h10','h9','h8','h7','h6','h5','h4','h3','h2','h1']
    #alltheta1 = [119.4841,117.6138,114.5787,110.2216,104.4481,97.28982,88.94462,79.75737,70.13718,60.45672,50.9865,41.88039,110.6423,108.5257,105.3983,101.1829,95.85575,89.47376,82.18741,74.2247,65.84932,57.31055,48.80589,40.46561,102.4427,100.2429,97.16377,93.17305,88.27757,82.53652,76.06442,69.02013,61.58364,53.92882,46.20107,38.50505,94.71713,92.53403,89.57684,85.83804,81.33793,76.13086,70.30529,63.97631,57.27198,50.31734,43.22037,36.06285,87.31276,85.21027,82.41408,78.93111,74.7873,70.03075,64.73105,58.97418,52.85399,46.46227,39.87944,33.16767,80.08457,78.10734,75.49476,72.26294,68.44019,64.06895,59.20506,53.91427,48.26676,42.3305,36.16477,29.81468,72.88291,71.06822,68.65616,65.66992,62.13932,58.10236,53.60484,48.69798,43.43468,37.86483,32.03041,25.96063,65.53224,63.92112,61.72933,58.98761,55.72801,51.98577,47.79946,43.20942,38.25514,32.97155,27.3848,21.50715]
    #alltheta2 = [171.7007,165.8025,159.4164,152.4932,145.05,137.2078,129.2057,121.3677,114.0266,107.4482,101.7913,97.11273,169.0437,163.2183,157.0534,150.547,143.7473,136.7667,129.7804,123.0038,116.6541,110.9124,105.9007,101.6799,167.3756,161.709,155.8243,149.7399,143.5099,137.229,131.0273,125.0557,119.4647,114.3838,109.9079,106.0939,166.5481,161.0759,155.4819,149.7911,144.0545,138.349,132.7729,127.4364,122.4487,117.9062,113.8844,110.4347,166.4537,161.1825,155.8659,150.5293,145.2169,139.9906,134.926,130.1059,125.6129,121.5214,117.8931,114.7745,167.0239,161.9423,156.8782,151.8537,146.9053,142.0826,137.4445,133.0553,128.979,125.2755,121.9969,119.1865,168.2294,163.3136,158.47,153.7154,149.0787,144.599,140.3229,136.3011,132.5854,129.2257,126.268,123.7544,170.088,165.3015,160.6399,156.1127,151.7408,147.5543,143.59,139.8893,136.4952,133.4516,130.8018,128.5897,]
    
    
    index = wellids.index(well)
    theta1 = alltheta1[index]
    theta2 = alltheta2[index]
        
    return [theta1,theta2]

#%% Find the standard position angle from the positive x axis given a point

def find_standard_position_angle(point):

    x = point[0]
    y = point[1]
    
    if x == 0:
        refangle = 90
    else:
        refangle = math.degrees(math.atan(y/x))
    
    # Finds the quadrant
    
    if x >= 0 and y >= 0:
        quadrant = 1
    elif x <= 0 and y >= 0:
        quadrant = 2
    elif x <= 0 and y <= 0:
        quadrant = 3
    else:
        quadrant = 4
    
    # Finds the standard angle given reference angle and quadrant
    
    if quadrant == 1 or quadrant == 4:
        standardangle = 0 + refangle
    elif quadrant == 2 or quadrant == 3:
        standardangle = 180 + refangle
        
    if standardangle < 0:
        standardangle = standardangle + 360
        
    return(standardangle)

#%% Calculating intersection of two circles (http://paulbourke.net/geometry/circlesphere/), (https://stackoverflow.com/questions/55816902/finding-the-intersection-of-two-circles)

def get_intersections(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d=math.sqrt((x1-x0)**2 + (y1-y0)**2)
    
    # non intersecting
    if d > r0 + r1 :
        return None
    # One circle within other
    if d < abs(r0-r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a=(r0**2-r1**2+d**2)/(2*d)
        h=math.sqrt(r0**2-a**2)
        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d   
        x3=round(x2+h*(y1-y0)/d,4)     
        y3=round(y2-h*(x1-x0)/d,4) 

        x4=round(x2-h*(y1-y0)/d,4)
        y4=round(y2+h*(x1-x0)/d,4)
        
        return (x3, y3, x4, y4)

#%% Inverse Kinematics calculations


def inverse_kinematics(L1,L2,L3,origin,p4):
    
    p1 = get_intersections(p4[0],p4[1],L3,origin[0],origin[1],L1) # Get the possible locations of point 1

    if p1 is None:
        print("No possible orientations")
    else:
        p1a = [p1[0],p1[1]] # Conformation 1
        p1b = [p1[2],p1[3]] # Conformation 2
        
        ## EVALUATES CONFORMATION 1
        
        con1p3vector = [p4[0] - p1a[0],p4[1]-p1a[1]] # Finds the vector extending from p1 to p4
        con1p3standangle = find_standard_position_angle(con1p3vector) # Finds the position angle for that vector
        
        #Calculates p2 and p3 by adding the length of L2 in the opposite direction of the postion angle drawn between p1 and p4
        
        con1p3 = [round(p1a[0] + L2 * math.cos(math.radians(con1p3standangle+180)),4),round(p1a[1] + L2 * math.sin(math.radians(con1p3standangle+180)),4)]
        con1p2 = [round(origin[0] + L2 * math.cos(math.radians(con1p3standangle+180)),4),round(origin[1] + L2 * math.sin(math.radians(con1p3standangle+180)),4)]
        
        #Finds the position angles for conformation 1
        
        con1theta1 = find_standard_position_angle(p1a)
        con1theta2 = find_standard_position_angle(con1p2)
        
        ## EVALUATES CONFORMATION 2, (Same calculation as conformation 1 but with p1b)
        
        con2p3vector = [p4[0] - p1b[0],p4[1]-p1b[1]] # Finds the vector extending from p1 to p4
        con2p3standangle = find_standard_position_angle(con2p3vector) # Finds the position angle for that vector
        
        #Calculates p2 and p3 by adding the length of L2 in the opposite direction of the postion angle drawn between p1 and p4
        
        con2p3 = [round(p1b[0] + L2 * math.cos(math.radians(con2p3standangle+180)),4),round(p1b[1] + L2 * math.sin(math.radians(con2p3standangle+180)),4)]
        con2p2 = [round(origin[0] + L2 * math.cos(math.radians(con2p3standangle+180)),4),round(origin[1] + L2 * math.sin(math.radians(con2p3standangle+180)),4)]
        
        #Finds the position angles for conformation 1
        
        con2theta1 = find_standard_position_angle(p1b)
        con2theta2 = find_standard_position_angle(con2p2)
        
        ## Choosing Between Conformation 1 and 2
        
        theta1check = [con1theta1,con2theta1]
        theta2check = [con1theta2,con2theta2]
        
        #Creates arrays of possible thetas to evaluate
        theta1possible = []
        theta2possible = []
        
        #Checks the mechanical constraints of the possible angles
        for i in range(len(theta1check)):
            if theta1check[i] <= 195:
                theta1possible.append(i)
        
        for i in range(len(theta2check)):
            if theta2check[i] <= 195:
                theta2possible.append(i)
        
        intersect = set.intersection(set(theta1possible),set(theta2possible))
        
        if len(intersect) == 0:
            return print("All possible orientations violate mechanical constraints")
            
        elif len(intersect) > 1:
            #print("Two possible orientations, first orientation selected")
            theta1 = con1theta1
            theta2 = con1theta2
            p1 = p1a
            p2 = con1p2
            p3 = con1p3
        elif list(intersect)[0] == 0: #conformation 1 chosen
            theta1 = con1theta1
            theta2 = con1theta2
            p1 = p1a
            p2 = con1p2
            p3 = con1p3
        elif list(intersect)[0] == 1: #conformation 2 chosen
            theta1 = con2theta1
            theta2 = con2theta2
            p1 = p1b
            p2 = con2p2
            p3 = con2p3
            
        return[theta1,theta2,p1,p2,p3]

#%% MultiChannel Inverse Kinematics

def inverse_kinematics_multi(L1,L2,L3,Ln,N,ptarget,origin):

    # First we need to define the length from P3 to the target nozzle N, we can do this with some trig
    
    if N == "N1" or N == "N3":
        # Formula for this length for nozzles 1 and 3
        nlength = math.sqrt(math.pow((L3-(Ln/math.sqrt(2))),2) + math.pow((Ln/math.sqrt(2)),2))
    elif N == "N2" or N == "N4":
        #Formula for this length for nozzles 2 and 4
        nlength = math.sqrt(math.pow((L3+(Ln/math.sqrt(2))),2) + math.pow((Ln/math.sqrt(2)),2))
    
        
    p1 = get_intersections(ptarget[0],ptarget[1],nlength,origin[0],origin[1],L1) # Get the possible locations of point 1
    
    if p1 is None:
        print("No possible orientations")
    else:
        p1a = [p1[0],p1[1]] # Conformation 1
        p1b = [p1[2],p1[3]] # Conformation 2
        
        ## Solves for N, Conformation 1
        
        n1vector = [ptarget[0]-p1a[0],ptarget[1]-p1a[1]] #finds the n1 vector
        n1standtheta = find_standard_position_angle(n1vector) #Finding the standard position angle for the n1 vector
        
        ## Changes the calculation based on which nozzle we're calculating for
        if N == "N1":
            # For N1 and N3, the triangle drawn faces towards the origin, so the adjacent leg is L3 - Ln/sqrt(2)
            n1thetaoffset = math.degrees(math.atan((Ln/math.sqrt(2))/(L3-(Ln/math.sqrt(2)))))
            # We subtract the offset from the standard angle, because the N1 Nozzle is above center position, so to find the center position we go down
            p4 = [round(p1a[0]+L3*math.cos(math.radians(n1standtheta-n1thetaoffset)),4), round(p1a[1]+L3*math.sin(math.radians(n1standtheta-n1thetaoffset)),4)]
        elif N == "N3":
            n1thetaoffset = math.degrees(math.atan((Ln/math.sqrt(2))/(L3-(Ln/math.sqrt(2)))))
            # We add the offset to the standard angle, because the N3 Nozzle is below the center position, so to find the center position we go up
            p4 = [round(p1a[0]+L3*math.cos(math.radians(n1standtheta+n1thetaoffset)),4), round(p1a[1]+L3*math.sin(math.radians(n1standtheta+n1thetaoffset)),4)]
        elif N == "N2":
            # For N2 and N4, we change the offset calculation. The triangle drawn faces away from the origin, so the adjacent leg is L3 + Ln/sqrt(2)
            n1thetaoffset = math.degrees(math.atan((Ln/math.sqrt(2))/(L3+(Ln/math.sqrt(2)))))
            # We subtract the offset from the standard angle, because the N2 Nozzle is above center position, so to find the center position we go down
            p4 = [round(p1a[0]+L3*math.cos(math.radians(n1standtheta-n1thetaoffset)),4), round(p1a[1]+L3*math.sin(math.radians(n1standtheta-n1thetaoffset)),4)]
        elif N == "N4":
            n1thetaoffset = math.degrees(math.atan((Ln/math.sqrt(2))/(L3+(Ln/math.sqrt(2)))))
            # We add the offset from the standard angle, because the N4 Nozzle is below center position, so to find the center position we go up
            p4 = [round(p1a[0]+L3*math.cos(math.radians(n1standtheta+n1thetaoffset)),4), round(p1a[1]+L3*math.sin(math.radians(n1standtheta+n1thetaoffset)),4)]
        
        ## After using the offset, standard angle, and length of L3 to find P4, we can use our original Inverse Kinematics function
        results = inverse_kinematics(L1,L2,L3,origin,p4)
        con1theta1 = results[0]
        con1theta2 = results[1]
        con1p1 = results[2]
        con1p2 = results[3]
        con1p3 =results[4]
        
        ## Solves for N Conformation 2
    
        n1vector = [ptarget[0]-p1b[0],ptarget[1]-p1b[1]] #finds the n1 vector
        n1standtheta = find_standard_position_angle(n1vector) #Finding the standard position angle for the n1 vector
        
        ## Changes the calculation based on which nozzle we're calculating for
        if N == "N1":
            # For N1 and N3, the triangle drawn faces towards the origin, so the adjacent leg is L3 - Ln/sqrt(2)
            n1thetaoffset = math.degrees(math.atan((Ln/math.sqrt(2))/(L3-(Ln/math.sqrt(2)))))
            # We subtract the offset from the standard angle, because the N1 Nozzle is above center position, so to find the center position we go down
            p4con2 = [round(p1b[0]+L3*math.cos(math.radians(n1standtheta-n1thetaoffset)),4), round(p1b[1]+L3*math.sin(math.radians(n1standtheta-n1thetaoffset)),4)]
        elif N == "N3":
            n1thetaoffset = math.degrees(math.atan((Ln/math.sqrt(2))/(L3-(Ln/math.sqrt(2)))))
            # We add the offset to the standard angle, because the N3 Nozzle is below the center position, so to find the center position we go up
            p4con2 = [round(p1b[0]+L3*math.cos(math.radians(n1standtheta+n1thetaoffset)),4), round(p1b[1]+L3*math.sin(math.radians(n1standtheta+n1thetaoffset)),4)]
        elif N == "N2":
            # For N2 and N4, we change the offset calculation. The triangle drawn faces away from the origin, so the adjacent leg is L3 + Ln/sqrt(2)
            n1thetaoffset = math.degrees(math.atan((Ln/math.sqrt(2))/(L3+(Ln/math.sqrt(2)))))
            # We subtract the offset from the standard angle, because the N2 Nozzle is above center position, so to find the center position we go down
            p4con2 = [round(p1b[0]+L3*math.cos(math.radians(n1standtheta-n1thetaoffset)),4), round(p1b[1]+L3*math.sin(math.radians(n1standtheta-n1thetaoffset)),4)]
        elif N == "N4":
            n1thetaoffset = math.degrees(math.atan((Ln/math.sqrt(2))/(L3+(Ln/math.sqrt(2)))))
            # We add the offset from the standard angle, because the N4 Nozzle is below center position, so to find the center position we go up
            p4con2 = [round(p1b[0]+L3*math.cos(math.radians(n1standtheta+n1thetaoffset)),4), round(p1b[1]+L3*math.sin(math.radians(n1standtheta+n1thetaoffset)),4)]
    
        results = inverse_kinematics(L1,L2,L3,origin,p4con2)
        con2theta1 = results[0]
        con2theta2 = results[1]
        con2p1 = results[2]
        con2p2 = results[3]
        con2p3 =results[4]
        
        ## Choosing Between Conformation 1 and 2
            
        theta1check = [con1theta1,con2theta1]
        theta2check = [con1theta2,con2theta2]
        
        #Creates arrays of possible thetas to evaluate
        theta1possible = []
        theta2possible = []
        
        #Checks the mechanical constraints of the possible angles
        for i in range(len(theta1check)):
            if theta1check[i] <= 195:
                theta1possible.append(i)
        
        for i in range(len(theta2check)):
            if theta2check[i] <= 195:
                theta2possible.append(i)
        
        intersect = set.intersection(set(theta1possible),set(theta2possible))
        
        if len(intersect) == 0:
            print("All possible orientations violate mechanical constraints")
        
        elif len(intersect) > 1:
            #print("Two possible orientations, first orientation selected")
            theta1 = con1theta1
            theta2 = con1theta2
            p1 = p1a
            p2 = con1p2
            p3 = con1p3
            p4 = p4
        elif list(intersect)[0] == 0: #conformation 1 chosen
            theta1 = con1theta1
            theta2 = con1theta2
            p1 = p1a
            p2 = con1p2
            p3 = con1p3
            p4 = p4
        elif list(intersect)[0] == 1: #conformation 2 chosen
            theta1 = con2theta1
            theta2 = con2theta2
            p1 = p1b
            p2 = con2p2
            p3 = con2p3
            p4 =p4con2
            
        finalresults = [theta1,theta2,p1,p2,p3,p4]
        return finalresults

#%% ForwardKinematics

def forward_kinematics(L1,L2,L3,theta1,theta2):
    #Define each of the vertex points by the angle and leg length
    p1 = [L1*math.cos(math.radians(theta1)),L1*math.sin(math.radians(theta1))]
    p2 = [L2*math.cos(math.radians(theta2)),L2*math.sin(math.radians(theta2))]
    p3 = [p1[0]+p2[0],p1[1]+p2[1]]

    #Parallel to L2, so add L3 and L2 to get the total length of the leg, multiply by theta 2, and subtract from p3
    p4 = [p3[0]-(L2+L3)*math.cos(math.radians(theta2)),p3[1]-(L2+L3)*math.sin(math.radians(theta2))]
    
    return p4

#%% Simple movement in the xy plane .5 mm intervals. Up is in reference to the plate and the center of the dispenser.

def motion(setting, theta1, theta2):
    p4 = forward_kinematics(setting['L1'], setting['L2'], setting['L3'], theta1, theta2)
    p4[0] -= setting['offset']
    result = inverse_kinematics(setting['L1'], setting['L2'], setting['L3'],setting['origin'],p4) + p4 
    return result

def up(theta1,theta2):
    #standard dimensions, change if needed.
    setting = {"L1":7, "L2":3, "L3":10, "origin":[0,0], "offset":0.05}
    return motion(setting, theta1, theta2)

def down(theta1,theta2):
    #standard dimensions, change if needed.
    setting = {"L1":7, "L2":3, "L3":10, "origin":[0,0], "offset":0.05}
    return motion(setting, theta1, theta2)

def left(theta1,theta2):
    #standard dimensions, change if needed.
    setting = {"L1":7, "L2":3, "L3":10, "origin":[0,0], "offset":0.05}
    return motion(setting, theta1, theta2)

def right(theta1,theta2):
    #standard dimensions, change if needed.
    setting = {"L1":7, "L2":3, "L3":10, "origin":[0,0], "offset":0.05}
    return motion(setting, theta1, theta2)
    
    