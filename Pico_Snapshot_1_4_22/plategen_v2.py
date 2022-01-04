import kinematicsfunctions as kf

#This is the plate generation function. It is used to remap the well locations if the default locations are inaccurate.
def remap_plate(A1,A6,A12,H1,H6,H12,L1,L2,L3,origin):
    # Defines the function to interpolate values between the calibration wells
    def linear_interpolate(startwell,endwell,distance):
        x_total = endwell[0]-startwell[0]
        y_total = endwell[1]-startwell[1]
        dx = x_total/distance
        dy = y_total/distance
        
        row = []
        for x in range(distance+1):
            if x == 0:
                row.append(startwell)
            elif x == distance:
                row.append(endwell)
            else:
                row.append([startwell[0] + dx*x,startwell[1] + dy*x])
        
        return(row)

    #Calculates the outer edges
    rowa1_6 = linear_interpolate(A1,A6,5)
    rowa6_12 = linear_interpolate(A6,A12,6)
    
    
    rowh1_6 = linear_interpolate(H1,H6,5)
    rowh6_12 = linear_interpolate(H6,H12,6)

    col1 = linear_interpolate(A1,H1,7)
    col6 = linear_interpolate(A6,H6,7)
    col12 = linear_interpolate(A12,H12,7)

    #Creates a nested list, and then fills it row by row
    loc_col = []

    for i in range(len(col1)):
        loc_col.append(linear_interpolate(col1[i],col6[i],5)+ linear_interpolate(col6[i],col12[i],6)[1:len(linear_interpolate(col6[i],col12[i],6))])

    #unpack positions into an x coord list, and y coord list
    xpos = []
    ypos = []

    for i in range(len(loc_col)):
        for ii in range(len(loc_col[i])):
            xpos.append(loc_col[i][ii][0])
            ypos.append(loc_col[i][ii][1])
            
    # Uses inverse kinematics to convert to thetas
    theta1 = []
    theta2 = []

    for i in range(len(xpos)):
        output = kf.inverse_kinematics(L1,L2,L3,origin,[xpos[i],ypos[i]])
        theta1.append(output[0])
        theta2.append(output[1])
        
    #Define lists to fill well ID list
    row = ["a","b","c","d","e","f","g","h"]
    column = list(map(str, list(range(1,13))))
    wellids = []

    #Fills list
    for a in range(len(row)):
        for x in range(len(column)):
            wellids.append(row[a] + column[x])
        
    return [theta1,theta2,wellids]
    
