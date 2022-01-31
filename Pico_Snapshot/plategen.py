import kinematicsfunctions as kf

# This function interpolates the remaining well locations from the four corner wells. Used when setting up a new plate, or if the Sidekick is plating inaccurately.
def remap_plate(A1,A12,H1,H12,L1,L2,L3,origin):
    """Given the four corner wells and the Sidekick dimensions, interpolate the other well locations"""
    def linear_interpolate(startwell,endwell,wells):
        """Interpolates the coordinates between a given start well, and end well."""
        
        dist_between_wells_x = (endwell[0] - startwell[0])/ (wells-1)
        dist_between_wells_y = (endwell[1] - startwell[1]) / (wells-1)

        final_coordinates = [[startwell[0] + i * dist_between_wells_x, startwell[1] + i * dist_between_wells_y] for i in range(wells)]
        return final_coordinates


    # Calculates the outer edges
    first_col = linear_interpolate(A1, H1, 8)
    last_col = linear_interpolate(A12, H12,8)
    all_coord = [linear_interpolate(first_col[i], last_col[i], 12) for i in range(len(first_col))]

    # Flattens list
    flattened_coords = [val for sublist in all_coord for val in sublist]

    # Uses inverse kinematics to convert to thetas
    thetas = [kf.inverse_kinematics(L1,L2,L3,origin,flattened_coords[i]) for i in range(len(flattened_coords))]
    theta1 = [thetas[i][0] for i in range(len(thetas))]
    theta2 = [thetas[i][1] for i in range(len(thetas))]

    # Fills well ID list
    row = ["a","b","c","d","e","f","g","h"]
    column = list(map(str, list(range(1,13))))
    wellids = [row[i] + column[j] for i in range(len(row)) for j in range(len(column))]
    
    return [theta1,theta2,wellids]
