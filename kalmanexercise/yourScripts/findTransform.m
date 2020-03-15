function transform = findTransform(odoPose, pose)
% transform = FINDTRANSFORM(odoPose,pose)
% Find the transformation from the world coordinates to the odometry
% coordinates given a pose in the odometry coordinates (odoPose) and the
% same point in the world coordinates (pose). The output (transform) is
% simply the origo of the odometry coordinates in the world coordinates

    
    thetat = odoPose(3) - pose(3);
    rot_matrix = [cos(thetat) -sin(thetat); sin(thetat) cos(thetat)];
    
    transform = [odoPose(1:2) - rot_matrix*pose(1:2); thetat];
end