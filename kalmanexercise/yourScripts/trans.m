function odoTargetPose = trans(transform,targetPose)
% odoTargetPose = trans(transform,targetPose)
% Transform a given point in world coordinates (targetPose) to odometry
% coordinates, using the origo of the odometry coordinates in world
% coordinates (transform).

    rot_matrix = [cos(transform(3)) -sin(transform(3)); sin(transform(3)) cos(transform(3))];

    odoTargetPose = [rot_matrix*targetPose(1:2) + transform(1:2); targetPose(3) + transform(3)];
end