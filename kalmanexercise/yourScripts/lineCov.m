function [sigmazp] = lineCov(zw,pose,poseCov)

%H = [0 0 -1; -cos(zw(1)) -sin(zw(1)) 0];
H = [0, 0, -1; -cos(zw(1)), -sin(zw(1)), 0.28*cos(zw(1))*sin(pose(3))-0.28*sin(zw(1))*cos(pose(3))];

sigmazp = H * poseCov * H';

end

