function [sigmazp] = lineCov(zw,pose,poseCov)

H = [0 0 -1; -cos(zw(1)) -sin(zw(1)) 0];

sigmazp = H * poseCov * H';

end

