function [ projectedLine, lineCov ] = projectToLaser( worldLine,poseIn,covIn)
%[projectedLine, lineCov] = PROJECTTOLASER(worldLine,poseIn,covIn) 
%Project a word line to the laser scanner frame given the
%world line, the robot pose and robot pose covariance. Note that the laser
%scanner pose in the robot frame is read globally
%   worldLine: The line in world coordinates
%   poseIn: The robot pose
%   covIn: The robot pose covariance
%
%   projectedLine: The line parameters in the laser scanner frame
%   lineCov: The covariance of the line parameters

%% Constants
global lsrRelPose % The laser scanner pose in the robot frame is read globally


%% Calculation

laserPose = poseIn + [cos(poseIn(3)) -sin(poseIn(3)) 0; sin(poseIn(3)) cos(poseIn(3)) 0; 0 0 1]*lsrRelPose';

projectedLine = [worldLine(1) - laserPose(3), worldLine(2) - laserPose(1)*cos(worldLine(1)) - laserPose(2)*sin(worldLine(1))];

H = [0 0 -1; -cos(worldLine(1)) -sin(worldLine(1)) 0];

lineCov = H * covIn * H';

%lineCov = lineCov(worldLine, poseIn, covIn);
end
