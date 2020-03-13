
function [ poseOut, covOut ] = positionPrediction(poseIn,covIn,delSr,delSl)
%[poseOut, covOut] = POSITIONPREDICTION(poseIn,covIn,delSr,delSl) perform
%one step of robot pose prediction from a set of wheel displacements
%   poseIn = old robot pose
%   covIn = uncertainty on the old robot pose
%   delSr = right wheel linear displacement
%   delSl = left wheel linear displacement


%% Constants
% The robot parameters are read globally, odoB is the wheel separation, kR
% and kL are the odometry uncertainty parameters
global odoB_kf kR_kf kL_kf
odoB = odoB_kf;
kR = kR_kf;
kL = kL_kf;

%% pose update

poseOut = poseIn + [(delSr + delSl)/2*cos(poseIn(3)+(delSr - delSl)/2/odoB), (delSr + delSl)/2*sin(poseIn(3)+(delSr - delSl)/2/odoB), (delSr - delSl)/odoB]';

%% Covariance update

delS = (delSr + delSl)/2;
delTheta = (delSr - delSl)/odoB;
Fx = [1 0 -delS*sin(poseIn(3) + delTheta/2); 0 1 delS*cos(poseIn(3) + delTheta/2); 0 0 1];
FdelS = [0.5*cos(poseIn(3) + delTheta/2) - delS/2/odoB*sin(poseIn(3) + delTheta/2), 0.5*cos(poseIn(3) + delTheta/2) + delS/2/odoB*sin(poseIn(3) + delTheta/2);
    0.5*sin(poseIn(3) + delTheta/2) + delS/2/odoB*cos(poseIn(3) + delTheta/2), 0.5*sin(poseIn(3) + delTheta/2) - delS/2/odoB*cos(poseIn(3) + delTheta/2);
    1/odoB, -1/odoB];
covOut = Fx*covIn*Fx' + FdelS*diag([kR*abs(delSr), kL*abs(delSl)])*FdelS';

end
