function [ poseOut, poseCovOut ] = measurementUpdate( poseIn, poseCovIn, matchResult )
%[ poseOut, poseCovOut ] =MEASUREMENTUPDATE ( poseIn, poseCovIn,
%matchResult ) perform extended Kalman filter measurement update on the
%estimated robot pose poseIn with covariance poseCovIn using a set of
%matched predicted and extracted laser scanner lines given in matchResult.
%The arguments are defined as:
%       poseIn: The estimated robot pose given as [x,y,theta]
%       poseCovIn: The estimated covariance matrix of the robot pose
%       matchResult: A (5xnoOfWorldLines) matrix whose columns are 
%       individual pairs of line matches. It is structured as follows:
%       matchResult = [ worldLine(1,1) , worldLine(1,2) ...  ]
%                     [ worldLine(2,1) , worldLine(2,2)      ]
%                     [ innovation1(1) , innovation2(1)      ]
%                     [ innovation1(2) , innovation2(2)      ]
%                     [ matchIndex1    , matchIndex2    ...  ]
%           Note that the worldLines are in the world coordinates!
%       
%       poseOut: The updated robot pose estimate
%       poseCovOut: The updated estimate of the robot pose covariance 
%       matrix 

    % Constants
    % The laser scanner pose in the robot frame is read globally(lsrRelpose)
    % The varAlpha and varR are the assumed variances of the parameters of
    % the extracted lines, they are also read globally
    global lsrRelPose varAlpha varR

    NrLines = size(matchResult,2)
    noise = zeros(2*NrLines,2*NrLines);
    NablaH = zeros(2*NrLines,3);
    vt = zeros(2*NrLines,1);
    
    j = 0;
    for i = 1:(2*NrLines)
        if(mod(i,2) == 1)
            j = j+1;
            noise(i,i) = varAlpha;
            noise(i+1,i+1) = varR;
            
            NablaHtemp = [0 0 -1; -cos(matchResult(1,j)) -sin(matchResult(1,j)) 0];
            NablaH(i,:) = NablaHtemp(1,:);
            NablaH(i+1,:) = NablaHtemp(2,:);
            
            vt(i) = matchResult(3,j);
            vt(i+1) = matchResult(4,j);
            
        end
    end
        
    SigmaIN = NablaH * poseCovIn * NablaH' + noise;
    
    Kt = poseCovIn * NablaH' * inv(SigmaIN);
    
    poseOut = poseIn + Kt*vt;
    poseCovOut = poseCovIn - Kt * SigmaIN * Kt';

end
