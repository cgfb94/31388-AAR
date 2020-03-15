function [ matchResult ] = match( pose, poseCov, worldLines, laserLines )
% [matchResult] = MATCH(pose,poseCov,worldLines,laserLines)
%   This function matches the predicted lines to the extracted lines. The
%   criterion for a match is the mahalanobis distance between the (alpha,
%   r) parameters of the predicted and the extracted lines. The arguments
%   are:
%       pose: The estimated robot pose given as [x,y,theta]
%       poseCov: The estimated covariance matrix of the robot pose
%       worldLines: Known world lines in world coordinates, given as
%       [alpha;r] for each line. Number of rows = number of lines
%       laserLines: Lines extracted from the laser scan. Given as [alpha;r]
%       for each line. Number of rows = number of lines
%
%       matchResult: A (5xnoOfWorldLines) matrix whose columns are 
%       individual pairs of line matches. It is structured as follows:
%       matchResult = [ worldLine(1,1) , worldLine(1,2) ...  ]
%                     [ worldLine(2,1) , worldLine(2,2)      ]
%                     [ innovation1(1) , innovation2(1)      ]
%                     [ innovation1(2) , innovation2(2)      ]
%                     [ matchIndex1    , matchIndex2    ...  ]
%           Note that the worldLines are in the world coordinates!

    % The varAlpha and varR are the assumed variances of the parameters of
    % the extracted lines, they are read globally.
    global varAlpha varR
    
    NoM = 1;
    % matlab is lame
    display(worldLines);
    display(laserLines);
    
    distances = zeros(size(worldLines, 2), size(laserLines, 2));
    
    res = zeros(5, size(worldLines, 2));
    matchResult = zeros(5, size(worldLines, 2));

    sigmaR = diag([varAlpha, varR]);
    
    for i = 1:size(worldLines, 2)
        [projectedLine, lineCov] = projectToLaser(worldLines(:,i), pose, poseCov);
        display(projectedLine);
        H = [0 0 -1; -cos(worldLines(1,i)) -sin(worldLines(1,i)) 0];
        for j = 1:size(laserLines, 2)
            innovation = laserLines(:,j) - projectedLine';
            display(innovation);
            sigma_innovation = H*poseCov*H' + sigmaR;
            distances(i,j) = innovation'*inv(sigma_innovation)*innovation;
            if innovation'*inv(sigma_innovation)*innovation <= 4
                res(1,NoM) = worldLines(1,i);
                res(2,NoM) = worldLines(2,i);
                res(3,NoM) = innovation(1);
                res(4,NoM) = innovation(2);
                res(5,NoM) = j;
                NoM = NoM + 1;
            end
        end
    end
    display(distances);
    matchResult = res(:,1:NoM-1);
    display(matchResult);
    
end
