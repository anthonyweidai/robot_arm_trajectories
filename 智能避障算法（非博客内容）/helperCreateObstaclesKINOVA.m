% Potential static obstacles
% Copyright 2019 The MathWorks, Inc.

aObs = collisionSphere(0.08);  
aObs.Pose = trvec2tform([0.4 0.4 0.25]);   

bObs = collisionSphere(0.08);  
bObs.Pose = trvec2tform([0.32 0.3 0.39]);

cObs = collisionSphere(0.05);  
cObs.Pose = trvec2tform([0.3 0.3 0.4]);  

dObs = collisionBox(0.4,0.04,0.25);  
dObs.Pose = trvec2tform([0.2 0.4 0.15]); 

% Potential moving obstacles
tsObs = 0.1;
freqMotion = 1/45;

timeObs = 0:tsObs:30;
aMovObs = collisionSphere(0.14);
aPosesObs = repmat([0.4 0.35 0.35],length(timeObs),1);
aPosesObs(:,3) = 0.32 + 0.15 * sin( (2*pi*freqMotion)*timeObs' );

bMovObs = collisionSphere(0.14);
bPosesObs = repmat([0.4 0.5 0.15],length(timeObs),1);
bPosesObs(:,1) = 0.38 + 0.15 * sin( (2*pi*freqMotion)*timeObs' );

% Set obstacles
if isMovingObst
    tNow = 0;
    aPoseObsNow = interp1(timeObs,aPosesObs,tNow);
    aMovObs.Pose = trvec2tform(aPoseObsNow);
    bPoseObsNow = interp1(timeObs,bPosesObs,tNow);
    bMovObs.Pose = trvec2tform(bPoseObsNow);
    world = {aMovObs, bMovObs};
else
%      world = {aObs, bObs ,cObs};
     world = {dObs};
    % try adding more static obstacles aObs to dObs or create your own
end