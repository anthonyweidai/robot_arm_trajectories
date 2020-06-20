% Copyright 2019 The MathWorks, Inc.

aPoseObsNow = interp1(timeObs,aPosesObs,time);
aMovObs.Pose = trvec2tform(aPoseObsNow);
bPoseObsNow = interp1(timeObs,bPosesObs,time);
bMovObs.Pose = trvec2tform(bPoseObsNow);
world = {aMovObs, bMovObs};
% Update constraint functions
nlobj.Optimization.CustomIneqConFcn = @(X,U,e,data) nlmpcIneqConFunctionKINOVA(X,U,e,data, collisionHelper , safetyDistance, world);
nlobj.Jacobian.CustomIneqConFcn = @(X,U,e,data) nlmpcJacobianConstraintKINOVA(X,U,e,data, world, robot, collisionHelper);
