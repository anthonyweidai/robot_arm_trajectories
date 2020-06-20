function cineq = nlmpcIneqConFunctionKINOVA(X,U,e,data, collisionHelper, safetyDistance, world)

    % Copyright 2019 The MathWorks, Inc.

    p = data.PredictionHorizon;
    numJoints = data.NumOfOutputs;
    numBodies = size(collisionHelper.RigidBodyCollisionArray,1)-1;
    % End-effector body missing
    if isempty(collisionHelper.RigidBodyCollisionArray{end,1})
        numBodies = numBodies - 1;
    end
    numObstacles = numel(world);
    allDistances = zeros(p*numBodies*numObstacles,1);
    for i =1:p
        collisionConfig = X(i+1,1:numJoints);
        collisionHelper.ExhaustiveChecking = true;
        [~, ~, ~, ~, ~, ~, distances, ~,~] = checkRobotWorldCollision(collisionHelper, collisionConfig, world);
        allDistances((1+(i-1)*numBodies*numObstacles):numBodies*numObstacles*i,1) = distances;
    end
    cineq = -allDistances + safetyDistance;
end