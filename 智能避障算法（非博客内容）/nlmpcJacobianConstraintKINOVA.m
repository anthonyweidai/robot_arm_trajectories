function [G,Gmv,Ge] = nlmpcJacobianConstraintKINOVA(X,U,e,data, world, robot, collisionHelper)

    % Copyright 2019 The MathWorks, Inc.

    p = data.PredictionHorizon;
    numJoints = data.NumOfOutputs;
    numBodies = size(collisionHelper.RigidBodyCollisionArray,1)-1;
    % End-effector body missing
    if isempty(collisionHelper.RigidBodyCollisionArray{end,1})
        numBodies = numBodies - 1;
    end
    numObstacles = numel(world);

    % Initialize Jacobians
    G = zeros(p, numJoints*2, p*numBodies*numObstacles);
    Gmv = zeros(p, numJoints, p*numBodies*numObstacles);
    Ge = zeros(p*numBodies*numObstacles,1);
    
    iter = 1;
    for i=1:p
        collisionConfig = X(i+1,1:numJoints);
        collisionHelper.ExhaustiveChecking = true;
        [~, ~, ~, ~, ~, ~, ~, ~ ,allWntPts] = checkRobotWorldCollision(collisionHelper, collisionConfig, world);
        for j=1:numBodies
            for k=1:numObstacles
                if any((allWntPts(:,1,j,k)-allWntPts(:,2,j,k))~=0)
                    normal = (allWntPts(:,1,j,k)-allWntPts(:,2,j,k))/norm(allWntPts(:,1,j,k)-allWntPts(:,2,j,k));
                else
                    normal = [0;0;0];
                end
                bodyJacobian = geometricJacobian(robot,collisionConfig', robot.BodyNames{j});
                G(i, 1:numJoints,  iter)= -normal' * bodyJacobian(4:6,:);
                iter = iter + 1;                 
            end           
        end
        
    end

end