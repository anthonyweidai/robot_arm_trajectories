function [C, D] = nlmpcJacobianOutputModelKINOVA(x,u)

    % Copyright 2019 The MathWorks, Inc.

    numJoints = length(u);
    C = zeros(numJoints, numJoints * 2);
    C(1:numJoints, 1:numJoints) = eye(numJoints);
    D = zeros(numJoints, numJoints);
end