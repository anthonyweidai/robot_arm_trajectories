function dxdt = nlmpcModelKINOVA(x,u)

    % Copyright 2019 The MathWorks, Inc.

    numJoints = length(u);
    dxdt = zeros(size(x));
    dxdt(1:numJoints) = x(numJoints+1:end);
    dxdt(numJoints+1:end) = u;
end