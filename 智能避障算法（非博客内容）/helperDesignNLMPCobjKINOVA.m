% Nonlinear Model Predictive Control (NLMPC)
% This function shows how to design nonlinear MPC controller for KINOVA. 
%
% Copyright 2019 The Mathworks, Inc. 

% Initialize nlmpc object (by modeling joints as double integrators).
nx = numJoints * 2; % [q,qDot]
ny = numJoints; % [q]
nu = numJoints; % [qDdot]
nlobj = nlmpc(nx,ny,nu);

% Set sample time, prediction horizon, and control horizon.
Ts = 1; % units in seconds
p = 5; 
c = 3;
nlobj.Ts = Ts; % sample time
nlobj.PredictionHorizon = p; 
nlobj.ControlHorizon = c; 

% Set constraints on States and MV.
stateMinValues = {-174.53;-2.2000;-174.53;-2.5656;-174.53;-2.0500;-174.53;...
    -0.8727;-0.8727;-0.8727;-0.8727;-0.8727;-0.8727;-0.8727};
stateMaxValues = {174.53;2.2000;174.53;2.5656;174.53;2.0500;174.53;...
    0.8727;0.8727;0.8727;0.8727;0.8727;0.8727;0.8727};
nlobj.States = struct('Min',stateMinValues,'Max',stateMaxValues);
nlobj.MV = struct('Min',{-1;-1;-1;-1;-10;-10;-10},'Max',{1;1;1;1;10;10;10});

% State functions and its Jacobian.
nlobj.Model.StateFcn = @(x,u) nlmpcModelKINOVA(x,u);  
nlobj.Jacobian.StateFcn = @(x,u) nlmpcJacobianModelKINOVA(x,u);

% Output function and its Jacobian.
nlobj.Model.OutputFcn = @(x,u) x(1:numJoints);
nlobj.Jacobian.OutputFcn = @(x,u) nlmpcJacobianOutputModelKINOVA(x,u);

% Cost function and its Jacobian. 
Qr = diag([3 3 3 0 0 0]); % running cost weight on desired end-effector pose [x, y, z, phi, theta, psi]
Qt = diag([5 5 5 1 1 0]); % terminal cost weight on desired end-effector pose [x, y, z, phi, theta, psi]
Qu = diag([1 1 1 1 1 1 1])/10; % input cost weight on joint accelerations qDdot
Qv = diag([1 1 1 1 1 1 1]); % terminal joint velocity cost weight on joint velocities qDot
nlobj.Optimization.CustomCostFcn = ...
    @(X,U,e,data) nlmpcCostFunctionKINOVA(X,U,e,data,poseFinal,robot,endEffector,Qr,Qt,Qu,Qv); 
nlobj.Optimization.ReplaceStandardCost = true;
nlobj.Jacobian.CustomCostFcn = ...
    @(X,U,e,data) nlmpcJacobianCostKINOVA(X,U,e,data,poseFinal,robot,endEffector,Qr,Qt,Qu,Qv);

% Inequality constraints and its Jacobian.
nlobj.Optimization.CustomIneqConFcn = ...
    @(X,U,e,data) nlmpcIneqConFunctionKINOVA(X,U,e,data,collisionHelper,safetyDistance,world);
nlobj.Jacobian.CustomIneqConFcn = ...
    @(X,U,e,data) nlmpcJacobianConstraintKINOVA(X,U,e,data,world,robot,collisionHelper);

% Set optimization solver options.
nlobj.Optimization.SolverOptions.FunctionTolerance = 0.001;
nlobj.Optimization.SolverOptions.StepTolerance = 0.001;
nlobj.Optimization.SolverOptions.MaxIter = 5;