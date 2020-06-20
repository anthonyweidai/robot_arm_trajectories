clc, clear, close all
robot = loadrobot('kinovaGen3', 'DataFormat', 'column');
% show(robot);
numJoints = numel(homeConfiguration(robot));%关节数量
endEffector = "EndEffector_Link"; %末端执行器

% 初始位子
taskInit = trvec2tform([[0.4 0 0.2]])*axang2tform([1 1 0 pi/2]);

% 运动学逆解
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];
currentRobotJConfig = ik(endEffector, taskInit, weights, robot.homeConfiguration);
currentRobotJConfig = wrapToPi(currentRobotJConfig);

% 末端位姿
taskFinal = trvec2tform([0.2 0.6 0.27])*axang2tform([1 1 0 pi]);  
anglesFinal = rotm2eul(taskFinal(1:3,1:3),'XYZ');
poseFinal = [taskFinal(1:3,4);anglesFinal']; % 6x1 vector for final pose: [x, y, z, phi, theta, psi]
%初始化网格碰撞
collisionHelper = helperManipCollsionsKINOVA(robot);
collisionHelper.ExhaustiveChecking = true;
% 
isMovingObst = false;
helperCreateObstaclesKINOVA;
% 
x0 = [currentRobotJConfig', zeros(1,numJoints)];
helperInitialVisualizerKINOVA;
% % 
safetyDistance = 0.01; %设置安全距离

helperDesignNLMPCobjKINOVA;

%轨迹规划
maxIters = 150;
u0 = zeros(1,numJoints);
mv = u0;
time = 0;
goalReached = false;


positions = zeros(numJoints,maxIters);
positions(:,1) = x0(1:numJoints)';
velocities = zeros(numJoints,maxIters);
velocities(:,1) = x0(numJoints+1:end)';
accelerations = zeros(numJoints,maxIters);
accelerations(:,1) = u0';
timestamp = zeros(1,maxIters);
timestamp(:,1) = time;



options = nlmpcmoveopt;
for timestep=1:maxIters
    disp(['Calculating control at timestep ', num2str(timestep)]);
    % Optimize next trajectory point 
    [mv,options,info] = nlmpcmove(nlobj,x0,mv,[],[], options);
    if info.ExitFlag < 0
        disp('Failed to compute a feasible trajectory. Aborting...')
        break;
    end
    % Update states and time for next iteration
    x0 = info.Xopt(2,:);
    time = time + nlobj.Ts;
    % Store trajectory points
    positions(:,timestep+1) = x0(1:numJoints)';
    velocities(:,timestep+1) = x0(numJoints+1:end)';
    accelerations(:,timestep+1) = info.MVopt(2,:)';
    timestamp(timestep+1) = time;
    % Check if goal is achieved 
    helperCheckGoalReachedKINOVA;
    if goalReached
        break;
    end
    % Update obstacle pose if it is moving
    if isMovingObst
        helperUpdateMovingObstaclesKINOVA;
    end
end

tFinal = timestep+1;
positions = positions(:,1:tFinal);
velocities = velocities(:,1:tFinal);
accelerations = accelerations(:,1:tFinal);
timestamp = timestamp(:,1:tFinal);
visTimeStep = 0.1;

motionModel = jointSpaceMotionModel('RigidBodyTree', robot);

% Control robot to target trajectory points in simulation using low-fidelity model
initState = [positions(:,1);velocities(:,1)];
targetStates = [positions;velocities;accelerations]';    
[t,robotStates] = ode15s(@(t,state) helperTimeBasedStateInputsKINOVA(motionModel, timestamp, targetStates, t, state), [timestamp(1):visTimeStep:timestamp(end)], initState);

helperFinalVisualizerKINOVA;
