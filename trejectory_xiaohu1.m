clc,clear,close all
%加载机器人模型
robot = loadrobot('kinovaGen3','DataFormat','row','Gravity',[0 0 -9.81]);
currentRobotJConfig = homeConfiguration(robot);%机器人关节配置

% 7 joints
numJoints = numel(currentRobotJConfig);%机器人关节数量
endEffector = 'EndEffector_Link';

timeStep = 0.1; % 步长
toolSpeed = 0.1; % 速度

jointInit = currentRobotJConfig;
taskInit = getTransform(robot,jointInit,endEffector);%初始
% axang2tform homogeneous transform齐次变换
% 绕y轴旋转180
taskFinal = trvec2tform([1.3,0.4,0.9])*axang2tform([0 1 0 pi/2]);%末端
% taskFinal = trvec2tform([1.3,0.4,0.9]);

%Task-Space Trajectory
distance= norm(tform2trvec(taskInit)-tform2trvec(taskFinal));%距离

% set time
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];
% Algorithm one
% Task-Space
% PID控制器
tsMotionModel = taskSpaceMotionModel('RigidBodyTree',robot,'EndEffectorName','EndEffector_Link');
tsMotionModel.Kp(1:3,1:3) = 0;%
tsMotionModel.Kd(1:3,1:3) = 0;

q0 = currentRobotJConfig; %初始状态
qd0 = zeros(size(q0));
%模拟机器人运动
[tTask,stateTask] = ode15s(@(t,state) exampleHelperTimeBasedTaskInputs(tsMotionModel,timeInterval,taskInit,taskFinal,t,state),timeInterval,[q0; qd0]);

%初始状态
show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);
%任务空间运动
for i=1:length(trajTimes)
    % Current time
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tTask,stateTask(:,1:numJoints),tNow);% Algorithm one
    poseNow = getTransform(robot,configNow,endEffector);
    show(robot,configNow,'PreservePlot',false,'Frames','off');
    plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20)
    drawnow;
end


figure(2);
grid on;
plot(tTask,stateTask(:,1:numJoints));
hold all;
plot(tTask(1:numJoints),stateTask(1:numJoints),'--');
title('Joint Position vs Reference ');
xlabel('Time (s)')
ylabel('Position (rad)');

