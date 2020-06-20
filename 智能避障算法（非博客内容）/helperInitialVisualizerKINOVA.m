% Copyright 2019 The MathWorks, Inc.

% Initial visualizer
positions = x0(1:numJoints)';

figure('Position', [375 446 641 480]);
ax1 = show(robot, positions(:,1),'PreservePlot', false, 'Frames', 'off');
view(150,29)
hold on
axis([-0.8 0.8 -0.6 0.7 -0.2 0.7]);
plot3(poseFinal(1), poseFinal(2), poseFinal(3),'r.','MarkerSize',20)

% Visualize collision world
if isMovingObst
    poseObsTemp = [0 0 0]; % frame reference
    aMovObs.Pose = trvec2tform(poseObsTemp);
    bMovObs.Pose = trvec2tform(poseObsTemp);
    [~,pObject1] = show(aMovObs);
    pObject1.LineStyle = 'none'; 
    [~,pObject2] = show(bMovObs);
    pObject2.LineStyle = 'none'; 
    h1 = hgtransform;
    pObject1.Parent = h1;
    h2 = hgtransform;
    pObject2.Parent = h2;
    tNow = 0;
    aPoseObsNow = interp1(timeObs,aPosesObs,tNow);
    aMovObs.Pose = trvec2tform(aPoseObsNow);
    h1.Matrix = aMovObs.Pose;
    bPoseObsNow = interp1(timeObs,bPosesObs,tNow);
    bMovObs.Pose = trvec2tform(bPoseObsNow);
    h2.Matrix = bMovObs.Pose;
else
    for i=1:numel(world)
        [~,pObj] = show(world{i});
        pObj.LineStyle = 'none';
    end
end



