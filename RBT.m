% one
rbtree=rigidBodyTree;
body1=rigidBody('b1');

jnt1=rigidBodyJoint('jnt1','revolute');
body1.Joint=jnt1;

basename=rbtree.BaseName;
addBody(rbtree,body1,basename);

showdetails(rbtree)

% two
% first column is the distance between z axis(d), first column is angle
% rotate with x axis(\alpha); third is the distance between x axis (a)
% dhparams = [0   	pi/2	0   	0;
%             0.4318	0       0       0;
%             0.0203	-pi/2	0.15005	0;
%             0   	pi/2	0.4318	0;
%             0       -pi/2	0   	0;
%             0       0       0       0];
% robot=rigidBodyTree;
% 
% body1 = rigidBody('body1');
% jnt1 = rigidBodyJoint('jnt1','revolute');
% 
% setFixedTransform(jnt1,dhparams(1,:),'dh');
% body1.Joint = jnt1;
% 
% addBody(robot,body1,'base')
% 
% body2 = rigidBody('body2');
% jnt2 = rigidBodyJoint('jnt2','revolute');
% body3 = rigidBody('body3');
% jnt3 = rigidBodyJoint('jnt3','revolute');
% body4 = rigidBody('body4');
% jnt4 = rigidBodyJoint('jnt4','revolute');
% body5 = rigidBody('body5');
% jnt5 = rigidBodyJoint('jnt5','revolute');
% body6 = rigidBody('body6');
% jnt6 = rigidBodyJoint('jnt6','revolute');
% 
% setFixedTransform(jnt2,dhparams(2,:),'dh');
% setFixedTransform(jnt3,dhparams(3,:),'dh');
% setFixedTransform(jnt4,dhparams(4,:),'dh');
% setFixedTransform(jnt5,dhparams(5,:),'dh');
% setFixedTransform(jnt6,dhparams(6,:),'dh');
% 
% body2.Joint = jnt2;
% body3.Joint = jnt3;
% body4.Joint = jnt4;
% body5.Joint = jnt5;
% body6.Joint = jnt6;
% 
% addBody(robot,body2,'body1')
% addBody(robot,body3,'body2')
% addBody(robot,body4,'body3')
% addBody(robot,body5,'body4')
% addBody(robot,body6,'body5')
% 
% showdetails(robot)
% 
% show(robot);
% axis([-0.5,0.5,-0.5,0.5,-0.5,0.5])
% axis on

% three
% load exampleRobots.mat
% % showdetails(puma1)
% 
% body3=getBody(puma1,'L3');
% childBody=body3.Children{1};
% 
% body3Copy = copy(body3);
% newJoint=rigidBodyJoint('prismatic');
% replaceJoint(puma1,'L3',newJoint);
% 
% showdetails(puma1)

% % four
% robot=rigidBodyTree('DataFormat','row');
% body1 = rigidBody('body1');
% body2 = rigidBody('body2');
% 
% joint1 = rigidBodyJoint('joint1','revolute');
% joint2 = rigidBodyJoint('joint2');
% setFixedTransform(joint2,trvec2tform([1 0 0]))
% body1.Joint = joint1;
% body2.Joint = joint2;
% 
% body1.Mass = 2;
% body1.CenterOfMass = [0.5 0 0];
% body1.Inertia = [0.167 0.001 0.167 0 0 0];
% 
% body2.Mass = 1;
% body2.CenterOfMass = [0 0 0];
% body2.Inertia = 0.0001*[4 4 4 0 0 0];
% 
% addBody(robot,body1,'base');
% addBody(robot,body2,'body1');
% 
% comPos = centerOfMass(robot);
% 
% show(robot);
% hold on
% plot(comPos(1),comPos(2),'or')
% view(2)
% 
% body2.Mass = 20;
% replaceBody(robot,'body2',body2)
% 
% comPos2 = centerOfMass(robot);
% plot(comPos2(1),comPos2(2),'*g')
% hold off

% five
% load exampleRobots.mat lbr
% lbr.DataFormat = 'row';
% lbr.Gravity = [0 0 -9.81];% z axis direction
% q = homeConfiguration(lbr);
% wrench=[0 0 0.5 0 0 0.3];
% fext=externalForce(lbr,'tool0',wrench,q);
% qddot = forwardDynamics(lbr,q,[],[],fext);

% six
% robot = importrobot('iiwa14.urdf');
% show(robot);