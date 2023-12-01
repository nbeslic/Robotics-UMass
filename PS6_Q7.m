%% Problem Set 6
% Question 7: 2D path tracing with inverse kinematics

%% SET UP 2D RPR ROBOT
% Initialize a robot 
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',4);

% Pick arm lengths
L1 = 1;
L2 = 1;
L3 = 1;

% Create body 1 and add joint 1 - revolute
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

% Create body 2 and add joint 2 - prismatic
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','prismatic');
joint.PositionLimits = [-2,2];
setFixedTransform(joint, trvec2tform([L1,0,0]));        
joint.JointAxis = [1 0 0];
body.Joint = joint;
addBody(robot, body, 'link1');

% Create body 3 and add joint 3 - revolute
body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint, trvec2tform([L2,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link2');

% Adding an endeffector called tool
body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L3, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link3');

%% DEFINE TRAJECTORY
% Define a circle to be traced over the course of 10 seconds. 
% This circle is in the xy plane with a radius of 0.15.

t = (0:0.2:10)'; % Time
count = length(t);
center = [1.5 0.5 0];
radius = 0.15;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

%% INVERSE KINEMATICS
% Use an inverseKinematics object to find a solution of robotic 
% configurations that achieve the given end-effector positions along the 
% trajectory.
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);    % preallocate configuration solution 

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];       % only x,y cartesian weights are important for this workflow
endEffector = 'tool';

qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

%% Animate the solution 
figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
% axis([-0.1 0.7 -0.3 0.5])
axis([-1 2 -1 1])

framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end