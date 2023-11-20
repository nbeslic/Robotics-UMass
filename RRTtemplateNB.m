close all;
clear all;
clc;
%% Rapidly exploring Random Tree's Sampling Method
% "The RRT algorithm searches for a collision-free motion from an initial 
% state Xstart to a goal set Xgoal. It is applied to kinematic problems, 
% where the state x is simply the configuration q, as well as to dynamic 
% problems, where the state includes the velocity."

%% create two link revolute robot.

robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);
L = [1 1];

body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

body = rigidBody('link2');
joint = rigidBodyJoint('joint2', 'revolute');
setFixedTransform(joint,trvec2tform([L(1) 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

body = rigidBody('endeffector');
joint = rigidBodyJoint('endpoint','fixed');
setFixedTransform(joint, trvec2tform([L(2), 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'endeffector'; 


%% create inital and final locations in workspace.

xinitial = [-.5 1.5 0];
xgoal = [-1 -1 0];
qinitial = ik(endEffector,trvec2tform(xinitial),weights,homeConfiguration(robot));

%% create obstacles in planar workspace.

obstacles{1} = [-1.2 .5; -.1 .5; -.1 1; -1.2 1];
obstacles{2} = [-.2 -1.5; -.1 -1.5; -.1 -2; -.2 -2];

%% plot robot and obstacles.

figure(1);
hold on;
% plot robot in initial position
plotRobotInConfig(robot, qinitial, gca)

hold on;
% plot obstacles
for i = 1:length(obstacles)
    line([obstacles{i}(:,1); obstacles{i}(1,1)] , [obstacles{i}(:,2); obstacles{i}(1,2)]);
    % obstacle = [obstacle(end, :); obstacle]; 
end

% arrange view
set(gca, 'Projection', 'orthographic', 'XLim', [-2.1 2.1], 'YLim', [-2.1 2.1], 'View', [0 90]);

%% create RRT.

maxT = 200; % maximum tree size
d = .2; % step size
reachGoal = false;

% initialize search tree T with xstart
G = graph;
NodeProps = table(xinitial(1), xinitial(2), qinitial(1), qinitial(2), 'VariableNames', {'x' 'y' 'q1' 'q2'}); % Create a node with variables x, y, q1, q2
G = addnode(G, NodeProps);
% 2: while T is less than the maximum tree size do
while (numnodes(G) < maxT && reachGoal == false)
    % 3: xsamp get sample from X
    xsamp = 4*rand([1,2])-2;        % generate random (x,y) pair from -2 to 2

    % 4: xnearest ? nearest node in T to xsamp
    distance = zeros(numnodes(G));        % allocate space to collect distances between nodes
    for i = 1:numnodes(G)
        distance(i) = min(sqrt((G.Nodes.x(i)-xsamp(1)).^2 + (G.Nodes.y(i)-xsamp(2)).^2));   
    end
    [distances, index_nearest] = sort(distance);     % sort distances in ascending order
    ind = index_nearest(1);
    xnearest = [G.Nodes.x(ind) G.Nodes.y(ind)];      % get xnearest coordinates 


    % 5: employ a straight line path to find a motion from xnearest to xnew in
    % the direction of xsamp at some interval v
    direction = (xsamp-xnearest)/norm(xsamp-xnearest(1:2));         % define unit vector in direction of xsamp from xnearest
    xnew = xnearest + d * direction;       % create xnew in the diretion of xsamp from xnearest

    % 6: check if motion is collision-free 
    xnew = [xnew, 0];       % make xnew a 3x1 with a 0 in the z coordinate
    qnew = ik(endEffector,trvec2tform(xnew),weights,homeConfiguration(robot));        % get q values from xnew

    for i =1:length(obstacles)
        collisions = checkCollision(qnew, L, obstacles{i});        % check for collisions 
        if collisions == 0
            xtoadd = xnew;
            qtoadd = qnew;
            % 7: if the motion is collision-free then add xnew to T with an edge from xnearest to xnew 
            NodeProps = table(xtoadd(1), xtoadd(2), qtoadd(1), qtoadd(2), 'VariableNames', {'x' 'y' 'q1' 'q2'});
            G = addnode(G, NodeProps);
            G = addedge(G,ind,numnodes(G));
        elseif collisions ~= 0
            break
        end
    end

    % 8: if xnew is within distance epsilon of xgoal, add xgoal to the
    % graph and return
    epsilon = 0.2;
    if (norm(xtoadd-xgoal) < epsilon)
        qgoal = ik(endEffector,trvec2tform(xgoal),weights,homeConfiguration(robot));
        NodeProps = table(xgoal(1), xgoal(2), qgoal(1), qgoal(2), 'VariableNames', {'x' 'y' 'q1' 'q2'});
        G = addnode(G, NodeProps);
        G = addedge(G,ind,numnodes(G));
        reachGoal = true;
    end
end

figure(1); 
% plot nodes
plot(G.Nodes.x, G.Nodes.y, 'k.')
% plot edges
plot([G.Nodes.x(G.Edges.EndNodes(:,1)) G.Nodes.x(G.Edges.EndNodes(:,2))]', ...
     [G.Nodes.y(G.Edges.EndNodes(:,1)) G.Nodes.y(G.Edges.EndNodes(:,2))]', 'k-');

% use the command shortestpath() to get the shortest path if graph reaches final configuration 
shortpath = shortestpath(G,1,numnodes(G));

% plot shortest path if graph reaches final configuration
plot(G.Nodes.x(shortpath), G.Nodes.y(shortpath), 'r-', 'LineWidth', 2);



%% function to plot robot in given configuration.
function plotRobotInConfig(robot, q, axes)
    show(robot, q, 'PreservePlot', true, 'Parent', axes);
    set(gca,'View', [0 90])
end

%% function that checks for collisions
function [collision] = checkCollision(theta, L, obstacle)
   
    obstacle = [obstacle(end, :); obstacle]; 
  
   % first link in the arm segment
   r1 = [[0 0]; L(1)*[cos(theta(1)) sin(theta(1))]];

   % second link in the arm segment
   r2 = [L(1)*[cos(theta(1)) sin(theta(1))]; [L(1)*cos(theta(1))+L(2)*cos(theta(1)+theta(2)) L(1)*sin(theta(1))+L(2)*sin(theta(1)+theta(2))]];
   
   for j = 1:length(obstacle(:,1))-1
       collision1 = checkIntersection(r1, obstacle(j:j+1,:));
       collision2 = checkIntersection(r2, obstacle(j:j+1,:));
       
       collision = collision1 + collision2;

       if collision >= 1
           return;
       end
   end
end

%% function that checks for intersection of two lines
function [intersection] = checkIntersection(p1, p2)

% see this site for an explanation: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
    r = p1(2,:)-p1(1,:);
    s = p2(2,:)-p2(1,:);

    uNum = cross([p2(1,:)-p1(1,:) 0], [r 0]);
    uDem = cross([r 0], [s 0]);
    
    if all([uNum uDem] == 0) 
        if any([sum(p1(1,:)-p2(1,:)) sum(p1(1,:)-p2(2,:)) sum(p1(2,:)-p2(1,:)) sum(p1(2,:)-p2(2,:))] == 0)
            intersection = 1;
        else
            vx = [p1(1,1)-p2(1,1) p1(1,1)-p2(2,1) p1(2,1)-p2(1,1) p1(2,1)-p2(2,1)];
            vy = [p1(1,2)-p2(1,2) p1(1,2)-p2(2,2) p1(2,2)-p2(1,2) p1(2,2)-p2(2,2)];
            intersection = (any(diff(sign(vx))) || any(diff(sign(vy))));
        end
    else
        if uDem == 0
            intersection = 0;
        else
            u = uNum/uDem;
            t = cross([p2(1,:)-p1(1,:) 0], [s 0])/uDem;
            intersection = ((t >= 0) && (t <= 1) && (u >= 0) && (u <= 1));
        end
    end
        
end


