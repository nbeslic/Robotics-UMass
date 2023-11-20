% NBeslic

close all;
clear all;
clc;

%[cspace] = getCSpace([0 .1; 0 .2], 1, 1); 
[cspace] = getCSpace([-1.2 .5; -.1 .5; -.1 1; -1.2 1], [1 1], 1); 
    
function [cspace] = getCSpace(obstacle, L, k)

% generate cspace grid
theta1 = 0:k:360;
theta2 = 0:k:360;

N = length(theta1);
M = length(theta2);


cspace = zeros(N,M);

for i = 1:N
    for j = 1:M
        cspace(j,i) = checkCollision([theta1(i) theta2(j)], L, obstacle);
    end
end

    imagesc([0 360], [0], cspace);
    set(gca, 'YDir', 'normal');
end


function [collision] = checkCollision(theta, L, obstacle)
   
    obstacle = [obstacle(end, :); obstacle]; 
  
   % first link in the arm segment
   r1 = [[0 0]; L(1)*[cosd(theta(1)) sind(theta(1))]];

   % second link in the arm segment
   r2 = [L(1)*[cosd(theta(1)) sind(theta(1))]; [L(1)*cosd(theta(1))+L(2)*cosd(theta(1)+theta(2)) L(1)*sind(theta(1))+L(2)*sind(theta(1)+theta(2))]];
   
   for j = 1:length(obstacle(:,1))-1
       collision1 = checkIntersection(r1, obstacle(j:j+1,:));
       collision2 = checkIntersection(r2, obstacle(j:j+1,:));
       
       
       collision = collision1 + collision2;
       if collision == 2
           collision = 1;
       end

       if collision == 1
           return;
       end
   end
end



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
