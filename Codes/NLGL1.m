function [x,y,theta] = NLGL1(xc,yc,x1,y1,x2,y2,theta1,v,dt,vwind,awind)
L1=2;
Rmin=2;
uavpos = [xc yc theta1];
W0 = [x1 y1];
W1 = [x2 y2];
theta = atan2(W1(2)-W0(2), W1(1)-W0(1));  

  % effect of wind
    uwd = [v*cos(uavpos(3))+vwind*cos(awind) v*sin(uavpos(3))+vwind*sin(awind)];
    kiang = atan2(uwd(2), uwd(1));  
    vg = sqrt(uwd(1)^2+uwd(2)^2);
    
    R = norm(W0-uavpos(1:2)); % distance between UAV and W1
    theta_u = ang_wrap(atan2(uavpos(2)-W0(2), uavpos(1)-W0(1))); % angle between UAV and W0 waypoint
    d = R * sin(ang_wrap(theta_u- theta));% cross track error
    alpha = atan2(abs(d), sqrt(abs(L1^2-d^2)));
    if d > 0
        eta = -ang_wrap(alpha + kiang-theta);
    else
        eta =  ang_wrap(alpha - kiang+theta);
    end
    u = 2*(vg^2/L1)*sin(eta);

uavpos(1) = uavpos(1) + v*cos(uavpos(3))*dt+ vwind*cos(awind)*dt; % update the new uav x position
    uavpos(2) = uavpos(2) + v*sin(uavpos(3))*dt+ vwind*sin(awind)*dt; % update the new uav y positon
    
    % limit the accelaration to the minimum turning radius
    if abs(u) > vg^2/Rmin
        if u > 0
            u = vg^2/Rmin;
        else
            u = - vg^2/Rmin;
        end
    end    
    uavpos(3) = uavpos(3)+dt*u/vg;% update the heading direction     
 x = uavpos(1);
 y= uavpos(2);
 theta = uavpos(3);
 
 
%     
%     
% t = atan2((yt-yc),(xt-xc));
% n = t-theta1;
% u = (2*(v^2)*sin(n))/L;
% if(u>0)
%     u=min(u,100);
% else
%     u=max(u,-100);
% end    
% x = xc+((v*cos(theta1)+vw*cos(thetaw))*dt);
% y = yc+((v*sin(theta1)+vw*sin(thetaw))*dt);
% vg = sqrt((v*cos(theta1)+vw*cos(thetaw))^2+(v*sin(theta1)+vw*sin(thetaw))^2);
% theta = theta1+(u*dt/vg);





