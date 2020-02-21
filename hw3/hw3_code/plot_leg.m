function [outputArg1,outputArg2] = plot_leg(gesture,theta,l1,l2)
%gestrue = [xl,xr,groundl,groundr,z]
%theta = [theta_la,theta_lk,theta_ra,theta_rk];

xl = gesture(1);
xr = gesture(2);
groundl = gesture(3);
groundr = gesture(4);
xl = -xl;
xr = -xr;
theta_la = theta(1);
theta_lk = theta(2) + theta_la;
theta_ra = theta(3);
theta_rk = theta(4) + theta_ra;

ankle_l = [xl,groundl];
ankle_r = [xr,groundr];
knee_l = ankle_l + [l1*sin(theta_la),l1*cos(theta_la)];
knee_r = ankle_r + [l1*sin(theta_ra),l1*cos(theta_ra)];
hip = knee_l + [l2*sin(theta_lk),l2*cos(theta_lk)];
left = [ankle_l;knee_l;hip];
right = [ankle_r;knee_r;hip];
plot(left(:,1),left(:,2))
hold on
plot(right(:,1),right(:,2),'r')
end

