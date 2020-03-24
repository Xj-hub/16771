function [state_dot] = dynamic_for_q4(params, state,tau, M,C,N)
%Input state
%Output state_dot
% state(1)  = 0;    %theta1
% state(2)  = 0;  %theta2
% state(3)  = 0;    %theta3
% state(4)  = 0;    %theta1_dot
% state(5)  = 0;    %theta2_dot
% state(6)  = 0;    %theta3_dot
% l0=params.l0;
% l1=params.l1;
% l2=params.l2;
% r0=params.r0;
% r1=params.r1;
% r2=params.r2;
% m1=params.m1;
% m2=params.m2;
% m3=params.m3;
% I1=params.I1;
% I2=params.I2;
% I3=params.I3;
% g = params.g;
% Ix1 = I1(1);
% Iy1 = I1(2);
% Iz1 = I1(3);
% Ix2 = I2(1);
% Iy2 = I2(2);
% Iz2 = I2(3);
% Ix3 = I3(1);
% Iy3 = I3(2);
% Iz3 = I3(3);

%************  DYNAMICS ************************
%syms t1 t2 t3 dt1 dt2 dt3
theta1 = state(1);
theta2 = state(2);
theta3 = state(3);
dtheta1 = state(4);
dtheta2 = state(5);
dtheta3 = state(6);

state_dot = zeros(6,1);
state_dot(1:3) = state(4:6);

% M_value = subs(M,[t1,t2,t3],[theta1,theta2,theta3]);
% M_value = double(M_value);
% C_value = subs(C,[t1,t2,t3,dt1,dt2,dt3],[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3]);
% C_value = double(C_value);
% N_value = subs(N,[t1,t2,t3],[theta1,theta2,theta3]);
% N_value = double(N_value);
state_dot(4:6) = inv(M)*(tau - N - C*state(4:6));


    
end