clc
close all;
%% Set up robot parameters
syms l0 l1 l2;
syms r0 r1 r2;
syms m1 m2 m3;
syms Ix1 Ix2 Ix3 Iy1 Iy2 Iy3 Iz1 Iz2 Iz3;

params = struct(...
    'l0',                0.3, ...
    'l1',                0.3,...
    'l2',                0.3,...
    'r0',                0.15, ...
    'r1',                0.15, ...
    'r2',                0.15,...
    'm1',                0.8, ...
    'm2',                0.8, ...
    'm3',                0.8, ...
    'I1',                Calc_Inertia(0.8,0.3,0.05),...% Calc_I from m=0.8 l=0.3 r=0.05
    'I2',                Calc_Inertia(0.8,0.3,0.05),...
    'I3',                Calc_Inertia(0.8,0.3,0.05),...
    'g',                 9.8);

%% Set the simualation parameters
time_initial = 0; 
time_final = 3;
time_step = 0.01; % sec
time_vec = time_initial:time_step:time_final;
max_iter = length(time_vec);

%% Create the state vector
state = zeros(6,1);
state(1)  = 0;    %theta1
state(2)  = 0;  %theta2
state(3)  = 0;    %theta3
state(4)  = 0;    %theta1_dot
state(5)  = 0;    %theta2_dot
state(6)  = 0;    %theta3_dot
theta1_des = 0;
theta2_des = -pi/2; % the desired angle which is point skyward.
theta3_des = 0;
%% Loop through the timesteps and update
history = zeros(6,max_iter-1);
for iter = 1:max_iter-1
    
    theta1 = state(1);
    theta2 = state(2);
    theta3 = state(3);
    
    theta1_error = theta1_des - theta1;
    theta2_error = theta2_des - theta2;
    theta3_error = theta3_des - theta3;
    
    tau1 = 10*theta1_error;
    tau2 = 10*theta2_error;
    tau3 = 10*theta3_error;
    tau = [tau1;tau2;tau3];
    timeint = time_vec(iter:iter+1);
    

    [tsave, xsave] = ode45(@(t,s) dynamic_for_q4(params, state, tau,M,C,N), timeint, state);
    state = xsave(end, :)';
    history(:,iter) = state;
end

%% plot
subplot(3,2, 1);
plot(1:max_iter-1,history(2,:));
title("alpha")
subplot(3,2, 3);
plot(1:max_iter-1,history(5,:));
title("alpha-dot")
subplot(3,2, 2);
plot(1:max_iter-1,history(3,:));
title("x")
subplot(3,2, 4);
plot(1:max_iter-1,history(6,:));
title("x-dot")