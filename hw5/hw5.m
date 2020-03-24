clc
clear all;
syms l0 l1 l2;
syms r0 r1 r2;
%% calculate twist and initial g for center of mass
w1 = [0 0 1]';
w2 = [-1 0 0]';
w3 = [-1 0 0]';
q1 = [0 0 l0]';
q2 = [0 0 l0]';
q3 = [0 l1 l0]';
tw1 = [cross(q1, w1);w1];
tw2 = [cross(q2, w2);w2];
tw3 = [cross(q3, w3);w3];

twList = [tw1,tw2,tw3];

g_sl1 = [eye(3) [0;0;r0];0 0 0 1];
g_sl2 = [eye(3) [0;r1;l0];0 0 0 1];
g_sl3 = [eye(3) [0;l1+r2;l0];0 0 0 1];

%% calculate body jacobian
syms t1 t2 t3;

J_sl1_b = simplify(jacobianBody([Adjoint(TransInv(g_sl1))*tw1,...
    zeros(6,1),zeros(6,1)],[t1,t2,t3]));

J_sl2_b = simplify(jacobianBody([Adjoint(TransInv(g_sl2))*tw1,...
    Adjoint(TransInv(g_sl2))*tw2,zeros(6,1)],[t1,t2,t3]));

J_sl3_b = simplify(jacobianBody([Adjoint(TransInv(g_sl3))*tw1,...
    Adjoint(TransInv(g_sl3))*tw2,Adjoint(TransInv(g_sl3))*tw3],[t1,t2,t3]));

%% calculate mass matrix
syms m1 m2 m3;
syms Ix1 Ix2 Ix3 Iy1 Iy2 Iy3 Iz1 Iz2 Iz3;

M1 = [eye(3)*m1,zeros(3);zeros(3),diag([Ix1,Iy1,Iz1])];
M2 = [eye(3)*m2,zeros(3);zeros(3),diag([Ix2,Iy2,Iz2])];
M3 = [eye(3)*m3,zeros(3);zeros(3),diag([Ix3,Iy3,Iz3])];

M = simplify(J_sl1_b.'*M1*J_sl1_b + J_sl2_b.'*M2*J_sl2_b + J_sl3_b.'*M3*J_sl3_b)
M11 = M(1,1)

%% calculate Coriolis matrix 
C = sym(zeros(3));
syms dt1 dt2 dt3;
DT = [dt1; dt2; dt3];
T = [t1; t2; t3];
for i = 1:3
    for j= 1:3
        for k = 1:3
            C(i,j) = C(i,j) +  ...
                1/2 * (diff(M(i,j),T(k)) + diff(M(i,k),T(j)) - diff(M(k,j),T(i))) * DT(k);
        end
    end
end
C = simplify(C)
C21 = C(2,1)

%% calculate N matrix

[R1,h1] = TransToRp(twistExp(tw1, t1)*g_sl1);
h1 = simplify(h1(3));
[R2,h2] = TransToRp(twistExp(tw1, t1)*twistExp(tw2, t2)*g_sl2);
h2 = simplify(h2(3));
[R3,h3] = TransToRp(twistExp(tw1, t1)*twistExp(tw2, t2)*twistExp(tw3, t3)*g_sl3);
h3 = simplify(h3(3));
syms g;
V = m1 * g * h1 + m2 * g * h2 + m3 * g * h3;
N = sym(zeros(3,1));
for k = 1:3
    N(k) = simplify(diff(V,T(k)));
end
N = simplify(N)
N3 = N(3)

%% question_4
% Set up robot parameters
params = struct(...
    'l0',                0.3, ...
    'l1',                0.3,...
    'l2',                0.3,...
    'r0',                0.15, ...
    'r1',                0.15, ...
    'r2',                0.15,...
    'm1',                1.8, ...
    'm2',                1.8, ...
    'm3',                1.8, ...
    'I1',                Calc_Inertia(0.8,0.3,0.05),...% Calc_I from m=0.8 l=0.3 r=0.05
    'I2',                Calc_Inertia(0.8,0.3,0.05),...
    'I3',                Calc_Inertia(0.8,0.3,0.05),...
    'g',                 9.8);
%% Set the simualation parameters
time_initial = 0; 
time_final = 2;
time_step = 0.01; % sec
time_vec = time_initial:time_step:time_final;
max_iter = length(time_vec);

% Create the state vector
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

% Loop through the timesteps and update
history = zeros(6,max_iter-1);

l0=params.l0;
l1=params.l1;
l2=params.l2;
r0=params.r0;
r1=params.r1;
r2=params.r2;
m1=params.m1;
m2=params.m2;
m3=params.m3;
I1=params.I1;
I2=params.I2;
I3=params.I3;
g = params.g;
Ix1 = I1(1);
Iy1 = I1(2);
Iz1 = I1(3);
Ix2 = I2(1);
Iy2 = I2(2);
Iz2 = I2(3);
Ix3 = I3(1);
Iy3 = I3(2);
Iz3 = I3(3);

M = subs(M);
C = subs(C);
N = subs(N);

for iter = 1:max_iter-1
    
    theta1 = state(1);
    theta2 = state(2);
    theta3 = state(3);
    dtheta1 = state(4);
    dtheta2 = state(5);
    dtheta3 = state(6);
    
    
    M_value = subs(M,[t1,t2,t3],[theta1,theta2,theta3]);
    M_value = double(M_value);
    C_value = subs(C,[t1,t2,t3,dt1,dt2,dt3],[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3]);
    C_value = double(C_value);
    N_value = subs(N,[t1,t2,t3],[theta1,theta2,theta3]);
    N_value = double(N_value);
    
    theta1_error = theta1_des - theta1;
    theta2_error = theta2_des - theta2;
    theta3_error = theta3_des - theta3;
    

    
    tau1 = 0;%theta1_error - dtheta1;
    tau2 = 3 * theta2_error - 1*dtheta2;
    tau3 = 6 * theta3_error - 0.1*dtheta3;
    tau = [tau1;tau2;tau3];
    tau =  tau + N_value;
    tau = min(max(tau,-50),50);

    timeint = time_vec(iter:iter+1);
    
    [tsave, xsave] = ode45(@(t,s) dynamic_for_q4(params, state, tau,M_value,C_value,N_value), timeint, state);
    state = xsave(end, :)';
    history(:,iter) = state;
end
% plot the result
subplot(2,3, 1);
plot(1:max_iter-1,history(1,:));
title("theta1")
subplot(2,3, 4);
plot(1:max_iter-1,history(4,:));
title("theta1-dot")
subplot(2,3, 2);
plot(1:max_iter-1,history(2,:));
title("theta2")
subplot(2,3, 5);
plot(1:max_iter-1,history(5,:));
title("theta2-dot")
subplot(2,3, 3);
plot(1:max_iter-1,history(3,:));
title("theta3")
subplot(2,3, 6);
plot(1:max_iter-1,history(6,:));
title("theta3-dot")


