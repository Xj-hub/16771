%% q_a
z = 0.8;
xl = 0.2;
xr = -0.2;
[theta_la,theta_lk] = q4_calcIK(z,xl);
[theta_ra,theta_rk] = q4_calcIK(z,xr);
l1 = 0.5;
l2 = 0.5;
groundl = 0;
groundr = 0;
gesture = [xl,xr,groundl,groundr];
theta = [theta_la,theta_lk,theta_ra,theta_rk];
plot_leg(gesture,theta,l1,l2)

%% q_b
robo_params = struct(...
    'm',           80, ...
    'g',                9.8,...
    'I',                2);
%% set initial and des state
state_init = [0.8,-1,0,0.1,0,0.1]';%z, z_dot, x, x_dot, theta, theta_dot
state_curr = state_init;
state_des = [0.8,0,0,0,0,0]';

%% Set the simualation parameters
time_initial = 0; 
time_final = 5;
time_step = 0.01; % sec
time_vec = time_initial:time_step:time_final;
max_iter = length(time_vec);
%% Set up k and d
k1 = 1000;
k2 = 100;
k3 = 100;
d1 = 300;
d2 = 75;
d3 = 75;
%% q_c
A = -l1*sin(theta_la)-l2*cos(theta_la+theta_lk);
B = -l1*sin(theta_la)-l2*sin(theta_la+theta_lk);
C = -l1*sin(theta_ra)-l2*cos(theta_ra+theta_rk);
D = -l1*sin(theta_ra)-l2*sin(theta_ra+theta_rk);
Q = -l2*cos(theta_la+theta_lk);
R = -l2*sin(theta_la+theta_lk);
S = -l2*cos(theta_ra+theta_rk);
T = -l2*sin(theta_ra+theta_rk);
E = C*B - A*D;
V = Q*B - R*A;
W = S*D - T*C;

J = [C*V/E, D*V/E, (-V-Q*D+R*C)/(2*E)-0.5;
     0, 0, -0.5;
     -A*W/E, -B*W/E, (W+S*B-T*A)/(2*E)-0.5;
     0, 0, -0.5;];
%% Loop through the timesteps and update
history = zeros(6,max_iter-1);
torque = zeros(4,max_iter-1);
for iter = 1:max_iter-1
    Fz_des = k1 * (state_des(1) - state_curr(1)) + d1 * (state_des(2) - state_curr(2)) + robo_params.m*robo_params.g;
    Fx_des = k2 * (state_des(3) - state_curr(3)) + d2 * (state_des(4) - state_curr(4));
    Ftheta_des = k3 * (state_des(5) - state_curr(5)) + d3 * (state_des(6) - state_curr(6));
    torque(:,iter) = J * [Fx_des;Fz_des;Ftheta_des];
    timeint = time_vec(iter:iter+1);
    [tsave, xsave] = ode45(@(t,s) q4_dynamic(robo_params, s,Fz_des,Fx_des,Ftheta_des), timeint, state_curr);
    state_curr = xsave(end, :)';
    history(:,iter) = state_curr;
end
%% plot result
figure
subplot(1,2,1)
plot(time_vec(1:end-1),history(1,:),'r');
hold on;
plot(time_vec(1:end-1),history(3,:),'g');
plot(time_vec(1:end-1),history(5,:),'b');
legend('z','x','theta') 
hold off;
subplot(1,2,2)
plot(time_vec(1:end-1),torque(1,:),'*');
hold on;
plot(time_vec(1:end-1),torque(2,:),'g*');
plot(time_vec(1:end-1),torque(3,:),'b*');
plot(time_vec(1:end-1),torque(4,:),'m');
legend('tau_lk','tau_lh','tau_rk','tau_rh') 
hold off;
%% q_d
[iter, thetas, poses] = q4_IK(theta, [0,0.8,0.1],[0,0.7,0.1],l1,l2);

 