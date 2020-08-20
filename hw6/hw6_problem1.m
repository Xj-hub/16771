J1 = 10/9;
J2 = 10;
c = 0.1;
k = 1;
kI = 1;
omega0 = sqrt(k * (J1+ J2)/(J1*J2));
A = [0 0 1 0;
     0 0 0 1;
     -k/(J1*omega0) k/(J1*omega0) -c/J1 c/J1;
     k/(J2*omega0) -k/(J2*omega0) c/J2 -c/J2];
B = [0;0;kI/omega0;0];
C = [1,1,1,1];
%% part2
e = eig(A);
%% part3
p = [-2,-1,-1+1i,-1-1i];
K = place(A,B,p);
C = K;

%% part4 1
time_initial = 0; 
time_final = 12;
time_step = 0.05; % sec
time_vec = time_initial:time_step:time_final;
max_iter = length(time_vec);

state = zeros(4,1);
state(1)  = 0;    
state(2)  = 0;    
state(3)  = 0;    
state(4)  = 0;   

history = zeros(4,max_iter-1);
for iter = 1:max_iter-1   
    state_ref = [1;1;0;0];
    state_error = state_ref - state;
    I = K * state_error;
    timeint = time_vec(iter:iter+1);
    dynamic = @(state,I) A*state + B*I;
    [tsave, xsave] = ode45(@(t,s) dynamic(state,I), timeint, state);
    state = xsave(end, :)';
    history(:,iter) = state;
end
figure()
subplot(4,1, 1);
plot(time_vec(1,2:end),history(1,:));
title("phi1")
subplot(4,1, 2);
plot(time_vec(1,2:end),history(2,:));
title("phi2")
subplot(4,1, 3);
plot(time_vec(1,2:end),history(3,:));
title("x3")
subplot(4,1, 4);
plot(time_vec(1,2:end),history(4,:));
title("x4")

%% part4 2
time_initial = 0; 
time_final = 12;
time_step = 0.05; % sec
time_vec = time_initial:time_step:time_final;
max_iter = length(time_vec);

state = zeros(4,1);
state(1)  = 0;    
state(2)  = 0;    
state(3)  = 0;    
state(4)  = 0;   
B_disturb = [0;0;0;1/omega0];
history = zeros(4,max_iter-1);
for iter = 1:max_iter-1   
    state_ref = [0;0;0;0];
    state_error = state_ref - state;
    I = K * state_error;
    timeint = time_vec(iter:iter+1);
    dynamic = @(state,I) A*state + B*I + B_disturb;
    [tsave, xsave] = ode45(@(t,s) dynamic(state,I), timeint, state);
    state = xsave(end, :)';
    history(:,iter) = state;
end
figure()
subplot(4,1, 1);
plot(time_vec(1,2:end),history(1,:));
title("phi1")
subplot(4,1, 2);
plot(time_vec(1,2:end),history(2,:));
title("phi2")
subplot(4,1, 3);
plot(time_vec(1,2:end),history(3,:));
title("x3")
subplot(4,1, 4);
plot(time_vec(1,2:end),history(4,:));
title("x4")

