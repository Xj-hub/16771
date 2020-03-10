%%  import the joint data
poses = readmatrix("poses.txt");


%% plot
figure;
plot3(poses(:,2), poses(:,3), poses(:,4));
t_end = size(poses,1);
%% center of mass velocity
x_est = (poses(end,2)-poses(1,2))/(t_end-1);
y_est = (poses(end,3)-poses(1,3))/(t_end-1);
z_est = (poses(end,4)-poses(1,4))/(t_end-1);



A = zeros(180-3,6);
Pdot = zeros(180-3,1);
R_dot = zeros(180-3,3);
for i = 2:t_end-1
    q1 = poses(i-1,[8,5:7]);
    q2 = poses(i+1,[8,5:7]);
    [R1,R2,Rdot] = cal_Rdot(q1,q2);
    
    index = i - 1;
    A(index*3-2:3*index,:) = [Rdot,-eye(3)]; 
    R_dot(index*3-2:3*index,:) = Rdot;
    Pdot(index*3-2:3*index) = -[(poses(i+1,2)-poses(i-1,2))/2, (poses(i+1,3)-poses(i-1,3))/2, (poses(i+1,4)-poses(i-1,4))/2]';
end
vc = pinv(A)*Pdot;

%% calculate the inertia tensor
wb_hat = zeros(180-3,3);
wb = zeros(180-3,1);
wb_dot = zeros(180-3-6,1);
A = zeros(180-3,6);
for i = 2:t_end-1
    index = i - 1;
    w_m = quat2rotm(poses(i,[8,5:7]))' * R_dot(index*3-2:3*index,:);
    wb_hat(index*3-2:3*index,:) = w_m;
%     w1 = w_m(3,2);
%     w3 = w_m(2,1);
%     w2 = w_m(1,3);
%     w_vec = [w1,w2,w3]'; 
    w_vec = vex(w_m);
    wb(index*3-2:3*index) = w_vec;
end
for i = 2:t_end-1-2
    index = i - 1;
    plus = i+1;
    minus = i-1;
    wb_dot(index*3-2:3*index) = (wb(plus*3-2:3*plus) - wb(minus*3-2:3*minus))/2;
end
A = zeros(180-3-6,6);
for i = 1:57
    index = i;
    wb_ = wb(4:end-3);
    A(index*3-2:3*index,:) = enlarge(wb_dot(index*3-2:3*index)) + ...
        wedge(wb_(index*3-2:3*index))* enlarge(wb_(index*3-2:3*index)); 
end
%%
[V,D] = eig(A'*A);
param_hat = [1,0,0,0,0,0]';
param = V*param_hat;
I = [param(1),param(2),param(3);
     param(2),param(4),param(5);
     param(3),param(5),param(6)];
[V_I, D_I] = eig(I);

%% angular momentum
H = 0;
for i = 1: t_end-3
    H = H+ quat2rotm(poses(i+1,[8,5:7])) *I * wb(i*3-2:3*i);
end
H = H/(t_end-3)

%% 120s pose
%plot(wb(1:3:end))
pose = poses(61,:);
wb_x = wb(1:3:end);
wb_y = wb(2:3:end);
wb_z = wb(3:3:end);
x = linspace(2,60,59);
wb_x = mean(wb(1:3:end));
wb_y = 0.05987*sin(0.2963*121-1.597);
wb_z = 0.07151*sin(0.2963*121+3.229);
wb_120 = [wb_x,wb_y,wb_z]';
R_120 = H * pinv(I*wb_120);
pose_quat = rotm2quat(R_120);
R_60 = quat2rotm(poses(61,[8,5:7]));
q_center = vc(1:3);
pose_xyz_center = poses(61,2:4)+(R_60*q_center)' + vc(4:6)'*60;
pose_120(1) = 121;
pose_120(2:4) = pose_xyz-(R_120*q_center)';
pose_120(5:8) = pose_quat([2:4,1]);