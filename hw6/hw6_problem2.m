%%  import the position data
ydata = readmatrix("ydata.txt");
%draw the raw data
plot3(ydata(:,1), ydata(:,2), ydata(:,3),'*r');
len = size(ydata,1);
hold on;
%% input parameters
T = 1;
M = 0.01;
A = eye(6);
A(1,4) = T;
A(2,5) = T;
A(3,6) = T;
B = zeros(6,3);
B(4,1) = T/M;
B(5,2) = T/M;
B(6,3) = T/M;
C = eye(3,6);

u = [0.01,0.01,0.01]';
P0 = diag([50, 50, 50, 10, 10, 10]);
Rv = 1e-5 * eye(3);
Rw = 50 * eye(3);

%% kalman filter
x = [0,0,0,0,0,0]';
P = P0;
x_list = zeros(6,len+1);
error_list = zeros(3,len);
x_list(:,1) = x;
for i = 1:len
    L = A * P * C' * inv(Rw + C*P*C');
    error_list(:,i) = ydata(i,:)'-C * x;
    x = A * x + B * u + L*(ydata(i,:)'-C * x);
    P = (A - L * C) * P * (A - L * C)' + B * Rv * B' + L * Rw * L';
    x_list(:,i+1) = x;
end
plot3(x_list(1,:), x_list(2,:), x_list(3,:));

% for i = 1:len
%     L = P * C' * inv(Rw + C*P*C');
%     x = A * x + B * u;
%     x = x + L*(ydata(i,:)'-C * x);
%     P = A*P*A' + B*Rv*B';
%     P = (eye(6) - L * C) * P;
%     x_list(:,i+1) = x;
% end
plot3(x_list(1,:), x_list(2,:), x_list(3,:));
figure()
subplot(3,1, 1);
plot(1:len,error_list(1,:));
title("x")
subplot(3,1, 2);
plot(1:len,error_list(2,:));
title("y")
subplot(3,1, 3);
plot(1:len,error_list(3,:));
title("z")



