%%  define robot structure 
base = [0.75, 0.5, 1.0];
base_shoulder = [-0.14,0.22,0.346];
shoulder_tip = [0,0,0.85];
shoulder_elbow = [0,0.045,0.55];
tip_mark = [0,0,0.12+0.06];
%%  calculate link length
shoulder = base + base_shoulder;
elbow = shoulder + shoulder_elbow;
tip = shoulder + shoulder_tip;
mark = tip + tip_mark;
g_init = [0,-1, 0, mark(1);
          1, 0, 0, mark(2);
          0, 0, 1, mark(3);
          0, 0, 0, 1];
      
%%  calculate twist
w1 = [0,0,1];
w2 = [-1,0,0];
w3 = [0,0,1];
w4 = [-1,0,0];
w5 = [0,0,1];
w6 = [-1,0,0];
w7 = [0,0,1];

tw1 = cat(1,-wedge(w1')*shoulder',w1');
tw2 = cat(1,-wedge(w2')*shoulder',w2');
tw3 = cat(1,-wedge(w3')*shoulder',w3');
tw4 = cat(1,-wedge(w4')*elbow',w4');
tw5 = cat(1,-wedge(w5')*tip',w5');
tw6 = cat(1,-wedge(w6')*tip',w6');
tw7 = cat(1,-wedge(w7')*tip',w7');

twists = [tw1,tw2,tw3,tw4,tw5,tw6,tw7];
theta = [0.1,0.2,0.3,0.4,0.5,0.6,0.7];
j = calcJ(twists,theta);
x_s = [0.44543; 1.12320; 2.22653; -0.29883; 0.44566; 0.84122; -0.06664]';
x_d1 = [ 0.46320; 1.16402; 2.22058; -0.29301; 0.41901; 0.84979; 0.12817]';
x_d2 = [ 0.49796; 0.98500; 2.34041; -0.11698; 0.07755; 0.82524; 0.54706]';
%% q3_3
[iter, thetas_d1] = calcIK(twists, theta, g_init, x_d1);
thetas_d1 = thetas_d1(1:iter,:);
fID = fopen('q3_4_d1.txt', 'w');
[M,N] = size(thetas_d1);
for m = 1:M
    fprintf(fID, '%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f %1.6f \n',...
        thetas_d1(m,1),thetas_d1(m,2), thetas_d1(m,3),...
        thetas_d1(m,4),thetas_d1(m,5), thetas_d1(m,6),thetas_d1(m,7));
end
fclose(fID);
[iter, thetas_d2] = calcIK(twists, theta, g_init, x_d2);
thetas_d2 = thetas_d2(1:iter,:);
fID = fopen('q3_4_d2.txt', 'w');
[M,N] = size(thetas_d2);
for m = 1:M
    fprintf(fID, '%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f %1.6f \n',...
        thetas_d2(m,1),thetas_d2(m,2), thetas_d2(m,3),...
        thetas_d2(m,4),thetas_d2(m,5), thetas_d2(m,6),thetas_d2(m,7));
end
fclose(fID);