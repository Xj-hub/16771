%%  import the joint data
joint_angles = readmatrix("JointData.txt");

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

%% calculate xyz position of tooltip
xyz_tip = NaN(length(joint_angles), 3);

for ii = 1:length(joint_angles)
    gab = calcG(twists, joint_angles(ii,:), g_init);
    xyz_tip(ii,:) = gab(1:3,4);

end

%% save
fID = fopen('xyz2.txt', 'w');
[M,N] = size(xyz_tip);
for m = 1:M
    fprintf(fID, '%1.6f %1.6f %1.6f \n', xyz_tip(m,1), xyz_tip(m,2), xyz_tip(m,3) );
end
fclose(fID);

%% plot
figure;
a = xyz_tip(1,:);
b = xyz_tip(2000,:);
c = xyz_tip(6000,:);
v= cross(a-b,c-b)*50;
plot3(xyz_tip(:,1), xyz_tip(:,2), xyz_tip(:,3), '*k',[b(1), b(1) - v(1)],[b(2), b(2) - v(2)],[b(3), b(3) - v(3)],'-or');
%axis equal; axis square;

