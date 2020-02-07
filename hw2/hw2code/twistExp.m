function exp_twist = twistExp(twist,theta)
%TWISTE �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
v = twist(1:3);
w = twist(4:6);
if (w == [0;0;0])
    exp_twist = [eye(3), v*theta; 0,0,0,1];
else
    exp_twist = [rodrigues(w,theta),(eye(3) - rodrigues(w, theta))*cross(w,v)...
        + w*dot(w,v)*theta; 0,0,0,1];
end
end

