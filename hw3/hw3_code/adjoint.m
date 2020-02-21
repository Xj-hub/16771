function ad = adjoint(theta,twist)
% x = theta(1);
% y = theta(2);
% z = theta(3);
% R = rotx(x)*roty(y)*rotz(z);
e = twistExp(twist,theta);
R = e(1:3,1:3);
p = e(1:3,4);
ad = [R,wedge(p)*R;zeros(3), R];

end

