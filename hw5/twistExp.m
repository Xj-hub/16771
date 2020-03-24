function exp_twist = twistExp(twist,theta)

% Takes a 6-vector (representing a spatial velocity) and theta.
% Returns a T matrix in SE(3) that is achieved by traveling along/about the 
% screw axis S for a distance theta from an initial configuration T = I.

v = twist(1:3);
w = twist(4:6);
if (w == [0;0;0])
    exp_twist = [eye(3), v*theta; 0,0,0,1];
else
    exp_twist = [rodrigues(w,theta),(eye(3) - rodrigues(w, theta))*VecToso3(w)*v...
        + w*dot(w,v)*theta; 0,0,0,1];
end
end

