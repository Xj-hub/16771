function Jb = jacobianBody(Blist, thetalist)

% Takes Blist: The joint screw axes in the end-effector frame when the
%              manipulator is at the home position, in the format of a 
%              matrix with the screw axes as the columns,
%       thetalist: A list of joint coordinates.
% Returns the corresponding body Jacobian (6xn real numbers).


Jb = Blist;
T = eye(4);
for i = length(thetalist) - 1: -1: 1   
    T = T * twistExp(-1 * Blist(:, i + 1),thetalist(i + 1));
	Jb(:, i) = Adjoint(T) * Blist(:, i);
end
end