function [R, p] = TransToRp(T)

% Takes the transformation matrix T in SE(3) 
% Returns R: the corresponding rotation matrix
%         p: the corresponding position vector .

R = T(1: 3, 1: 3);
p = T(1: 3, 4);
end

