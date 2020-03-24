function se3mat = VecTose3(V)

% Takes a 6-vector (representing a spatial velocity).
% Returns the corresponding 4x4 se(3) matrix.


se3mat = [VecToso3(V(1: 3)), V(4: 6); 0, 0, 0, 0];
end

