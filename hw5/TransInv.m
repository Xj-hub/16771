function invT = TransInv(T)

% Takes a transformation matrix T.
% Returns its inverse. 


[R, p] = TransToRp(T);
invT = [R', -R' * p; 0, 0, 0, 1];
end

