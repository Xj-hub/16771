function AdT = Adjoint(T)

% Takes T a transformation matrix SE3. 
% Returns the corresponding 6x6 adjoint representation [AdT].


[R, p] = TransToRp(T);
AdT = [R, VecToso3(p) * R;zeros(3), R];
end
