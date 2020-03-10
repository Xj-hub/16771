function v = vex(S)
%     if trace(abs(S)) > 10*eps
%         error('SMTB:vex:badarg', 'argument is not skew symmetric tr=%g', trace(abs(S)));
%     end
    if all(size(S) == [3 3])
        v = 0.5*[S(3,2)-S(2,3); S(1,3)-S(3,1); S(2,1)-S(1,2)];
    elseif all(size(S) == [2 2])
        v = 0.5*(S(2,1)-S(1,2));
    else
        error('SMTB:vex:badarg', 'argument must be a 2x2 or 3x3 matrix');
    end
