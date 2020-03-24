function out = rodrigues(w,theta)

%Takes the 3x1 vector omega and theta
%Return the result used in twistExp function
w_hat = VecToso3(w);
out = eye(3) + sin(theta)*w_hat+w_hat^2*(1-cos(theta));


end

