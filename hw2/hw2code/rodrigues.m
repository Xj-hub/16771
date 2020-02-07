function out = rodrigues(w,theta)
w_hat = wedge(w);
out = eye(3) + sin(theta)*w_hat+w_hat^2*(1-cos(theta));


end

