function [theta_a, theta_k] = q4_calcIK(z,x)
add = 2*atan(x/z);
minus = acos(2*z^2 + 2*x^2 - 1);
theta_a = (add+minus)/2;
theta_k = (add-minus)/2;
theta_k = theta_k - theta_a;
end

