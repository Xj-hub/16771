syms theta_dd theta_d theta;
syms x_dd x_d x;
syms alpha_dd alpha_d alpha;
syms Ip Iw Mw m M;
syms d r l;
syms taul taur dl dr;


Itheta = Ip + d^2*(Mw + Iw/(r^2));
theta_dd = (d/(r*Itheta))*(taul - taur) + (d/Itheta)*(dl - dr);
