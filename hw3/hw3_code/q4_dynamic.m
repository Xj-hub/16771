function [state_dot] = q4_dynamic(robo_params, state,Fz_des,Fx_des,Ftheta_des)
    state_dot = zeros(6,1);
    state_dot(1)= state(2);
    state_dot(3)= state(4);
    state_dot(5)= state(6);
    state_dot(2) = (Fz_des - robo_params.m*robo_params.g)/robo_params.m;
    state_dot(4) = Fx_des/robo_params.m;
    state_dot(6) = Ftheta_des/robo_params.I;
end

