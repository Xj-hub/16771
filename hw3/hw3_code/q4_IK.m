function [iter, thetas,poses] = q4_IK(theta_init,init_pos, pos,l1,l2)
    step = 0.002;
    pos_des = pos(1:2)';
    rot_des = pos(3);
    theta_curr = theta_init;
    err_norm = 1;
    iter = 0;
    kp = 1;
    ko = 1;
    thetas = zeros(5000,4);
    poses = zeros(5000,3);
    thetas(1,:) = theta_init;
    pos_curr = init_pos(1:2)';
    rot_curr = init_pos(3);
    
    while((err_norm > 1e-3) && (iter < 5000))
        iter = iter + 1;
        
        %calc pos_curr
        theta_la = theta_curr(1);
        theta_lk = theta_curr(2) + theta_la;
        ankle_l = [-0.2,0];
        knee_l = ankle_l + [l1*sin(theta_la),l1*cos(theta_la)];
        hip = knee_l + [l2*sin(theta_lk),l2*cos(theta_lk)];
        pos_curr = hip';
        rot_curr = theta_curr(1) + theta_curr(2) -(theta_init(1) + theta_init(2));
        poses(iter + 1,:) = [hip,rot_curr];
        %calc err
        pos_err = pos_des - pos_curr;
        rot_err = rot_des - rot_curr;
        
        %quat_err = quat_curr.conj*quat_des;
        
        V = [kp*(pos_err); ko*rot_err];
        
        theta_la = theta_curr(1);
        theta_lk = theta_curr(2);
        theta_ra = theta_curr(3);
        theta_rk = theta_curr(4);
        A = -l1*sin(theta_la)-l2*cos(theta_la+theta_lk);
        B = -l1*sin(theta_la)-l2*sin(theta_la+theta_lk);
        Q = -l2*cos(theta_la+theta_lk);
        R = -l2*sin(theta_la+theta_lk);
        C = -l1*sin(theta_ra)-l2*cos(theta_ra+theta_rk);
        D = -l1*sin(theta_ra)-l2*sin(theta_ra+theta_rk);
        S = -l2*cos(theta_ra+theta_rk);
        T = -l2*sin(theta_ra+theta_rk);
        J_l = [A,B,-1;
             Q,R,-1;
             0,0,-1]';
        J_r = [C,D,-1;
             S,T,-1;
             0,0,-1]';
        
        
        
        theta_l = pinv(J_l) * V;
        theta_r = pinv(J_r) * V;
        theta_dot(1:2,:) = theta_l(1:2,:);
        theta_dot(3:4,:) = theta_r(1:2,:);
        %theta_dot = dls_inverse(J,0.5) * V;
        theta_curr = theta_curr + theta_dot' * step;
        thetas(iter + 1,:) = theta_curr;
        err_norm = norm([pos_err; rot_err]);
    end
end

