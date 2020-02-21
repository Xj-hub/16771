function [iter, thetas] = calcIK(twists, theta_init, g_init, pos)
    step = 0.005;
    pos_des = pos(1:3)';
    quat_des = quaternion(pos(7),pos(4),pos(5),pos(6));
    theta_curr = theta_init;
    err_norm = 1;
    iter = 0;
    kp = 1;
    ko = 1;
    thetas = zeros(10000,7);
    %poses = zeros(5000,7);
    thetas(1,:) = theta_init;
    while((err_norm > 1e-3) && (iter < 10000))
        iter = iter + 1;
        g = calcFK(twists, theta_curr, g_init);
        pos_curr = g(1:3,4);
        quat_curr = quaternion(g(1:3,1:3),'rotmat','point');
        %poses(iter,:) = [pos_curr',quat_curr.compact];
        pos_err = pos_des - pos_curr;
        quat_err = quat_des*(quat_curr.conj);
        %quat_err = quat_curr.conj*quat_des;
        quat_err_compact = quat_err.compact';
        V = [kp*(pos_err); ko*quat_err_compact(1)*quat_err_compact(2:end)];
        J = calcJ(twists, theta_curr);
        %theta_dot = pinv(J) * V;
        theta_dot = dls_inverse(J,0.01) * V;
        theta_curr = theta_curr + theta_dot' * step;
        thetas(iter + 1,:) = theta_curr;
        err_norm = norm([pos_err; quat_err_compact(1)*quat_err_compact(2:end)]);
    end
end

