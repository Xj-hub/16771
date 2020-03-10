function [R1,R2,Rdot] = cal_Rdot(q1,q2)
    R1 = quat2rotm(q1);
    R2 = quat2rotm(q2);
    Rdot = (R2-R1)/2;
end

