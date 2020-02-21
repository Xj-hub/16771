function J_inv = dls_inverse(J,lambda)

    J_inv = J'*inv(J*J'+lambda^2 * eye(size(J,1)));


end

