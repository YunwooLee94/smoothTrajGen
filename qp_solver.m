function p = qp_solver(QP)
    cvx_begin quiet
        variable p(QP.n_var)
        minimize(p.'*QP.Q*p);
        QP.Aeq * p == QP.beq;
    cvx_end

end