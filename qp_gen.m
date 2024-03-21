function QP = qp_gen(knots, waypoints, v0, a0)
    poly_order = 6;
    n_seg = length(knots)-1; % # of interval
    n_var_total = n_seg * (poly_order+1);
    % Q = zeros(n_var_total,n_var_total);
    Q=[];

    %% Cost_Matrix
    for i = 1:n_seg
       Dn = time_scaling_mat(knots(i+1)-knots(i),poly_order);
       dn = knots(i+1)-knots(i);
       Q_sub = Dn*cost_block(poly_order)*Dn/(dn^5);
       Q= blkdiag(Q,Q_sub);
    end

%% Initial Constraint
    [Aeq0_,beq0_] = get_init_constraint(waypoints(1),v0,a0);
    Aeq0 = zeros(3,n_var_total);
    beq0 = zeros(3,1);
    Aeq0(1:3,1:(poly_order+1)) = Aeq0_;
    beq0(1:3,1) = beq0_;
%% Waypoint Constraint
    Aeq1 = zeros(n_seg, n_var_total);
    beq1 = zeros(n_seg, 1);
    for i = 1:n_seg
        Dn = time_scaling_mat(knots(i+1)-knots(i), poly_order);

        Aeq1(i, (i-1)*(poly_order+1)+1:i*(poly_order+1)) = t_vec(poly_order,1,0).'*Dn;
        beq1(i) = waypoints(i+1);
    end
%% Continuity Constraint
    Aeq2 = zeros(3*(n_seg-1), n_var_total);
    beq2 = zeros(3*(n_seg-1), 1);
    for i = 1:n_seg-1
        [Aeq2((3*i-2):3*i, (i-1)*(poly_order+1)+1:(i+1)*(poly_order+1)), ...
            beq2((3*i-2):3*i, 1)] = get_continuity_constraint(knots(i+1)-knots(i),knots(i+2)-knots(i+1));
    end
    
%% To Sum up
    Aeq = [Aeq0;Aeq1;Aeq2];
    beq = [beq0;beq1;beq2];
    QP.Q = Q;
    QP.Aeq = Aeq;
    QP.beq = beq;
    QP.n_var = n_var_total;
end