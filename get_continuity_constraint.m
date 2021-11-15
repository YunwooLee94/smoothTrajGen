function [Aeq, beq] = get_continuity_constraint(dt1, dt2)
    N_constraint = 3; %position velocity acceleration continuity
    poly_order = 6;
    
    Aeq = zeros(N_constraint, 2*(poly_order+1));
    beq = zeros(N_constraint, 1);
    
    D1 = time_scaling_mat(dt1, poly_order);
    D2 = time_scaling_mat(dt2, poly_order);
    % position continuity
    Aeq(1,1:poly_order+1) = t_vec(poly_order,1,0).'*D1;
    Aeq(1,poly_order+2:2*(poly_order+1)) = -t_vec(poly_order,0,0).'*D2;
    % velocity continuity
    Aeq(2,1:poly_order+1) = t_vec(poly_order,1,1).'*D1*dt2;
    Aeq(2,poly_order+2:2*(poly_order+1)) = -t_vec(poly_order,0,1).'*D2*dt1;
    % acceleration continuity
    Aeq(3,1:poly_order+1) = t_vec(poly_order,1,2).'*D1*dt2^2;
    Aeq(3,poly_order+2:2*(poly_order+1)) = -t_vec(poly_order,0,2).'*D2*dt1^2;
    
end