function [Aeq0, beq0] = get_init_constraint(x0, v0, a0)
    poly_order = 6;
    dt = 1;
    Aeq0 = zeros(3,poly_order+1);
    beq0 = zeros(3,1);
    Aeq0(1,:) = t_vec(poly_order,0,0).'*time_scaling_mat(dt,poly_order);
    Aeq0(2,:) = t_vec(poly_order,0,1).'*time_scaling_mat(dt,poly_order);
    Aeq0(3,:) = t_vec(poly_order,0,2).'*time_scaling_mat(dt,poly_order);
    
    beq0(1) = x0;
    beq0(2) = v0 * dt; %% v0, a0, power of dt why?
    beq0(3) = a0 * dt;
    
end