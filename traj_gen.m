function p_opt3 = traj_gen(p_mpc, knots, V, V_info)

    waypoints = zeros(3, length(knots));
    for i = 1:(1+p_mpc.Nmpc)
        waypoints(:, i) = V{i}.xyz;
    end

    v0 = V_info.vel_init;
    a0 = V_info.acc_init;
    
    temp_QPx = qp_gen(knots, waypoints(1, :), v0(1), a0(1));
    temp_QPy = qp_gen(knots, waypoints(2, :), v0(2), a0(2));
    temp_QPz = qp_gen(knots, waypoints(3, :), v0(3), a0(3));
    opt_px = qp_solver(temp_QPx);
    opt_py = qp_solver(temp_QPy);
    opt_pz = qp_solver(temp_QPz);
    p_opt3 = [opt_px, opt_py, opt_pz];
%     [x_array, t_array] = eval_spline(knots,opt_px);
%     [y_array, ~] = eval_spline(knots,opt_py);
%     [z_array, ~] = eval_spline(knots,opt_pz);
 
    %video_writer(t_array,x_array,y_array,z_array,waypoints);
end