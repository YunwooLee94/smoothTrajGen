function [pts,time_stamp] = eval_spline(knots, opt_p)
    num_eval = 20;
    poly_order = 6;
    pts = [];
    time_stamp=[];
    for i =1:length(knots)-1
        t1 = knots(i);
        t2 = knots(i+1);
        eval_time_horizon = linspace(t1,t2,num_eval);
        eval_time_horizon = eval_time_horizon(1:end-1);
        Dn = time_scaling_mat(t2-t1,poly_order);
        for j = 1: num_eval-1
            j_eval = (eval_time_horizon(j)-t1)/(t2-t1);
            x_temp = t_vec(poly_order,j_eval,0).'*Dn*opt_p((i-1)*(poly_order+1)+1:i*(poly_order+1));
            pts=[pts;x_temp];
        end
        time_stamp=[time_stamp,eval_time_horizon];
    end
    
end

