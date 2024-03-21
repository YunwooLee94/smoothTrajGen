function D = time_scaling_mat(dt, poly_order)
    D = zeros(poly_order+1, poly_order+1);
    for i = 1:poly_order+1
       D(i,i) = dt^(i-1);
    end
end

