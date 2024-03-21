    function vec = t_vec(poly_order, t, n_diff)
    vec = zeros(poly_order+1,1);
    switch n_diff
        case 0
            for i = 1:poly_order+1
                vec(i) = t^(i-1);
            end
        case 1
            for i = 2:poly_order+1
                vec(i) = (i-1)*t^(i-2);
            end
        case 2
            for i = 3:poly_order+1
                vec(i) = (i-1)*(i-2)*t^(i-3);
            end
        case 3
            for i =4:poly_order+1
                vec(i) = (i-1)*(i-2)*(i-3)*t^(i-4);
            end
    end
end

