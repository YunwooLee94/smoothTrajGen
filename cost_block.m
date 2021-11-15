function mat_cost = cost_block(poly_order)
mat_cost = zeros(poly_order+1,poly_order+1);
for i = 4:poly_order+1
    for j = 4:poly_order+1
        mat_cost(i,j) = (i-1)*(i-2)*(i-3)*(j-1)*(j-2)*(j-3)/(i+j-7);
        
    end
end

end