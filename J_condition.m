%function to find the conditional number of the Jacobian
function conditional_num = J_condition(J)

%
s = svd(J);
conditional_num = s(1)/s(end);

end
