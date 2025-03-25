%function to use formula to convert homogenous T to T_inverse

function inverse = T_inverse(T_homog)
    R = T_homog(1:3, 1:3);
    p = T_homog(1:3,4);
    inverse = [transpose(R), -transpose(R)*p;
                0 0 0 1];
end