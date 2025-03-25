%function to find the adjoint matrix of T

function Adjoint_T = Adjoint(T)
    R = T(1:3, 1:3);
    p = T(1:3,4);
    Adjoint_T = [R, zeros(3,3); skew(p)*R, R];
end
