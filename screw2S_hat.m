%function to convert screw axes (w and v) to se3 matrix (S_hat)

function se3matrix = screw2S_hat(V)
    w = V(1:3);
    v = V(4:6);
    se3matrix = [skew(w), v; 0 0 0 0];
end