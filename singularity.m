function singularities = singularity(screwaxes, M, joint_limits)

n = size(screwaxes, 2);
q = sym('q', [n, 1], 'real');



T = eye(4);
for i=1:n
    T = simplify(T * expm(screw2S_hat(screwaxes(:, i)) * q(i)));
end

T0 = simplify(T*M);

spaceJ = sym(zeros(6,n));
T_i = eye(4);
for i = 1:n
    if i ==1
        spaceJ(:,1) = screwaxes(:,1);
    else
        adjointT = Adjoint(T_i);
        spaceJ(:,i) = simplify(adjointT * screwaxes(:,i));
    end

    T_i = simplify(T_i * expm(screw2S_hat(screwaxes(:, i)) * q(i)));
end
spaceJ = simplify(spaceJ)

if n ~= 6
    warning('The Jacobian is not square. Taking the first 6 columns for determinant computing');
    squareJ = spaceJ(:, 1:6);
else
    squareJ = spaceJ;
end

detJ = simplify(det(squareJ))
for i = 1:n
    assumeAlso(q(i) >= joint_limits(i,1));
    assumeAlso(q(i) <= joint_limits(i,2));
end

assumeAlso(q(1) == 0);   
assumeAlso(q(4) == 0);  

singularities = solve(detJ == 0, q, 'ReturnConditions', false);

fprintf('Singularity condition (det(J) = 0) is: \n');
disp(detJ);
fprintf('Singularity solutions and conditions:\n');
disp(singularities);

% condition = detJ == 0;
% 
% singularities = struct();
% singularities.detJ = detJ;
% singularities.condition = condition;

end
