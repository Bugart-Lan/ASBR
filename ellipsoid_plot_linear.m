function ellipsoid_plot_linear(Jv)
% ellipsoid_plot_linear.m plot the manipulability ellipsoids for the
% linear velocities.
%
% Inputs:
%   Jv: the bottom three rows of the Jacobian of the manipulator

A = Jv * Jv';
[V, D] = eig(A);
disp('Eigenvalues:'), disp(diag(D));
disp('Eigenvectors:'), disp(V);
r_min = 1e-6;
[X, Y, Z] = ellipsoid(0, 0, 0, sqrt(max(D(1,1),r_min)), sqrt(max(D(2,2),r_min)), sqrt(max(D(3,3),r_min)));

ellipsoid_points = [X(:) Y(:) Z(:)]';
rotated_points = V * ellipsoid_points;
X_rot = reshape(rotated_points(1,:), size(X));
Y_rot = reshape(rotated_points(2,:), size(Y));
Z_rot = reshape(rotated_points(3,:), size(Z));


figure;
s = surf(X_rot, Y_rot, Z_rot);
hold on;
set(s, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
%plotting eigenvectors
quiver3(0,0,0, 100*V(1,1), 100*V(2,1), 100*V(3,1), 0, 'r', 'LineWidth', 4, 'AutoScale', 'off');
quiver3(0,0,0, 100*V(1,2), 100*V(2,2), 100*V(3,2), 0, 'r', 'LineWidth', 4, 'AutoScale', 'off');
quiver3(0,0,0, 100*V(1,3), 100*V(2,3), 100*V(3,3), 0, 'r', 'LineWidth', 4, 'AutoScale', 'off');
%plotting scaled eigenvalues
quiver3(0, 0, 0, sqrt(D(1,1))*V(1,1), sqrt(D(1,1))*V(2,1), sqrt(D(1,1))*V(3,1), 0, 'b', 'LineWidth', 2);
quiver3(0, 0, 0, sqrt(D(2,2))*V(1,2), sqrt(D(2,2))*V(2,2), sqrt(D(2,2))*V(3,2), 0, 'b', 'LineWidth', 2);
quiver3(0, 0, 0, sqrt(D(3,3))*V(1,3), sqrt(D(3,3))*V(2,3), sqrt(D(3,3))*V(3,3), 0, 'b', 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Eigenvectors and Ellipsoid');
axis equal;
grid on;
view(3);

% TODO: check if the axes are correct
% ang = rad2deg(m_rotm2zyz(V));
% rotate(s, [0 0 1], ang(1));
% rotate(s, [0 1 0], ang(2));
% rotate(s, [0 0 1], ang(3));

% title("Angular Ellipsoid Plot at Home Configuration");
% xlabel('Eigenvector 1 = %d', V(:,1));
% ylabel('Eigenvector 2 = %d', V(:,2));
% zlabel('Eigenvector 3 = %d', V(:,3));

% [X, Y, Z] = ellipsoid(0, 0, 0, D(1,1), D(2,2), D(3,3));
% s = surf(X, Y, Z);
% 
% % TODO: check if the axes are correct
% ang = rad2deg(m_rotm2zyz(V));
% rotate(s, [0 0 1], ang(1));
% rotate(s, [0 1 0], ang(2));
% rotate(s, [0 0 1], ang(3));
% 
end