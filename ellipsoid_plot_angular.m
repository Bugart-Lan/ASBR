function ellipsoid_plot_angular(Jw)
% ellipsoid_plot_angluar.m plot the manipulability ellipsoids for the
% angular velocities.
%
% Inputs:
%   Jw: the top three rows of the Jacobian of the manipulator

A = Jw * Jw';
[V, D] = eig(A);
[X, Y, Z] = ellipsoid(0, 0, 0, D(1,1), D(2,2), D(3,3));

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
quiver3(0,0,0, V(1,1), V(2,1), V(3,1), 0, 'r', 'LineWidth', 4);
quiver3(0,0,0, V(1,2), V(2,2), V(3,2), 0, 'r', 'LineWidth', 4);
quiver3(0,0,0, V(1,3), V(2,3), V(3,3), 0, 'r', 'LineWidth', 4);
%plotting scaled eigenvalues
quiver3(0, 0, 0, D(1,1)*V(1,1), D(1,1)*V(2,1), D(1,1)*V(3,1), 0, 'b', 'LineWidth', 2);
quiver3(0, 0, 0, D(2,2)*V(1,2), D(2,2)*V(2,2), D(2,2)*V(3,2), 0, 'b', 'LineWidth', 2);
quiver3(0, 0, 0, D(3,3)*V(1,3), D(3,3)*V(2,3), D(3,3)*V(3,3), 0, 'b', 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Eigenvectors and Ellipsoid');
axis equal
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
end