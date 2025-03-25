%Function to compute the ellipsoid volume of the Jacobian
function volume = J_ellipsoid_volume(J)
%Finds size of J by rows
n = size(J,1);
%calculate determinant of J * J'
detJJt = det(J * J');
%computes the volume of the ellipsoid based on formula
volume = sqrt(detJJt);

end
