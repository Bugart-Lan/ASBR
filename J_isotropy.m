function iso = J_isotropy(J)

s = svd(J);

iso = sqrt(s(1)/s(end));

end
