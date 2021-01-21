function V = J_ellipse_z(q, xi, yi, zi, thi, hi, a, b, xc,yc)

% Find value of parameter t
q = q - [xc ; yc];
q = ROT(q, -thi);
t = atan2( q(2)/b, q(1)/a );

% Corresponding point on base pattern
V = ROT([a*cos(t)/zi ; b*sin(t)/zi], thi)+[xc ; yc];
end