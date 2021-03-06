function coef = coeff(x0,x1,t)

d = norm(x1-x0);

if d/t>2
    v = 2;
    t = d / v;
end

L = inv([0 0 0 0 0 1;
    0 0 0 0 1 0;
    0 0 0 2 0 0;
    t^5 t^4 t^3 t^2 t 1;
    5*t^4 4*t^3 3*t^2 2*t 1 0;
    20*t^3 12*t^2 6*t 2 0 0]);

coef = [L*[x0(1); 0; 0; x1(1); 0; 0],...
    L*[x0(2); 0; 0; x1(2); 0; 0],...
    L*[x0(3); 0; 0; x1(3); 0; 0]]';

end
