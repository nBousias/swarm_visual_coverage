function B = ROT(A,theta)

R = [    cos(theta)  -sin(theta);
            sin(theta)  cos(theta)];
B = zeros( size(A) );
N = length(A(1,:));
for i=1:N
    B(:,i) = R * A(:,i);
end
