
l =  5;                                                 % Length
w =  5;                                                 % Width
h = 30;                                                 % Height
X = [-1  -1   1   1  -1; -1  -1   1   1  -1]*l;
Y = [-1   1   1  -1  -1; -1   1   1  -1  -1]*w;
Z = [ 1   1   1   1   1;  0   0   0   0   0]*h;

figure(2)
surf(X, Y, Z)                                           % Plot Walls - Building #1
hold on
patch(X(1,:), Y(1,:), Z(1,:), 'y')                      % Plot Flat Roof - Building #1
surf(X + ones(size(X))*15, Y + ones(size(Y))*20, Z)     % Plot Walls - Building #2
patch(X(1,:) + 15, Y(1,:) + 20, Z(1,:), 'y')            % Plot Flat Roof - Building #2
surf(X + ones(size(X))*35, Y + ones(size(Y))*25, Z)     % Plot Walls - Building #3
patch(X(1,:) + 35, Y(1,:) + 25, Z(1,:), 'y')            % Plot Flat Roof - Building #3
hold off
grid on
axis equal
axis([-10  50    -10  50    0  40])