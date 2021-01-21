function area = polyarea_nan(x, y)
% The outer contour is the last NaN delimited segment

nan_indices = find(isnan( x ));
indices = [ 0 nan_indices length(x)+1 ];
area = 0;

if ~isempty(nan_indices)
    for i=1:length(nan_indices)+1

        tempx = x( indices(i)+1 : indices(i+1)-1 );
        tempy = y( indices(i)+1 : indices(i+1)-1 );
        if ispolycw(tempx, tempy)
            % external contour
            area = area + polyarea(tempx, tempy);
        else
            % internal contour
            area = area - polyarea(tempx, tempy);
        end

    end
else
    area = polyarea(x, y);
end

