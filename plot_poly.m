function H = plot_poly( P , colorspec )

hh = ishold;
hold on

if nargin == 1
    colorspec = 'b';
end

if ~isempty(P)
    x = P(1,:);
    y = P(2,:);

    nan_indices = find( isnan(P(1,:)) ); % The indices are the same for x and y
    indices = [ 0 nan_indices length(x)+1 ];

    % Collect the handles of all the plots
    H = zeros(1, length( nan_indices )+1);
    for i=1:length( nan_indices )+1

        tmpP = [x( indices(i) + 1 : indices(i+1) - 1 ) ; ...
                y( indices(i) + 1 : indices(i+1) - 1 )];
        % if the last vertex is not the same as the last, add the first vertex to
        % the end
        if ~isequal( tmpP(:,1), tmpP(:,end) )
            tmpP = [tmpP tmpP(:,1)];
        end

        h = plot( tmpP(1,:), tmpP(2,:), colorspec );

        H(i) = h;

    end

    if ~hh
		hold off;
	end
end
