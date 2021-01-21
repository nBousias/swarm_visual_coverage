function W = sensed_partitioning_uniform_anisotropic_cell(region, C, f, i,N)

% Initialize cell to sensing disk
W = C{i};
% Loop over all other nodes
for j=1:N
    if j ~= i
        % Remove a portion of the sensing region if fi <= fj
        if f(i) <= f(j)
            % Remove Cj
			if ~isempty(W)
				[pbx, pby] = polybool( 'minus', W(1,:), W(2,:), C{j}(1,:), C{j}(2,:) );

				% Save to the current cell
				W = [pbx ; pby];
			else
				break;
			end
        end
    end
end

% AND with the region omega
if ~isempty(W)
    [pbx, pby] = polybool( 'and', W(1,:), W(2,:), region(1,:), region(2,:) );
    W = [pbx ; pby];
end