function uTH = control_uniform_rotational(region, W, Cgs,C_maxz, f, i, Jz_maxz,Jz_minz)

N = length(W);
uTH = 0;

if ~isempty(W{i})
    % Keep only CW (external) contours
    % If Wi has holes, remove the corresponding contours since they
    % dont contribute to the control law
    % Find NaN indices
    nanindex = find( isnan( W{i}(1,:) ) );
    if ~isempty( nanindex )
        Wi = []; % It will contain the external contour of Wi
        indices = [ 0 nanindex length( W{i}(1,:) )+1 ];
        for k=1:length(nanindex)+1
            % Keep a part of Wi
            tempx = W{i}(1, indices(k)+1 : indices(k+1)-1 );
            tempy = W{i}(2, indices(k)+1 : indices(k+1)-1 );
            if ispolycw(tempx, tempy)
                Wi = [Wi [tempx ; tempy]];
            end

        end
    else
        % Wi has no holes
        Wi = W{i};
    end


    % Integrate over the arcs
    % Loop over all line segments of Wi
    % Wi is used instead of W{i} to include cases with NaNs
    % Wi is a closed list of points
    for k=1:length(Wi(1,:))-1
        % endpoints of the current line segment
        pt1 = Wi(:,k);
        pt2 = Wi(:,k+1);
        

        % Check if they are on the boundary. If both are on it dont
        % integrate
        [~, onB1] = inpolygon( pt1(1), pt1(2), region(1,:), region(2,:) );
        [~, onB2] = inpolygon( pt2(1), pt2(2), region(1,:), region(2,:) );

        if ~(onB1 && onB2)
            % Check if any of them is on Ci, if not, dont integrate
            [~, onCi1] = inpolygon( pt1(1), pt1(2), Cgs{i}(1,:), Cgs{i}(2,:) );
            [~, onCi2] = inpolygon( pt2(1), pt2(2), Cgs{i}(1,:), Cgs{i}(2,:) );

            if (onCi1 && onCi2)
                % Check if they are both inside Cj for all j in overlap
                % If they are, then this is a dominant arc
                free_arc = 1; % Free arc flag
                % Loop over all overlapping nodes
                for j=1:N
                    if i~=j
                        [inCj1] = inpolygon( pt1(1), pt1(2), Cgs{j}(1,:), Cgs{j}(2,:) );
                        [inCj2] = inpolygon( pt2(1), pt2(2), Cgs{j}(1,:), Cgs{j}(2,:) );

                        if inCj1 && inCj2
                            % This is a dominant arc, normal vector with
                            % magnitude fi-fj

                            % The magnitude of the normal vector is the
                            % edge length
                            nvector = ROT(pt2-pt1, pi/2);
%--------------------------------------------------------------------------
                            q=ismember(pt1(1),C_maxz{j}(1,:))+ismember(pt1(2),C_maxz{j}(2,:));
%--------------------------------------------------------------------------

                            if q==2
                                uTH = uTH + (f(i)-f(j)) * dot(Jz_maxz(pt1), nvector);
                            else
                                uTH = uTH + (f(i)-f(j)) * dot(Jz_minz(pt1), nvector);
                            end

                        end

                        % If any of the points is inside a Cj, this is
                        % not a free arc
                        if inCj1 || inCj2
                            free_arc = 0;
                        end
                    end
                end % All other node for

                if free_arc
                    % This is a free arc, normal vector
                    % The magnitude of the normal vector is the
                    % edge length
                    nvector = ROT(pt2-pt1, pi/2);

%--------------------------------------------------------------------------
                    q=ismember(pt1(1),C_maxz{i}(1,:))+ismember(pt1(2),C_maxz{i}(2,:));
%--------------------------------------------------------------------------
                 
                    if q==2
                        uTH = uTH + f(i) * dot(Jz_maxz(pt1), nvector);
                    else
                        uTH = uTH + f(i) * dot(Jz_minz(pt1), nvector);
                    end
                end

            end
        end
    end % line segment for
end
