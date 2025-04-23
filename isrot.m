function h = isrot(r)
    % ISROT Test if argument is a rotation matrix
    % Returns true (1) if r is a 3x3 rotation matrix,
    % else false (0).
    
    d = size(r);
    if ndims(r) >= 2
        h = all(d(1:2) == [3 3]);        % check dimensions
        if h && ndims(r) == 2            % check contents
            h = abs(det(r) - 1) < eps;    % check determinant
            h = h && all(all(abs(r*r' - eye(3)) < eps));   % check orthogonality
        end
    else
        h = false;
    end
end 