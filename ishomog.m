function h = ishomog(tr)
    % ISHOMOG Test if argument is a homogeneous transformation matrix
    % Returns true (1) if tr is a 4x4 homogeneous transformation matrix,
    % else false (0).
    
    d = size(tr);
    if ndims(tr) >= 2
        h = all(d(1:2) == [4 4]);          % check dimensions
        if h && ndims(tr) == 2             % check contents
            h = abs(det(tr(1:3,1:3)) - 1) < eps;     % check rotation matrix
            h = h && all(abs(tr(4,:) - [0 0 0 1]) < eps);  % check bottom row
        end
    else
        h = false;
    end
end 