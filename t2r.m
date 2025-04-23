function R = t2r(T)
    % T2R Extract rotation matrix from homogeneous transform
    % Returns the 3x3 rotation matrix from a 4x4 homogeneous transformation matrix
    
    if ~ishomog(T)
        error('Input must be a 4x4 homogeneous transformation matrix');
    end
    
    R = T(1:3, 1:3);
end 