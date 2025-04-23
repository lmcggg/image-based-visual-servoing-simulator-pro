function T = r2t(R)
    % R2T Convert rotation matrix to homogeneous transform
    % Returns a 4x4 homogeneous transformation matrix from a 3x3 rotation matrix
    
    if size(R,1) ~= 3 || size(R,2) ~= 3
        error('R must be a 3x3 matrix');
    end
    
    T = [R zeros(3,1); 0 0 0 1];
end 