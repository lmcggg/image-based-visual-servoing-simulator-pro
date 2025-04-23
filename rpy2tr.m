%RPY2TR Roll-pitch-yaw angles to homogeneous transform
%
% T = RPY2TR(RPY, OPTIONS) is a homogeneous transformation equivalent to the 
% specified roll, pitch, yaw angles which correspond to rotations about the 
% X, Y, Z axes respectively. If RPY has multiple rows they are assumed to 
% represent a trajectory and T is a three dimensional matrix, where the last index  
% corresponds to the rows of RPY.
%
% T = RPY2TR(ROLL, PITCH, YAW, OPTIONS) as above but the roll-pitch-yaw angles 
% are passed as separate arguments. If ROLL, PITCH and YAW are column vectors 
% they are assumed to represent a trajectory and T is a three dimensional matrix,
% where the last index corresponds to the rows of ROLL, PITCH, YAW.
%
% Options::
%  'deg'   Compute angles in degrees (radians default)
%  'zyx'   Return solution for sequential rotations about Z, Y, X axes (Paul book)
%
% Note::
% - In previous releases (<8) the angles corresponded to rotations about ZYX. Many 
%   texts (Paul, Spong) use the rotation order ZYX. This old behaviour can be enabled 
%   by passing the option 'zyx'
%
% See also TR2RPY, RPY2R, EUL2TR.

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

function T = rpy2tr(roll, varargin)
    if nargin == 1
        % rpy2tr(RPY)
        rpy = roll;
    else
        % rpy2tr(r,p,y)
        rpy = [roll varargin{1} varargin{2}];
    end
    
    % Calculate rotation matrix
    cr = cos(rpy(1));
    sr = sin(rpy(1));
    cp = cos(rpy(2));
    sp = sin(rpy(2));
    cy = cos(rpy(3));
    sy = sin(rpy(3));
    
    % Create rotation matrix
    R = [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr;
         sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr;
         -sp,   cp*sr,          cp*cr];
    
    % Create homogeneous transformation matrix
    T = [R, zeros(3,1);
         0 0 0 1];
end
