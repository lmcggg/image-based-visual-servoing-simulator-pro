%TRPLOT Draw a coordinate frame
%
% TRPLOT(T, OPTIONS) draws a 3D coordinate frame represented by the homogeneous 
% transform T (4x4).
%
% H = TRPLOT(T, OPTIONS) as above but returns a handle.
%
% TRPLOT(H, T) moves the coordinate frame described by the handle H to
% the pose T (4x4).
%
% TRPLOT(R, OPTIONS) draws a 3D coordinate frame represented by the orthonormal
% rotation matrix R (3x3).
%
% H = TRPLOT(R, OPTIONS) as above but returns a handle.
%
% TRPLOT(H, R) moves the coordinate frame described by the handle H to
% the orientation R.
%
% Options::
% 'color',C          The color to draw the axes, MATLAB colorspec C
% 'noaxes'           Don't display axes on the plot
% 'axis',A           Set dimensions of the MATLAB axes to A=[xmin xmax ymin ymax zmin zmax]
% 'frame',F          The frame is named {F} and the subscript on the axis labels is F.
% 'text_opts', opt   A cell array of MATLAB text properties
% 'handle',H         Draw in the MATLAB axes specified by the axis handle H
% 'view',V           Set plot view parameters V=[az el] angles, or 'auto' 
%                    for view toward origin of coordinate frame
% 'arrow'            Use arrows rather than line segments for the axes
% 'width', w         Width of arrow tips (default 1)
% 'thick',t          Thickness of lines (default 0.5)
% '3d'               Plot in 3D using anaglyph graphics
% 'anaglyph',A       Specify anaglyph colors for '3d' as 2 characters for 
%                    left and right (default colors 'rc'):
%                     'r'   red
%                     'g'   green
%                     'b'   green
%                     'c'   cyan
%                     'm'   magenta
% 'dispar',D         Disparity for 3d display (default 0.1)
%
% Examples::
%
%       trplot(T, 'frame', 'A')
%       trplot(T, 'frame', 'A', 'color', 'b')
%       trplot(T1, 'frame', 'A', 'text_opts', {'FontSize', 10, 'FontWeight', 'bold'})
%
%       h = trplot(T, 'frame', 'A', 'color', 'b');
%       trplot(h, T2);
%
% 3D anaglyph plot
%       trplot(T, '3d');
%
% Notes::
% - The arrow option requires the third party package arrow3.
% - The handle H is an hgtransform object.
% - When using the form TRPLOT(H, ...) the axes are not rescaled.
% - The '3d' option requires that the plot is viewed with anaglyph glasses.
% - You cannot specify 'color' 
%
% See also TRPLOT2, TRANIMATE.


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

% TODO
%  need to decide how to handle scaling
%  what does hold on mean?  don't touch scaling?

function hout = trplot(T, varargin)
    if isscalar(T) && ishandle(T)
        % trplot(H, T)
        H = T; T = varargin{1};
        if isrot(T)
            T = r2t(T);
        end
        set(H, 'Matrix', T);
        return;
    end

    if size(T,3) > 1
        error('trplot cannot operate on a sequence');
    end
    if ~ishomog(T) && ~isrot(T)
        error('trplot operates only on transform (4x4) or rotation matrix (3x3)');
    end
    
    % parse options
    if nargin < 2
        varargin = {};
    end
    opt.color = 'b';
    opt.frame = [];
    opt.text_opts = {};
    opt.width = 1;
    opt.arrow = false;
    opt.thick = 0.5;
    
    % get the color
    for i = 1:2:length(varargin)
        if strcmpi(varargin{i}, 'color')
            opt.color = varargin{i+1};
        end
    end
    
    if isrot(T)
        T = r2t(T);
    end
    
    % create unit vectors
    o =  [0 0 0]';
    x1 = [1 0 0]';
    y1 = [0 1 0]';
    z1 = [0 0 1]';
    
    % draw the axes
    mstart = o';
    mend = [x1 y1 z1]';
    
    for i=1:3
        plot3([mstart(1) mend(i,1)], [mstart(2) mend(i,2)], [mstart(3) mend(i,3)], ...
            'Color', opt.color, 'LineWidth', opt.thick);
        hold on;
    end
    
    % label the axes
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    
    if nargout > 0
        hout = gca;
    end
end

function out = ag_color(c)

% map color character to an color triple, same as anaglyph.m

    % map single letter color codes to image planes
    switch c
    case 'r'
        out = [1 0 0];        % red
    case 'g'
        out = [0 1 0];        % green
    case 'b'
        % blue
        out = [0 0 1];
    case 'c'
        out = [0 1 1];        % cyan
    case 'm'
        out = [1 0 1];        % magenta
    case 'o'
        out = [1 1 0];        % orange
    end
end
