%% define vertices and faces of camera %%
function plotcamera(x)
    % x = [x, y, z, rx, ry, rz] - camera position and orientation
    Rmax=0;
    a_fov=0;
    b_fov=0;
    
    % Create rotation matrix for camera frame
    rrr=rotoz(-pi/2)*rotoy(0)*rotox(-pi/2);
    rr=[rrr(1,1:3),0;
        rrr(2,1:3),0;
        rrr(3,1:3),0;
        0,0,0,1];
    
    % Create homogeneous transform for camera pose
    Tic = transl(x(1),x(2),x(3))*rpy2tr(x(4),x(5),x(6),'zyx');
    pc_i = transl(Tic);
    Ric = t2r(Tic);
    
    % Define camera vertices
    pc1 = (pc_i + Ric*[    0;-0.04;-0.03])';
    pc2 = (pc_i + Ric*[    0; 0.04;-0.03])';
    pc3 = (pc_i + Ric*[    0; 0.04; 0.03])';
    pc4 = (pc_i + Ric*[    0;-0.04; 0.03])';
    pc5 = (pc_i + Ric*[-0.04;-0.02;-0.02])';
    pc6 = (pc_i + Ric*[-0.04; 0.02;-0.02])';
    pc7 = (pc_i + Ric*[-0.04; 0.02; 0.02])';
    pc8 = (pc_i + Ric*[-0.04;-0.02; 0.02])';
    
    % Define camera faces
    vm_c1 = [pc1; pc2; pc3; pc4; pc5; pc6; pc7; pc8];
    fm_c1 = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    
    % Define lens vertices
    pc9  = (pc_i + Ric*[-0.12;-0.02;-0.02])';
    pc10 = (pc_i + Ric*[-0.12; 0.02;-0.02])';
    pc11 = (pc_i + Ric*[-0.12; 0.02; 0.02])';
    pc12 = (pc_i + Ric*[-0.12;-0.02; 0.02])';
    
    vm_c2 = [pc5; pc6; pc7; pc8; pc9; pc10; pc11; pc12];
    fm_c2 = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    
    % Plot camera frame
    trplot(Tic*rr);
    
    % Plot camera body
    patch('Vertices', vm_c1, 'Faces', fm_c1, 'FaceColor', 'r', 'FaceAlpha', 0.2);
    patch('Vertices', vm_c2, 'Faces', fm_c2, 'FaceColor', 'r', 'FaceAlpha', 0.2);

    %% define the 8 vertices and 6 faces of fov of camera %%

    pc13 = pc_i';
    pc14 = pc_i';
    pc15 = pc_i';
    pc16 = pc_i';
    pc17 = (pc_i + Ric*[Rmax;-Rmax*tan(a_fov/2);-Rmax*tan(b_fov/2)])';
    pc18 = (pc_i + Ric*[Rmax; Rmax*tan(a_fov/2);-Rmax*tan(b_fov/2)])';
    pc19 = (pc_i + Ric*[Rmax; Rmax*tan(a_fov/2); Rmax*tan(b_fov/2)])';
    pc20 = (pc_i + Ric*[Rmax;-Rmax*tan(a_fov/2); Rmax*tan(b_fov/2)])';

    vm_fov = [ pc13 ; pc14 ; pc15 ; pc16 ; pc17 ; pc18 ; pc19 ; pc20 ];   % define the 8 vertices at which the faces meet

    fm_fov = [1 2 6 5 ; 2 3 7 6 ; 3 4 8 7 ; 4 1 5 8 ; 1 2 3 4 ; 5 6 7 8];   % define the six faces

    % patch('Vertices', vm_fov, 'Faces', fm_fov, 'FaceColor', 'y', 'FaceAlpha', 0.1);
end
