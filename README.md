# Image-Based-Visual-Servoing-Simulator

A environment built in Matlab for image Based Visual Servoing (IBVS) simulation studies. This simulator includes several IBVS methods:

1. Conventional IBVS
2. Prescribed Performance IBVS (PP-IBVS)
3. Lambda-based IBVS (single and sequential lambda variants)
4. Visual Servoing Control with Input Mapping (VSC-IM)

## How to Use:
For using just run the MAIN.m function and follow the steps described inside.

To select a specific control method, modify the `control_method` parameter in MAIN.m:
```matlab
% Control method selection
% 0 = conventional IBVS
% 1 = PPIBVS
% 2 = Lambda-based IBVS (single lambda)
% 3 = Lambda-based IBVS (sequential lambda)
% 4 = VSC-IM (Visual Servoing Control with Input Mapping)
control_method = 4;  % Change this value to select a method
```

## Implemented Methods:

### Conventional IBVS
The standard Image-Based Visual Servoing approach using image Jacobian.

### PP-IBVS
The Prescribed Performance Image Based Visual Servoing controller proposed in "Prescribed performance image based visual servoing under field of view constraints", Shahab Heshmati-alamdari, Charalampos P Bechlioulis, Minas V Liarokapis, Kostas J Kyriakopoulos, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2014.

### Lambda-based IBVS
An implementation of Lambda-based IBVS with Broyden online Jacobian estimation and Min-Max NMPC optimization.

### VSC-IM (Visual Servoing Control with Input Mapping)
A novel approach that leverages historical data to improve visual servoing performance. VSC-IM uses past control inputs and their effects to adaptively optimize current control actions, resulting in improved convergence and robustness. For more details, see the [VSC-IM README](README_VSCIM.md).

## Licence and Citing: 
If you use Image-Based-Visual-Servoing-Simulator in work that leads to a publication, I would appreciate it if you would kindly cite relative paper in your manuscript. Please cite the paper:


''Prescribed performance image based visual servoing under field of view constraints'', Shahab Heshmati-alamdari, Charalampos P Bechlioulis, Minas V Liarokapis, Kostas J Kyriakopoulos IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2014.

Bibtex:
@CONFERENCE{Heshmati-Alamdari20142721,
author={Heshmati-Alamdari, S. and Bechlioulis, C.P. and Liarokapis, M.V. and Kyriakopoulos, K.J.},
title={Prescribed performance image based visual servoing under field of view constraints},
journal={IEEE International Conference on Intelligent Robots and Systems},
year={2014},
pages={2721-2726},
doi={10.1109/IROS.2014.6942934}
}


