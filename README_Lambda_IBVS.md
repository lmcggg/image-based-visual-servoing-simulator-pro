# Lambda-based IBVS Method Implementation

This repository contains an implementation of a Lambda-based IBVS (Image-Based Visual Servoing) method with Broyden online Jacobian estimation and Min-Max NMPC (Nonlinear Model Predictive Control) for AUV control.

## Overview

The Lambda-based IBVS method has been added to the existing IBVS simulation framework. The new method uses the Broyden update rule to estimate the image Jacobian matrix online, and implements both single-lambda and sequential-lambda control strategies.

## Key Features

1. **Online Jacobian Estimation**: Uses the Broyden method to update the Jacobian matrix without requiring depth information.

2. **Min-Max NMPC Approach**: Handles uncertainties in the Jacobian estimation by considering the worst-case scenario.

3. **Lambda-based Parameterization**: Simplifies the optimization problem by parameterizing the control law with a scalar λ.

4. **Two Implementation Variants**:
   - **Single λ-Based**: Uses a constant λ value optimized for the entire control horizon.
   - **Sequential λ-Based**: Uses a sequence of λ values, one for each time step in the prediction horizon.

## How to Use

1. Open `MAIN.m` in MATLAB.

2. Set the control method parameter:
   ```matlab
   % Control method selection
   % 0 = conventional IBVS
   % 1 = PPIBVS
   % 2 = Lambda-based IBVS (single lambda)
   % 3 = Lambda-based IBVS (sequential lambda)
   control_method = 2;  % Change this value to select a method
   ```

3. Adjust the parameters for the Lambda-based IBVS method:
   ```matlab
   KLambda = 1;         % gain for Lambda-based IBVS
   alpha_broyden = 1;   % Broyden update parameter (0 < alpha < 2)
   ```

4. Run the simulation. Results will be plotted automatically.

## Implementation Details

The Lambda-based IBVS method is implemented in `LambdaIBVS.m`, which follows these steps:

1. Initialize or update the Jacobian matrix using the Broyden method.
2. Calculate the feature error.
3. Depending on the selected scenario:
   - For single lambda: Find the optimal λ that minimizes the worst-case cost.
   - For sequential lambda: Use a predefined sequence of λ values.
4. Apply the control law: `ν = -λ·J⁺·e`.
5. Apply velocity constraints.

## Comparison with Other Methods

The Lambda-based IBVS method offers the following advantages:

- No need for depth estimation, unlike conventional IBVS.
- Enhanced robustness against Jacobian estimation errors.
- Lower computational complexity compared to full NMPC implementations.
- Better convergence properties and constraint handling than standard IBVS.

## Plotting Functions

Specialized plotting functions are provided to visualize the performance:

- `plot_LambdaIBVS.m`: For both single and sequential lambda methods.
- `plot_IBVS_func.m`: For conventional IBVS.
- `plot_PPIBVS_func.m`: For PPIBVS.

These functions display feature errors, control inputs, camera trajectory, and performance metrics. 