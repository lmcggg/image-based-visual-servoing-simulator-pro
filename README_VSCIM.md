# Visual Servoing Control with Input Mapping (VSC-IM)

This repository includes an implementation of the VSC-IM method for image-based visual servoing simulation. The VSC-IM method leverages historical control inputs and their effects to improve convergence performance and robustness.

## Overview

VSC-IM (Visual Servoing Control - Input Mapping) is a novel approach to image-based visual servoing that addresses the limitations of conventional IBVS. Unlike traditional approaches, VSC-IM uses historical data to compensate for model uncertainties and dynamic variations by:

1. Maintaining a history of past error changes and control inputs
2. Adaptively combining past control inputs to improve current control performance 
3. Learning from previous control actions to reduce feature tracking errors

## Key Features

1. **Historical Data Utilization**: Employs a data-driven approach by using past control inputs and their effects to enhance performance.

2. **Adaptive Control Scheme**: Automatically adjusts control strategy based on observed error dynamics.

3. **Error Compensation**: Corrects for modeling errors and uncertainties through the history-based input mapping.

4. **Improved Convergence Rate**: Typically achieves faster error convergence compared to standard IBVS methods.

## Theory

The control law of VSC-IM consists of two main components:

1. **Original Feedback Term**: A standard feedback control based on the error between current and desired feature positions.

2. **Historical Data Compensation**: An adaptive term that combines past control inputs to improve current performance.

The complete control law is:

```
V_c(k) = -F(e(k) + Δe_vec(k)λ(k)) + V_c_vec(k)λ(k)
```

where:
- F is the feedback gain matrix
- e(k) is the current feature error
- Δe_vec(k) is the matrix of past error changes
- V_c_vec(k) is the matrix of past control inputs
- λ(k) is the optimized coefficient vector

The coefficient vector λ(k) is optimized to minimize the predicted next-step error:

```
λ(k) = -η(Δe_vec(k)^T Q Δe_vec(k))^+ Δe_vec(k)^T Q e(k)
```

where:
- η is the learning rate (between 0 and 1)
- Q is a positive definite weighting matrix
- (.)^+ denotes the pseudoinverse operation

## How to Use

1. Open `MAIN.m` in MATLAB.

2. Set the control method parameter:
   ```matlab
   % Control method selection
   % 0 = conventional IBVS
   % 1 = PPIBVS
   % 2 = Lambda-based IBVS (single lambda)
   % 3 = Lambda-based IBVS (sequential lambda)
   % 4 = VSC-IM (Visual Servoing Control with Input Mapping)
   control_method = 4;  % Set to 4 to use VSC-IM
   ```

3. Adjust the parameters for the VSC-IM method:
   ```matlab
   KVSCIM = 1.0;     % control gain for VSC-IM
   eta_VSCIM = 0.5;  % learning rate for input mapping (0 < eta < 1)
   ```

4. Run the simulation. Results will be automatically plotted and compared with other methods.

## Implementation Details

The implementation follows these key steps:

1. **Data Collection**: Maintain a sliding window of past error changes and control inputs.
2. **Lambda Optimization**: Compute the optimal coefficient vector to minimize predicted error.
3. **Control Synthesis**: Combine the feedback term with the historical data compensation.
4. **Velocity Saturation**: Apply constraints to ensure control inputs are within bounds.

## Advantages Over Other Methods

- No need for accurate depth information or camera calibration
- Robust to modeling errors and parameter uncertainties
- Adaptive to changing dynamics during servoing
- Improved convergence rate and trajectory smoothness
- Enhanced stability during large camera motions

## References

This implementation is based on the paper:
"Input Mapping-Enhanced Image-Based Visual Servoing for Dynamic Visual Tracking" 