# Cosserat-Rod Modeling

This folder contains the MATLAB implementation of the static and dynamic
Cosserat-rod models used for the single-segment tendon-driven continuum
robot (TDCR).

The modeling work includes:

- Newtonian static Cosserat-rod formulation
- Lagrangian static Cosserat-rod formulation
- Numerical comparison of the two static formulations
- Displacement-to-tension iteration
- Dynamic Cosserat-rod formulation
- Supporting mathematical and numerical functions

The implementations were developed and evaluated using MATLAB R2025a.

## Main Files

| File | Purpose |
|---|---|
| `Newtonian.m` | Newtonian force-and-moment equilibrium formulation for static TDCR modeling |
| `Lagrangian.m` | Lagrangian energy-based formulation for static TDCR modeling |
| `Newtonian_PD.m` | Newtonian model used with the displacement-to-tension numerical iteration |
| `TDCR_DynamicModel.m` | Dynamic Cosserat-rod implementation |
| `compare_four_cases.m` | Numerical comparison of selected Newtonian and Lagrangian static cases |
| `main_Lagrangian.m` | Entry script for running the Lagrangian static formulation |
| `run_PD_test.m` | Demonstration of the displacement-to-tension iteration |
| `materials.m` | Backbone material and constitutive parameters |
| `tendons.m` | Tendon geometry and routing parameters |

## Supporting Functions

### `Lagrangian_approach_subfunctions/`

Contains mathematical and numerical functions required by the
Lagrangian formulation, including strain-basis, quaternion,
Lie-algebra, integration, and plotting utilities.

### `tools/`

Contains shared numerical and post-processing utilities used by the
static modeling workflow.

These include functions for:

- Rotation and quaternion operations
- Numerical integration
- Result reading and saving
- Tendon-tension combinations
- Static-model visualization and comparison

## Static Newtonian Formulation

The Newtonian formulation describes the backbone through Cosserat-rod
kinematics together with distributed force and moment equilibrium.

The spatial differential equations are integrated using MATLAB `ode45`.
Unknown boundary quantities are determined through a shooting method
using `fsolve`.

For a prescribed tendon-tension vector, the model predicts the complete
backbone configuration and the resulting tip pose.

## Static Lagrangian Formulation

The Lagrangian formulation represents the distributed rod strains using
a finite set of basis functions and generalized coordinates.

The equilibrium configuration is obtained from the energy-based
formulation through a Newton-Raphson numerical solution.

The Newtonian and Lagrangian formulations produced practically identical
tip positions for the loading cases investigated in the thesis.

## Static Model Comparison

The script

```matlab
compare_four_cases