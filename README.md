# Tendon-Driven Continuum Robot Platform

A single-segment tendon-driven continuum robot developed during my
master’s thesis internship at ISIR, Sorbonne Université, as part of
the MSc in Mechatronics Engineering at Politecnico di Torino.

This project connects static and dynamic Cosserat-rod modeling,
mechanical design, electronics, tendon actuation, and preliminary
experimental evaluation.

![Complete tendon-driven continuum robot prototype](media/prototype-overview.jpg)
## Robot demonstration

Physical prototype during tendon actuation.



https://github.com/user-attachments/assets/f8559108-5f53-4d4b-8bde-74a4fe4d064c


## Prototype

The robot uses:

- A 250 mm hollow Nitinol backbone.
- Eleven spacer disks.
- Three nylon tendons arranged at 120° intervals.
- A tendon-routing radius of 6 mm.
- Three DYNAMIXEL RX-28 actuators.
- Motor-driven spools with a nominal effective radius of 2.5 mm.
- A MATLAB actuation interface using the DYNAMIXEL SDK.

## Modeling approach

### Static models

Two Cosserat-rod formulations predict the backbone configuration
under prescribed tendon tensions:

- Newtonian formulation: force and moment equilibrium, integrated
  using ode45 and solved through a shooting method with fsolve.
- Lagrangian formulation: strain-basis approximation and
  Newton–Raphson solution of the static equilibrium equations.

The formulations produced practically identical tip positions for
the loading cases investigated in the thesis.

### Displacement-to-tension iteration

An outer PD-based numerical iteration adjusts tendon tension until
the Newtonian model reproduces a prescribed tendon shortening.

This is an offline numerical procedure. It is separate from the
physical motor command interface.

### Dynamic model

The dynamic formulation includes backbone inertia, internal damping,
gravity, quadratic drag, and prescribed time-dependent tendon tensions.
It uses implicit BDF-alpha time discretization and spatial shooting.

Dynamic results are numerical predictions and have not been
experimentally validated.

## Actuation

The MATLAB motor interface converts nominal tendon pulls in
millimetres into RX-28 Goal Position commands.

Actual tendon displacement and tension are not directly measured.
Transmission effects such as friction, slack, and tendon elongation
can therefore cause the physical motion to differ from the command.

## Preliminary experimental results

Static evaluation comprised one unloaded condition and six
single-active-tendon tests using nominal pulls of 5 mm and 10 mm.

Across the six loaded cases:

| Metric | Value |
|---|---:|
| Mean absolute vertical-position error | 11.54 mm |
| Root-mean-square vertical-position error | 14.01 mm |
| Maximum absolute vertical-position error | 21.08 mm |

The model captured the bending trend but predicted greater bending
than observed, particularly for the 10 mm commands.

Validation was limited to manually measured vertical tip coordinates,
with one measurement per operating point.

## Repository contents

| Location | Contents |
|---|---|
| `modeling/` | Static models, dynamic model, PD iteration, comparison scripts, and supporting functions |
| `actuation/` | MATLAB commands for the DYNAMIXEL RX-28 actuators |
| `LICENSE` | GPL-3.0 license text |

Mechanical files, electronics documentation, experimental data,
photographs, and videos will be added progressively.

## Software

The thesis implementations used MATLAB R2025a.

The Newtonian and dynamic solvers use fsolve from Optimization Toolbox.
The motor interface additionally requires the ROBOTIS DYNAMIXEL SDK
and configuration for the connected hardware.

The repository version has not yet been independently runtime-tested.

## Attribution

The static Newtonian and Lagrangian implementations are adapted from
the TIMClab-CAMI code associated with:

Matthias Tummers et al. (2023),
“Cosserat Rod Modeling of Continuum Robots from Newtonian and
Lagrangian Perspectives,” IEEE Transactions on Robotics.

DOI: https://doi.org/10.1109/TRO.2023.3238171

Original code:
https://github.com/TIMClab-CAMI/Cosserat-Rod-Modeling-of-Tendon-Actuated-Continuum-Robots

The adaptations include the prototype geometry and hollow-backbone
material properties, additional numerical outputs, and a
displacement-to-tension iteration. See LICENSE for the included
GPL-3.0 terms.

The dynamic formulation follows the research references documented
in the thesis.

## Author

Mohamad Nouredeen
