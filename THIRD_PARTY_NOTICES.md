# Third-Party Notices

This repository contains original project material together with portions of MATLAB code adapted from previously published open-source research.

## TIMClab-CAMI Cosserat-Rod Modeling Code

Parts of the static Newtonian and Lagrangian Cosserat-rod implementations in the `modeling/` directory are adapted from the open-source project:

**Cosserat Rod Modeling of Tendon-Actuated Continuum Robots**

Original repository:  
https://github.com/TIMClab-CAMI/Cosserat-Rod-Modeling-of-Tendon-Actuated-Continuum-Robots

Associated publication:

Matthias Tummers et al.,  
*"Cosserat Rod Modeling of Continuum Robots from Newtonian and Lagrangian Perspectives,"*  
IEEE Transactions on Robotics, 2023.

DOI:  
https://doi.org/10.1109/TRO.2023.3238171

The original software is distributed under the **GNU General Public License version 3 (GPL-3.0)**.

The GPL-3.0 license text is included in the repository root as [`LICENSE`](LICENSE).

## Modifications in This Repository

The static modeling implementation was adapted for the tendon-driven continuum robot developed in this project.

Project-specific modifications include, where applicable:

- Adaptation of the robot geometry and backbone parameters
- Adaptation of material properties for the hollow Nitinol backbone
- Adaptation of the tendon-routing configuration
- Addition of project-specific simulation inputs and outputs
- Addition of comparison scripts
- Addition of the numerical displacement-to-tension procedure
- Integration with the broader TDCR modeling and experimental workflow

The modified files should not be interpreted as the original unmodified TIMClab-CAMI implementation.

## Other Project Material

The mechanical designs, experimental documentation, electronics documentation, prototype media, MATLAB actuation interface, thesis documentation, and project-specific integration presented in this repository were developed as part of the associated master's thesis project unless otherwise indicated within individual files.

## Citation

Users of the adapted Cosserat-rod implementation should cite the original Tummers et al. publication in addition to this repository.

Citation information for this repository is available in [`CITATION.cff`](CITATION.cff).
