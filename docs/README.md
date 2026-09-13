# Thesis Documentation

This folder contains the final master's thesis associated with the
tendon-driven continuum robot platform developed at ISIR, Sorbonne
Université.

## Master's Thesis

**Title:**  
Static and Dynamic Cosserat-Rod Modeling of a Single-Segment
Tendon-Driven Continuum Robot

**Author:**  
Mohamad Nour Edeen

**Degree:**  
Master's Degree in Mechatronic Engineering

**University:**  
Politecnico di Torino

**Research host:**  
ISIR, Sorbonne Université

The complete thesis is available here:

[Master_Thesis_Mohamad_Nouredeen.pdf](Master_Thesis_Mohamad_Nouredeen.pdf)

## Thesis Overview

The thesis develops an integrated modeling and experimental framework
for a single-segment tendon-driven continuum robot (TDCR).

The work combines:

- Mechanical design and prototype development
- Static Cosserat-rod modeling
- Newtonian and Lagrangian formulations
- Numerical displacement-to-tension estimation
- Dynamic Cosserat-rod modeling
- DYNAMIXEL RX-28 tendon actuation
- MATLAB-based system integration
- Preliminary experimental evaluation

The work applies and adapts established Cosserat-rod formulations to
the geometry and physical parameters of the developed robot rather than
proposing a new fundamental rod theory.

## Thesis Structure

The thesis is organized into eight chapters.

### Chapter 1 — Introduction

Introduces the motivation, problem statement, objectives, scope,
methodology, and principal contributions of the work.

### Chapter 2 — Continuum Robotics: Background and State of the Art

Reviews continuum-robot architectures, tendon-driven continuum robot
design principles, backbone-modeling approaches, and relevant control
strategies.

### Chapter 3 — Design and Development of the TDCR Platform

Presents the mechanical design, SolidWorks development, material
selection, additive manufacturing, assembly, and final TDCR prototype.

### Chapter 4 — Static Modeling Based on Cosserat-Rod Theory

Develops and compares the Newtonian and Lagrangian static Cosserat-rod
formulations.

The chapter also presents the numerical displacement-to-tension
procedure used to relate prescribed tendon shortening to the tendon
loads required by the static model.

### Chapter 5 — Dynamic Modeling of the TDCR

Introduces the dynamic Cosserat-rod formulation and the numerical
simulation of the time-dependent backbone response under prescribed
tendon loading.

### Chapter 6 — Actuation and System Integration

Describes actuator selection, tendon transmission, DYNAMIXEL RX-28
communication, power and communication hardware, and the MATLAB-based
actuation interface.

### Chapter 7 — Experimental Evaluation and Discussion

Presents the preliminary single-active-tendon experimental evaluation
and compares the static Newtonian model with the physical prototype
under equivalent nominal tendon-pull commands.

The chapter also discusses the main causes of discrepancy between the
model and the experimental robot.

### Chapter 8 — Conclusions and Future Work

Summarizes the principal outcomes and limitations of the project and
discusses future developments including parameter calibration, sensing,
dynamic experimental validation, and closed-loop control.

## Repository Relationship

The thesis provides the complete scientific and engineering description
of the project. The corresponding implementation files are organized
throughout this repository.

- [`../modeling/`](../modeling/)  
  Static Newtonian and Lagrangian models, dynamic model,
  displacement-to-tension procedure, and supporting MATLAB functions.

- [`../actuation/`](../actuation/)  
  MATLAB interface for commanding the DYNAMIXEL RX-28 actuators.

- [`../mechanical/`](../mechanical/)  
  Mechanical-component documentation and STL files.

- [`../electronics/`](../electronics/)  
  Communication, power, and actuator hardware architecture.

- [`../experiments/`](../experiments/)  
  Preliminary experimental comparison, measurements, figures, and
  validation results.

- [`../media/`](../media/)  
  Prototype photographs and demonstration media.

## Important Reproducibility Note

The repository contains the implementation and design files associated
with the thesis, but the physical prototype includes manufacturing,
assembly, transmission, and material effects that are not fully
represented by the mathematical models.

In particular, tendon friction, slack, elongation, spool winding,
manufacturing tolerances, and uncertainty in actual tendon displacement
can influence the physical robot response.

The experimental comparison should therefore be interpreted as a
preliminary evaluation of the model–prototype relationship rather than
a complete calibrated validation of the system.

## Project Overview

For a concise introduction to the complete project, see the
[main repository README](../README.md).