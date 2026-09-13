# DYNAMIXEL Tendon Actuation

This folder contains the MATLAB interface used to convert prescribed
nominal tendon pulls into DYNAMIXEL RX-28 Goal Position commands for the
three-tendon continuum robot.

The physical actuation system consists of three ROBOTIS DYNAMIXEL RX-28
actuators, one for each tendon.

## Main File

| File | Purpose |
|---|---|
| `TDCR_MotorPullCommand.m` | Converts nominal tendon-pull commands into DYNAMIXEL Goal Position commands and sends them to the three RX-28 actuators |

## Actuation Architecture

The high-level actuation sequence is:

```text
Nominal tendon pull [mm]
        |
        v
MATLAB R2025a
        |
        v
Convert tendon pull to spool rotation
        |
        v
Convert spool rotation to DYNAMIXEL Goal Position
        |
        v
ROBOTIS U2D2
        |
        v
U2D2 Power Hub
        |
        v
RX-28 actuators