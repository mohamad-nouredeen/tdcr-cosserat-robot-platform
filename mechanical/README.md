# Mechanical Design

This folder contains STL exports of mechanical components developed
for the tendon-driven continuum robot platform.

The files include the backbone disk, base structure, pulley supports,
tendon guides, actuator mounts, and coupling components.

## Files

The STL files are stored in [STL](STL/).
Original filenames have been preserved.

| File | Component description |
|---|---|
| `Disk_tendon.STL` | Disk identified by the author as used in the prototype |
| `BasePlatform.stl` | Base platform plate |
| `BasePlatform_wall.stl` | Base wall/support component |
| `BasePlatform_PullyHolder.stl` | Pulley holder |
| `BasePlatform_PulleyShaft.stl` | Pulley retaining shaft |
| `holder_tendon_horizontal.stl` | Horizontal tendon-guide support |
| `holder_tendon_vertical.stl` | Vertical tendon-guide support |
| `Tube_coppling_1mm.stl` | Backbone tube support/coupling |
| `ActuationUnit_EncoderCouplingSide.stl` | Encoder-side support component |
| `ActuationUnit_MotorSide_Cube.stl` | Motor-side support component |
| `Shaft_GearCoupling.stl` | Gear-coupling shaft component |
| `Shaft_MotorEncoder.stl` | Motor/encoder shaft component |
| `dynamixel/BASE1_DAYNAMIXEL.STL` | DYNAMIXEL mounting component |
| `dynamixel/Base_dynamixel.STL` | DYNAMIXEL mounting component |
| `dynamixel/coupling_connection.STL` | DYNAMIXEL coupling component |

Descriptions are based on filenames and geometry inspection.
Assembly positions and quantities remain to be documented.

The presence of motor, gear, and encoder components does not establish
an operational axial-rotation mechanism. Axial rotation was a proposed
future capability and was not actuated in the thesis experiments.

## Manufacturing

The thesis reports PETG 3D printing for the custom mechanical parts.

STL files do not encode units. The exports have been interpreted in
millimetres; check the imported scale before printing.

Print orientation, supports, tolerances, and assembly fit should be
checked for each component. Complete printing settings and a bill of
materials are not yet included.

## Disk geometry and model differences

`Disk_tendon.STL` is retained as supplied and identified by the author
as the disk used in the physical robot.

Inspection of this mesh, assuming millimetres, gives:

| Feature | STL geometry |
|---|---|
| Outside diameter | Approximately 20 mm |
| Thickness | Approximately 2 mm |
| Centre-to-tendon-hole distances | Approximately 7.00, 7.17, and 7.17 mm |
| Central backbone passage | Approximately 3 mm diameter |
| Tendon passages | Approximately 2.5 mm at the faces, narrowing to 1 mm internally |

These are mesh measurements, not measurements of the manufactured part.

The thesis and MATLAB models use a symmetric tendon-routing radius
of 6 mm. The thesis also describes a central disk passage tapering
from 3 mm to 1.5 mm. These differences remain unresolved and should
be considered when reproducing the model–prototype comparison.

The MATLAB parameters have been preserved to retain the reported
simulation configuration.

## Mesh inspection notes

The supplied exports have not been repaired:

- `Disk_tendon.STL` contains duplicate triangles and non-manifold edges.
- `ActuationUnit_MotorSide_Cube.stl` contains two non-manifold edges.
- `dynamixel/BASE1_DAYNAMIXEL.STL` contains small vertex seams that
  disappear when nearly coincident vertices are merged.

Inspect these meshes in CAD or slicing software before manufacturing.
Passing basic mesh checks does not establish dimensional accuracy
or assembly compatibility.

## Documentation status

This collection currently contains STL meshes. Editable CAD files,
assembly drawings, confirmed part quantities, and detailed assembly
instructions have not yet been added.

For the project overview, prototype photograph, and demonstration
video, see the [main README](../README.md).