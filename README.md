[![MATLAB](https://github.com/gnestlinger/Path2D/actions/workflows/ci.yml/badge.svg)](https://github.com/gnestlinger/Path2D/actions/workflows/ci.yml)

# Path2D
Path2D provides methods for **creating, analyzing, and modifying 2D paths** in MATLAB – ideal for research, teaching, and applications such as robotics and autonomous vehicle guidance.

## Features

- **Path operations:** Orthogonal projection, cartesian ↔ frenet transformation, intersection with lines/circles
- **Path modification:** Shift, rotate, and combine paths
- **Visualization:** Plot and display paths and results

### Supported path types
- `PolygonPath.m`: Polygonal paths (defined by waypoints)
- `SplinePath.m`: Polynomial spline paths
- `DubinsPath.m`: Dubins paths (sequences of arcs and lines) – [work in progress](https://github.com/gnestlinger/Path2D/tree/DubinsPath)

Most functionality supports code generation and can be used in Simulink.

## Motivation
Two well-known path tracking controllers are the Pure Pursuit [1] and the Stanley controller [2]. Their underlying path tracking error model differs in terms of reference point (rear/front axle), look-ahead (in the direction of the path/none) and lateral error orientation (orthogonal to vehicle heading/path). In literature, you can find several additional path tracking error definitions. As a result, the implementation of a specific path tracking controller not only requires the implementation of the control itself, but also the implementation of the according error model.  
To generalize the computation of the path tracking error (and therefore the interface of the local path planner and the path tracking controller), [3] has proposed a classification of path tracking error definitions and also listed the required path operations (e.g. intersection of line/circle with path).
This repository implements these operations (and other such as frenet transformation) for different represenations of 2-dimensional paths.

## Quick Start
```matlab
% Example: Create a PolygonPath from waypoints and visualize the path
points = [0 0; 1 1; 2 0];
pathObj = PolygonPath.xy2Path(points(:,1), points(:,2));
pathObj.plot('r', 'LineWidth',2);

% Example: Create a SplinePath from a piecewise polynomial struct and visualize the path
pp = mkpp([0 1], [0 0 10 0; -6 9 0 0], 2);
pathObj = SplinePath.pp2Path(pp);
pathObj.plot('r', 'LineWidth',2)
```

Further examples are available in the test files and [below](#examples).
Check [GitHub Discussions](https://github.com/gnestlinger/Path2D/discussions) for questions & exchange.

## Requirements
- Just MATLAB – no additional toolboxes required.

## Installation
1. Download or clone the repository.
2. Add the root folder to your MATLAB search path or add the project file `Path2D.prj` as a [Project Reference](https://de.mathworks.com/help/simulink/ug/add-or-remove-a-reference-to-another-project.html) to your project.

## Documentation
For class-specific help:
```matlab
help Path2D      % Base class
help PolygonPath % Polygonal path
help SplinePath  % Spline path
```

## Examples
See the [examples] (https://github.com/gnestlinger/Path2D/tree/main/examples) folder.

### Intersection of spline path with a circle
<img src="https://user-images.githubusercontent.com/84226458/233801288-a0665561-353f-4edc-a8e8-89793ea8414b.svg" width="500">

### Orthogonal projection w.r.t. to spline path
<img src="https://user-images.githubusercontent.com/84226458/233801353-1c4cc9b6-3151-44bf-9b33-15054bf7119f.svg" width="500">

### Vector field to guide a robot towards a reference path [4]
<img src="https://github.com/user-attachments/assets/170b6913-2a0b-47f7-84a7-8d4d5e9210a7" width="500">


## Contributing
Contributions are welcome – whether through **issues**, **pull requests**, or feedback!
- Use issues for feature requests or bug reports.

## References
[1] R. Craig Coulter. "Implementation of the Pure Pursuit Path Tracking Algorithm", 1992, Carnegie Mellon University.  
[2] G. M. Hoffmann, C. J. Tomlin, M. Montemerlo and S. Thrun. "Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing", 2007 American Control Conference, New York, NY, USA, 2007, pp. 2296-2301, https://doi.org/10.1109/ACC.2007.4282788  
[3] Rumetshofer, J.; Stolz, M.; Watzenig, D. "A Generic Interface Enabling Combinations of State-of-the-Art Path Planning and Tracking Algorithms", Electronics 2021, 10, 788. https://doi.org/10.3390/electronics10070788  
[4] A. M. C. Rezende, V. M. Goncalves and L. C. A. Pimenta, "Constructive Time-Varying Vector Fields for Robot Navigation", in IEEE Transactions on Robotics, vol. 38, no. 2, pp. 852-867, April 2022, doi: https://doi.org/10.1109/TRO.2021.3093674  
