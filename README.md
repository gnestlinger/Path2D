[![MATLAB](https://github.com/gnestlinger/Path2D/actions/workflows/ci.yml/badge.svg)](https://github.com/gnestlinger/Path2D/actions/workflows/ci.yml)

# Path2D

Provides methods to 
- perform path operations (e.g. orthogonal projection, cartesian to frenet, frenet to cartesian, intersection with a line/circle),
- modify (e.g. shift, rotate) and 
- visualize paths.

These path types are available:
- `PolygonPath.m` implements a class to work with a polygonal path representation, i.e. a path defined by waypoints. 
- `SplinePath.m` implements a class to work with a polynomial spline path representation.
- `DubinsPath.m` models a path as a sequence of Dubins segments, i.e. fixed radius arcs and straight lines. (Under development; check branch "DubinsPath"!)
The implementation aims to support code generation, i.e. can be used in Simulink.  

## Motivation
Two well-known path tracking controllers are the Pure Pursuit [1] and the Stanley controller [2]. Their underlying path tracking error model differs in terms of reference point (rear/front axle), look-ahead (in the direction of the path/none) and lateral error orientation (orthogonal to vehicle heading/path). In literature, you can find several additional path tracking error definitions. As a result, the implementation of a specific path tracking controller not only requires the implementation of the control itself, but also the implementation of the according error model.  
To generalize the computation of the path tracking error (and therefore the interface of the local path planner and the path tracking controller), [3] has proposed a classification of path tracking error definitions and also listed the required path operations (e.g. intersection of line/circle with path).
This repository implements these operations (and other such as frenet transformation) for different represenations of 2-dimensional paths.

## Requirements
- Just MATLAB. No additional toolboxes!

## Installation
1. Download or clone the repository.
2. Add the root folder to your MATLAB search path or add the project file `Path2D.prj` as a [Project Reference](https://de.mathworks.com/help/simulink/ug/add-or-remove-a-reference-to-another-project.html) to your project.

## Documentation
To view the help of a specific class type `help <ClassName>`, e.g. the help of the base class `Path2D` is viewed by typing `help Path2D` in the command window and the help of the `PolygonPath` class is viewed by typing `help PolygonPath`.

## Examples
### Intersection of spline path with a circle

![Spline_CircleIntersection](https://user-images.githubusercontent.com/84226458/233801288-a0665561-353f-4edc-a8e8-89793ea8414b.svg)

### Orthogonal projection w.r.t. to spline path

![Spline_Cart2Frenet](https://user-images.githubusercontent.com/84226458/233801353-1c4cc9b6-3151-44bf-9b33-15054bf7119f.svg)

Further examples can be found in the test files.

## Contribution
Contributers are welcome! Both, by submitting issues and creating pull requests.

## References
[1] R. Craig Coulter, Implementation of the Pure Pursuit Path Tracking Algorithm, 1992, Carnegie Mellon University.  
[2] G. M. Hoffmann, C. J. Tomlin, M. Montemerlo and S. Thrun, "Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing," 2007 American Control Conference, New York, NY, USA, 2007, pp. 2296-2301, https://doi.org/10.1109/ACC.2007.4282788.  
[3] Rumetshofer, J.; Stolz, M.; Watzenig, D. A Generic Interface Enabling Combinations of State-of-the-Art Path Planning and Tracking Algorithms. Electronics 2021, 10, 788. https://doi.org/10.3390/electronics10070788 
