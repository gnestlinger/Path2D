[![MATLAB](https://github.com/gnestlinger/Path2D/actions/workflows/ci.yml/badge.svg)](https://github.com/gnestlinger/Path2D/actions/workflows/ci.yml)

# Path2D

Provides methods to 
- perform path operations (e.g. orthogonal projection, cartesian to frenet, frenet to cartesian, intersection with a line/circle),
- modify (e.g. shift, rotate) and 
- visualize paths.

These path types are available:
- `PolygonPath.m` implements a class to work with a polygonal path representation, i.e. a path defined by waypoints. 
- `SplinePath.m` implements a class to work with a spline based path representation.
Both classes are implemented to support code generation, i.e. can be used in Simulink.  

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
