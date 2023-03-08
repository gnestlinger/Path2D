[![MATLAB](https://github.com/gnestlinger/Path2D/actions/workflows/ci.yml/badge.svg)](https://github.com/gnestlinger/Path2D/actions/workflows/ci.yml)

# Path2D

## Features
The base class `Path2D` defines methods to 
- perform path operations (e.g. cartesian to frenet, frenet to cartesian, intersection with a line/circle),
- modify (e.g. shift, rotate) and 
- visualize paths.

These path types are available:
- `PolygonPath.m` implements a class to work with a polygonal path representation, i.e. a path defined by waypoints. 
- `SplinePath.m` implements a class to work with a spline based path representation.
Both classes are implemented to support code generation, i.e. can be used in Simulink too.  

## Requirements
- Just MATLAB. No additional toolboxes!

## Installation
1. Download or clone the repository
2. Add the root folder to your MATLAB search path or add the project file `Path2D.prj` as a [Project Reference](https://de.mathworks.com/help/simulink/ug/add-or-remove-a-reference-to-another-project.html) to your project.

## Documentation
To view the help of a specific class type `help <ClassName>`, e.g. the help of the base class `Path2D` is viewed by typing `help Path2D` in the command window and the help of the `PolygonPath` class is viewed by typing `help PolygonPath`.

## Examples
Some examples can be found in the test files.
tbd

## Contribution
Contributers are welcome! Both, by submitting issues and creating pull requests.
