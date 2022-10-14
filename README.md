# Path2D

## Installation
Just download the repository and add the root folder to your MATLAB search path.

### Dependencies
Only a MATLAB license is required. No further toolboxes!


## Features
The base class `Path2D` defines methods to 
- perform path operations (e.g. cartesian to frenet, frenet to cartesian, intersection with a line/circle),
- modify (e.g. shift, rotate) and 
- visualize paths.

A class to work with polygonal paths is implemented in `PolygonPath.m`. It is implemented to suport code generation, i.e. can be used in Simulink too.  
A class to work with polynomial paths is under development (`SplinePath.m`).


## Documentation
To view the help of a specific class type `help <ClassName>`, e.g. the help of the base class `Path2D` is viewed by typing `help Path2D` in the command window and the help of the `PolygonPath` class is viewed by typing `help PolygonPath`.

## Examples
tbd

## Contribution
Contributers are welcome! Both, by submitting issues and creating pull requests.
