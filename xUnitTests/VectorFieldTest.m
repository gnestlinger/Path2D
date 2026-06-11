classdef VectorFieldTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        PathEmpty = struct(...
            'PolygonPath', PolygonPath(), ...
            'SplinePath', SplinePath())
        
        PathStraight = {
            PolygonPath.straight([-5 -2], [5 2]);
            SplinePath.straight([-5 -2], [5 2]);
            }
        
        PathCircle = {% {Path object, number of NaN's}
            {PolygonPath.circle(3, [0 2*pi], 1e3), 31};
            {SplinePath.circle(3, [0 2*pi], 11), 0};
            }
        
        xy = struct(...
            'Scalar',{{0 0}}, ...
            'Vector',{{-linspace(-5,5,10), linspace(-3,3,10)}}, ...
            'Matrix',{{meshgrid(-5:5), meshgrid(-5:5)'}})
        
        DoPlot = {false}
    end
    
    
    
    methods (Test)
        function testReturnSizeEmptyPath(testCase, PathEmpty, xy, DoPlot)
        % Test the size of return arguments for empty paths.
        
            [X,Y] = extractXY(xy);
            [Fx,Fy] = PathEmpty.vectorField(X, Y, 1, DoPlot);
            verifyEqual(testCase, Fx, NaN(size(X)));
            verifyEqual(testCase, Fy, NaN(size(Y)));
        end%fcn
        
        function testReturnSizeNonemptyPath(testCase, PathStraight, xy, DoPlot)
        % Test the size of return arguments for nonempty paths.
        
            [X,Y] = extractXY(xy);
            [Fx,Fy] = PathStraight.vectorField(X, Y, 1, DoPlot);
            
            verifySize(testCase, Fx, size(X));
            verifySize(testCase, Fy, size(Y));
        end%fcn
        
        function testStraightPath(testCase, PathStraight, DoPlot)
        % Given a straight path, test for "correct" return values by
        % checking the number of non-NaN values.
           
            [X,Y] = meshgrid(-5:5);
            [Fx,Fy] = PathStraight.vectorField(X, Y, 1, DoPlot);
            
            import matlab.unittest.constraints.HasNaN
            verifyThat(testCase, [Fx Fy], ~HasNaN)
        end%fcn
        
        function testCirclePath(testCase, PathCircle, DoPlot)
        % Given a circular path, test for "correct" return values by
        % checking the number of non-NaN values.
        
            [X,Y] = meshgrid(-5:0.5:5);
            PathObj = PathCircle{1};
            nbrNaNs = PathCircle{2};
            [Fx,Fy] = PathObj.vectorField(X, Y, 1, DoPlot);
            
            verifyEqual(testCase, sum(isnan([Fx(:); Fy(:)])), nbrNaNs*2)
%             import matlab.unittest.constraints.HasNaN
%             verifyThat(testCase, [Fx Fy], ~HasNaN)
        end%fcn
    end
    
end%class


function [x,y] = extractXY(xy)
x = xy{1};
y = xy{2};
end%fcn
