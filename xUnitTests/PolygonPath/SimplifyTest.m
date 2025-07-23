classdef SimplifyTest < matlab.unittest.TestCase
    
    methods (Test)
        function testLine(testCase)
        % Test simplification of a straight path.
            
            % Create a line with intermediate points
            obj0 = PolygonPath.xy2Path(0:10, 0:10);
            
            obj1 = obj0.simplify();
            
            % The terminal points must match
            [exp0,exp1] = obj0.termPoints();
            [act0,act1] = obj1.termPoints();
            verifyEqual(testCase, act0, exp0);
            verifyEqual(testCase, act1, exp1);
            
            % The length must match
            verifyEqual(testCase, obj1.length(), obj0.length())
            
            % Exactly two points, i.e. one segment, must survive
            verifyEqual(testCase, obj1.numel(), 1)
        end%fcn
        
        function testNothingToRemove(testCase)
        % Test path which cannot be simplified.
        
            % Create a path where each line segment contains only two
            % points -> no simplification possible
            obj0 = PolygonPath.xy2Path(...
                [0 1 1 2 2 3 3 4 4], ...
                [0 0 1 1 2 2 3 3 4]);
            
            verifyEqual(testCase, obj0, obj0.simplify())
        end%fcn
    end
    
end%class
