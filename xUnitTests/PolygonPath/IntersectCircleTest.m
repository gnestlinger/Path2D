classdef IntersectCircleTest < matlab.unittest.TestCase
    
    methods (Test)
        function testIntersectCircle(testCase)
            doPlot = false;
            
            % Intersection with end of first/start of second path segment
            obj0 = PolygonPath.xy2Path([-2 0 2], [1 1 2]);
            [act,s] = intersectCircle(obj0, [0 0], 1, doPlot);
            exp = [0 1];
            testCase.verifyEqual(act, exp);
            
            % Two intersections with first path segment
            obj0 = PolygonPath.xy2Path([-10 -2 0 4 10], [0 0 0 3 3]);
            [act,s] = intersectCircle(obj0, [-5 0], 1, doPlot);
            exp = [-6 0; -4 0];
            testCase.verifyEqual(act, exp);
            
            % One intersection with final path segment
            [act,s] = intersectCircle(obj0, [11 3], 2, doPlot);
            exp = [9 3];
            testCase.verifyEqual(act, exp);
            
            % No intersection with path
            [act,s] = intersectCircle(obj0, [0 4], 2, true);
            exp = zeros(0,2);
            testCase.verifyEqual(act, exp);
            testCase.verifyEqual(size(act,1), size(s,1));
            
        end%fcn
    end
    
end%class
