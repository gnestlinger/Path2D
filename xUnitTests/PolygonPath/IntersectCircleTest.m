classdef IntersectCircleTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        DoPlot = {false}
    end
    
    
    
    methods (Test)
        function testSegmentTransition(testCase, DoPlot)
        % Intersection with end of first/start of second path segment
            obj0 = PolygonPath.xy2Path([-2 0 2], [1 1 2]);
            [xy,tau] = obj0.intersectCircle([0 0], 1, DoPlot);
            testCase.verifyEqual(xy, [0 1]);
            testCase.verifyEqual(tau, 1);
        end%fcn
        
        function testTwoIntersectionsPerSegment(testCase, DoPlot)
        % Two intersections with first path segment
            obj0 = PolygonPath.xy2Path([-10 -2 0 4 10], [0 0 0 3 3]);
            [xy,tau] = obj0.intersectCircle([-5 0], 1, DoPlot);
            testCase.verifyEqual(xy, [-6 0; -4 0]);
            testCase.verifyEqual(tau, [0.5; 0.75]);
        end%fcn
        
        function testSingleIntersection(testCase, DoPlot)
        % One intersection with final path segment
            obj0 = PolygonPath.xy2Path([-10 -2 0 4 10], [0 0 0 3 3]);
            [xy,tau] = obj0.intersectCircle([11 3], 4, DoPlot);
            testCase.verifyEqual(xy, [7 3], 'AbsTol', 1e-14);
            testCase.verifyEqual(tau, 3.5);
        end%fcn
        
        function testNoIntersection(testCase, DoPlot)
        % No intersection with path
            obj0 = PolygonPath.xy2Path([-10 -2 0 4 10], [0 0 0 3 3]);
            [xy,tau] = obj0.intersectCircle([0 4], 2, DoPlot);
            testCase.verifyEqual(xy, zeros(0,2));
            testCase.verifyEqual(tau, zeros(0,1));
        end%fcn
    end
    
end%class
