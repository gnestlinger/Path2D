classdef IntersectLineTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        Offset = {0 1e1 1e2 1e3 1e4 1e5}
    end
    
    
    
    methods (Test)
        function testNoIntersection(testCase)
            
            obj0 = PolygonPath.xy2Path([0 1], [0 0]);
            [act,tau] = intersectLine(obj0, [0 4], 2, false);
            exp = zeros(0,2);
            verifyEqual(testCase, act, exp);
            verifyEqual(testCase, tau, zeros(0,1));
        end%fcn
        
        function testSingleIntersection(testCase)
            
            obj0 = PolygonPath.xy2Path([-10 -2 0 4 10], [0 0 0 3 3]);
            [act,tau] = intersectLine(obj0, [5 0], pi/2, false);
            exp = [5 3];
            verifyEqual(testCase, act, exp);
            verifyEqual(testCase, tau, 3+1/6);
        end%fcn
        
        function testMultipleIntersections(testCase)
            
            obj0 = PolygonPath.xy2Path([0 5 8 8 5 0], [0 0 0 2 2 2]);
            [act,tau] = intersectLine(obj0, [6 0], pi/2, false);
            exp = [6 0; 6 2];
            verifyEqual(testCase, act, exp, 'AbsTol',1e-12);
            verifyEqual(testCase, tau, [1+1/3;3+2/3]);
            
        end%fcn
        
        function testIntersectionInitPoint(testCase, Offset)
            
            obj0 = PolygonPath.xy2Path([0 2 10] + Offset, [0 0 1] + Offset);
            P0 = obj0.termPoints();
            
            % Intersection with initial point
            [act,tau] = intersectLine(obj0, [2 -2] + Offset, 3*pi/4, false);
            exp = P0';
            verifyEqual(testCase, act, exp, 'AbsTol',1e-12);
            verifyEqual(testCase, tau, 0, 'AbsTol',1e-15);
        end
        
        function testIntersectionEndPoint(testCase, Offset)
            
            obj0 = PolygonPath.xy2Path([0 2 10] + Offset, [0 0 1] + Offset);
            [~,P1] = obj0.termPoints();
            
            % Intersection with end point
            [act,tau] = intersectLine(obj0, [10 0] + Offset, pi/2, false);
            exp = P1';
            verifyEqual(testCase, act, exp, 'AbsTol',1e-12);
            verifyEqual(testCase, tau, 2);
        end%fcn
        
        function testIntersectionWithWaypoint(testCase, Offset)
        % Test the intersection of a line with a non-terminal waypoint of
        % the path. This situation can cause redundant solutions, i.e. end
        % of segment k and start of segmen i+1.
        
            obj0 = PolygonPath.xy2Path([0 1 2] + Offset, [0 1 2] + Offset);
        
            [act,tau] = intersectLine(obj0, [2 0] + Offset, 3*pi/4, false);
            
            verifyEqual(testCase, act, [1 1] + Offset, 'AbsTol',1e-12);
            verifyEqual(testCase, tau, 1);
        end%fcn
    end
    
end%class
