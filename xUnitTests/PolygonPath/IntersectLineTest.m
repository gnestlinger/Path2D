classdef IntersectLineTest < matlab.unittest.TestCase
    
    methods (Test)
        function testNoIntersection(testCase)
            
            obj0 = PolygonPath.xy2Path([0 1], [0 0]);
            [act,s] = intersectLine(obj0, [0 4], 2, false);
            exp = zeros(0,2);
            testCase.verifyEqual(act, exp);
            testCase.verifyEqual(s, zeros(0,1));
            
        end%fcn
        
        function testSingleIntersection(testCase)
            
            obj0 = PolygonPath.xy2Path([-10 -2 0 4 10], [0 0 0 3 3]);
            [act,s] = intersectLine(obj0, [5 0], pi/2, false);
            exp = [5 3];
            testCase.verifyEqual(act, exp);
            testCase.verifyEqual(size(s), [1,1]);
            
        end%fcn
        
        function testMultipleIntersections(testCase)
            
            obj0 = PolygonPath.xy2Path([0 5 8 8 5 0], [0 0 0 2 2 2]);
            [act,s] = intersectLine(obj0, [6 0], pi/2, false);
            exp = [6 0; 6 2];
            testCase.verifyEqual(act, exp, 'AbsTol',1e-12);
            testCase.verifyEqual(size(s), [2,1]);
            
        end%fcn
        
        function testTerminalIntersections(testCase)
            obj0 = PolygonPath.xy2Path([0 2 10], [0 0 1]);
            [P0,P1] = obj0.termPoints();
            
            % Intersection with initial point
            [act,s] = intersectLine(obj0, [2 -2], 3*pi/4, false);
            exp = P0';
            testCase.verifyEqual(act, exp, 'AbsTol',1e-12);
            testCase.verifyEqual(size(s), [1,1]);
            
            % Intersection with end point
            [act,s] = intersectLine(obj0, [10 0], pi/2, true);
            exp = P1';
            testCase.verifyEqual(act, exp, 'AbsTol',1e-12);
            testCase.verifyEqual(size(s), [1,1]);
        end%fcn
    end
    
end%class
