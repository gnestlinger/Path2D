classdef IntersectCircleTestSpline < matlab.unittest.TestCase
    
    methods (Test)
        function testNoIntersection(testCase)
            obj0 = SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0],2,2,2));
            [xy,tau] = obj0.intersectCircle([1 1], 0.5, false);
            testCase.verifyEqual(xy, zeros(0,2));
            testCase.verifyEqual(tau, zeros(0,1));
        end%fcn
        
        function testSingleIntersection(testCase)
            obj0 = SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0],2,2,2));
            [act,tau] = obj0.intersectCircle([2 1], 1, false);
            exp = [1.292893 0.292893];
            testCase.verifyEqual(act, exp, 'AbsTol', 1e-6);
            testCase.verifyEqual(tau, 1.292893, 'AbsTol', 1e-6);
        end%fcn
        
        function testMultipleIntersections(testCase)
            obj0 = SplinePath.pp2Path(mkpp([-3 3], [0 1 -3; 1 -6 9], 2));
            [act,tau] = intersectCircle(obj0, [0 4], 2, false);
            exp = [-2 4; -1.73205 3.0; 1.73205 3.0; 2 4];
            testCase.verifyEqual(act, exp, 'AbsTol',1e-6);
            testCase.verifyEqual(tau, [-2; -1.73205; 1.73205; 2], 'AbsTol',1e-6);
        end%fcn
        
        function testTerminalIntersections(testCase)
            x0 = -2; 
            x1 = 2;
            obj0 = SplinePath.pp2Path(mkpp([x0 x1], [0 1 x0; 1 2*x0 x0^2], 2));
            
            % Intersection with initial & end point
            [act,tau] = intersectCircle(obj0, [0 6], sqrt(8), false);
            % Due to limited accuracy of ROOTS(), the intersection at the
            % initial point is not found
            exp = [-2 4];
            testCase.verifyEqual(act, exp, 'AbsTol',1e-15);
            testCase.verifyEqual(tau, -2, 'AbsTol',1e-15);
            
            % Intersection with end point
            [act,tau] = intersectCircle(obj0, [3 4], 1, false);
            exp = [2 4];
            testCase.verifyEqual(act, exp, 'AbsTol',1e-12);
            testCase.verifyEqual(tau, 2, 'AbsTol',1e-13);
        end%fcn
    end
    
end%class
