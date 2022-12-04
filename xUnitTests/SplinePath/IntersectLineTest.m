classdef IntersectLineTest < matlab.unittest.TestCase
    
    methods (Test)
        function testNoIntersection(testCase)
            
            obj0 = SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0],2,2,2));
            [act,tau] = obj0.intersectLine([1 1], pi/4, false);
            exp = zeros(0,2);
            testCase.verifyEqual(act, exp);
            testCase.verifyEqual(tau, zeros(0,1));
            
        end%fcn
        
        function testSingleIntersection(testCase)
            obj0 = SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0],2,2,2));
            [act,tau] = obj0.intersectLine([1 1], pi/4+eps, false);
            exp = [0 0];
            testCase.verifyEqual(act, exp, 'AbsTol', 1e-15);
            testCase.verifyEqual(tau, 0, 'AbsTol', 1e-15);
        end%fcn
        
        function testMultipleIntersections(testCase)
            
            obj0 = SplinePath.pp2Path(mkpp([-3 3], [0 1 -3; 1 -6 9], 2));
            [act,tau] = intersectLine(obj0, [0 4], pi, false);
            exp = [-2 4; 2 4];
            testCase.verifyEqual(act, exp, 'AbsTol',1e-15);
            testCase.verifyEqual(tau, [-2; 2]);
            
        end%fcn
        
        function testTerminalIntersections(testCase)
            
            obj0 = SplinePath.pp2Path(mkpp([-2 2], [0 1 -2; 1 -4 4], 2));
            
            % Intersection with initial & end point
            [act,tau] = intersectLine(obj0, [0 4], pi, false);
            exp = [-2 4; 2 4];
            testCase.verifyEqual(act, exp, 'AbsTol',1e-15);
            testCase.verifyEqual(tau, [-2; 2]);
            
        end%fcn
    end
    
end%class
