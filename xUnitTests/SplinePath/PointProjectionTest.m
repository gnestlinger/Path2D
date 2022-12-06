classdef PointProjectionTest < matlab.unittest.TestCase
    
    methods (Test)
        
        function testUniqueSolutions(testCase)
            cc = [-1/2 1 0];
            pp = mkpp([-4 -2 0 2 4],[0 1 -4; cc; 0 1 -2; -cc; 0 1 0; cc; 0 1 2; -cc], 2);
            obj = SplinePath.pp2Path(pp);
            
            [Q,idx,tau,dphi] = pointProjection(obj, [3 0], [], false);
            testCase.verifyEqual(Q, [3 -0.5], 'AbsTol',2e-15);
            testCase.verifyEqual(idx, 4);
            testCase.verifyEqual(tau, 3, 'AbsTol',2e-15);
            testCase.verifyEqual(dphi, 0);
        end%fcn
        
        function testTerminalSolutions(testCase)
            obj = SplinePath.pp2Path(mkpp(...
                [-3 -2 1 2], ...
                [0 0 -2; 0 -1 5; 0 1 -2; 1 -4 4; 0 1 1; 0 0 1], 2)...
                );
            [Q,idx,tau,dphi] = pointProjection(obj, [2 5], [], false);
            
            % Initial point solution
            testCase.verifyEqual(Q(1,:), [-2 5]);
            testCase.verifyEqual(tau(1), obj.Breaks(1));
            
            % End point solution
            testCase.verifyEqual(Q(4,:), [2 1]);
            testCase.verifyEqual(tau(4), obj.Breaks(end));
            
            % Intermediate solution(s)
            testCase.verifyEqual(Q(2,:), [-2 4]);
            testCase.verifyEqual(tau(2), obj.Breaks(2));
            
            testCase.verifyEqual(idx, [1; 2; 2; 3]);
            testCase.verifyEqual(dphi, [0; 0; 0; 0]);
        end%fcn
        
        function testMultipleSolutions(testCase)
            obj = SplinePath.pp2Path(mkpp([-3 3], [0 1 -3; 1 -6 9], 2));
            
            [Q,idx,tau,dphi] = pointProjection(obj, [0 6], [], false); % [6 3]
            sq11 = sqrt(11/2);
            testCase.verifyEqual(Q, [-sq11 5.5; 0 0; sq11 5.5], 'AbsTol',2e-14);
            testCase.verifyEqual(idx, [1; 1; 1]);
            testCase.verifyEqual(tau, [-sq11; 0; sq11], 'AbsTol',1e-14);
            testCase.verifyEqual(dphi, [0; 0; 0]);
        end%fcn
        
        function testNoSolution(testCase)
            cc = [-1/2 1 0];
            pp = mkpp([-4 -2 0 2 4],[0 1 -4; cc; 0 1 -2; -cc; 0 1 0; cc; 0 1 2; -cc], 2);
            obj = SplinePath.pp2Path(pp);
            
            [Q,idx,tau,dphi] = pointProjection(obj, [4 1], [], false);
            testCase.verifyEqual(Q, zeros(0,2));
            testCase.verifyEqual(idx, zeros(0,1));
            testCase.verifyEqual(tau, zeros(0,1));
            testCase.verifyEqual(dphi, zeros(0,1));
        end%fcn
        
    end
    
end%class
