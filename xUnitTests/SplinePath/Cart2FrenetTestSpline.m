classdef Cart2FrenetTestSpline < matlab.unittest.TestCase
    
    methods (Test)
        
        function testUniqueSolutions(testCase)
            cc = [-1/2 1 0];
            pp = mkpp([-2 0 2 4],[0 1 -2; -cc; 0 1 0; cc; 0 1 2; -cc], 2);
            obj = SplinePath.pp2Path(pp);
            
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([3 0], [], false);
            testCase.verifyEqual(sd(1), 5.739, 'AbsTol',1e-4);
            testCase.verifyEqual(sd(2), -0.5);
            testCase.verifyEqual(Q, [3 -0.5], 'AbsTol',1e-12);
            testCase.verifyEqual(idx, 3);
            testCase.verifyEqual(tau, 3, 'AbsTol',2e-15);
            testCase.verifyEqual(dphi, 0);
        end%fcn
        
        function testTerminalSolutions(testCase)
%             obj = SplinePath(0:3, cat(3, [1 1 1;0 0 0], [0 1 2; 1 1 1]));
            cc = [-1/2 1 0];
            pp = mkpp([0 2 4],[0 1 0; cc; 0 1 2; -cc], 2);
            obj = SplinePath.pp2Path(pp);
            
            % Initial point solution and others
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([1 -1], [], false);
            testCase.verifyEqual(sd(:,1), [0; 1.1478; 2.2956], 'AbsTol',2e-5);
            testCase.verifyEqual(sd(:,2), [sqrt(2); 1.5; sqrt(2)]);
            testCase.verifyEqual(Q, [0 0; 1 0.5; 2 0]);
            testCase.verifyEqual(idx, [1 1 2]');
            testCase.verifyEqual(tau, [0 1 2]');
            testCase.verifyEqual(dphi, zeros(3,1));
            
            % End point solution and others
            % TODO: This call returns one repeated solution
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([3 1], [], false);
            testCase.verifyEqual(sd(:,1), [2.2956 2.2956 3.4434 4.5912]', 'AbsTol',5e-5);
            testCase.verifyEqual(sd(:,2), [-sqrt(2) -sqrt(2) -1.5 -sqrt(2)]');
            testCase.verifyEqual(Q, [2 0; 2 0; 3 -0.5; 4 0], 'AbsTol',2e-15);
            testCase.verifyEqual(idx, [1 2 2 2]');
            testCase.verifyEqual(tau, [2 2 3 4]', 'AbsTol',2e-15);
            testCase.verifyEqual(dphi, zeros(4,1));
        end%fcn
        
        function testMultipleSolutions(testCase)
            cc = [-1/2 1 0];
            pp = mkpp([-2 0 2 4],[0 1 -2; -cc; 0 1 0; cc; 0 1 2; -cc], 2);
            obj = SplinePath.pp2Path(pp);
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([1 4], [], false);
            
            testCase.verifyEqual(sd, [0.5104 -5.0421; 3.4434 -3.5; 6.3764 -5.0421], 'AbsTol',1e-3);
            testCase.verifyEqual(Q, [-1.6027 -0.3184; 1 0.5; 3.6027 -0.3184], 'AbsTol',1e-3);
            testCase.verifyEqual(idx, [1 2 3]');
            testCase.verifyEqual(tau, [-1.6027 1 3.6027]', 'AbsTol',1e-5);
            testCase.verifyEqual(dphi, zeros(3,1));
        end%fcn
        
    end
    
end%class
