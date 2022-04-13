classdef PointProjectionTest < matlab.unittest.TestCase
    
    methods (Test)
        
        function testUniqueSolutions(testCase)
            obj = PolygonPath.xy2Path(1:4, [2 2 2 2]);
            [Q,idx,tau] = pointProjection(obj, [2 0], false);
            testCase.verifyEqual(Q, [2 2]);
            testCase.verifyEqual(idx, 2);
            testCase.verifyEqual(tau, 0);
        end%fcn
        
        function testTerminalSolutions(testCase)
            obj = PolygonPath.xy2Path(1:4, [2 2 2 2]);
            
            % Initial point solution
            [Q,idx,tau] = pointProjection(obj, [1 0], false);
            testCase.verifyEqual(Q, [1 2]);
            testCase.verifyEqual(idx, 1);
            testCase.verifyEqual(tau, 0);
            
            % End point solution
            [Q,idx,tau] = pointProjection(obj, [4 0], false);
            testCase.verifyEqual(Q, [4 2]);
            testCase.verifyEqual(idx, 3);
            testCase.verifyEqual(tau, 1);
        end%fcn
        
        function testMultipleSolutions(testCase)
            obj = PolygonPath.circle(3, [-pi pi]/2, 10);
            [Q,idx,tau] = pointProjection(obj, [0 0], false);
            
            N = 9; % Number of solutions
            testCase.verifySize(Q, [N 2]);
            testCase.verifyEqual(idx, (1:N)');
            testCase.verifyEqual(tau, 0.5*ones(N,1), 'AbsTol',1e-10);
        end%fcn
        
        function testNoSolution(testCase)
            obj = PolygonPath.xy2Path(0:2, [0 0 -1]);
            [Q,idx,tau] = pointProjection(obj, [1.5 0.6], false);
            testCase.verifyEqual(Q, zeros(0,2));
            testCase.verifyEqual(idx, zeros(0,1));
            testCase.verifyEqual(tau, zeros(0,1));
        end%fcn
    end
    
end%class
