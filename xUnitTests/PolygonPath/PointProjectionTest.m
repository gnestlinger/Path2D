classdef PointProjectionTest < matlab.unittest.TestCase
    
    methods (Test)
        
        function testUniqueSolutions(testCase)
            obj = PolygonPath.xy2Path(1:4, [2 2 2 2]);
            [Q,idx,tau,dphi] = pointProjection(obj, [2 0], [], false);
            testCase.verifyEqual(Q, [2 2]);
            testCase.verifyEqual(idx, 2);
            testCase.verifyEqual(tau, 1);
            testCase.verifySize(dphi, [1 1]);
        end%fcn
        
        function testTerminalSolutions(testCase)
            obj = PolygonPath.xy2Path(1:4, [2 2 2 2]);
            
            % Initial point solution
            [Q,idx,tau,dphi] = pointProjection(obj, [1 0], [], false);
            testCase.verifyEqual(Q, [1 2]);
            testCase.verifyEqual(idx, 1);
            testCase.verifyEqual(tau, 0);
            testCase.verifySize(dphi, [1 1]);
            
            % End point solution
            [Q,idx,tau,dphi] = pointProjection(obj, [4 0], [], false);
            testCase.verifyEqual(Q, [4 2]);
            testCase.verifyEqual(idx, 3);
            testCase.verifyEqual(tau, 3);
            testCase.verifySize(dphi, [1 1]);
        end%fcn
        
        function testMultipleSolutions(testCase)
            obj = PolygonPath.circle(3, [-pi pi]/2, 10);
            [Q,idx,tau,dphi] = pointProjection(obj, [0 0], [], false);
            
            N = 9; % One solution per path segment
            testCase.verifySize(Q, [N 2]);
            testCase.verifyEqual(idx, (1:N)');
            testCase.verifyEqual(tau, (0:N-1)'+0.5, 'AbsTol',1e-10);
            testCase.verifySize(dphi, [N 1]);
        end%fcn
        
        function testNoSolution(testCase)
            obj = PolygonPath.xy2Path(0:1, [0 0]);
            [Q,idx,tau,dphi] = pointProjection(obj, [1.5 0.6], [], false);
            testCase.verifySize(Q, [0 2]);
            testCase.verifySize(idx, [0 1]);
            testCase.verifySize(tau, [0 1]);
            testCase.verifySize(dphi, [0 1]);
        end%fcn
        
        function testCircuitPath(testCase)
            obj = PolygonPath.circle(3, [0 2*pi], 15);
            assert(obj.IsCircuit)
            [Q,idx,tau,dphi] = pointProjection(obj, [0 0], [], false);
            
            % One solution per path segment, at the center of each segment
            N = 14;
            testCase.verifySize(Q, [N 2]);
            testCase.verifyEqual(idx, (1:N)');
            testCase.verifyEqual(tau, (0:N-1)'+0.5, 'AbsTol',1e-15);
            testCase.verifyEqual(dphi, zeros(N,1));
        end%fcn
    end
    
end%class
