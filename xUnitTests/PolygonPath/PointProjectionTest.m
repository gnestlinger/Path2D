classdef PointProjectionTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        DoPlot = {false}
    end
    
    
    
    methods (Test)
        function testUniqueSolutions(testCase, DoPlot)
            obj = PolygonPath.xy2Path(1:4, [2 2 2 2]);
            [Q,idx,tau,dphi] = pointProjection(obj, [2 0], [], DoPlot);
            testCase.verifyEqual(Q, [2 2]);
            testCase.verifyEqual(idx, 2);
            testCase.verifyEqual(tau, 1);
            testCase.verifySize(dphi, [1 1]);
        end%fcn
        
        function testTerminalSolutions(testCase, DoPlot)
            obj = PolygonPath.xy2Path(1:4, [2 2 2 2]);
            
            % Initial point solution
            [Q,idx,tau,dphi] = pointProjection(obj, [1 0], [], DoPlot);
            testCase.verifyEqual(Q, [1 2]);
            testCase.verifyEqual(idx, 1);
            testCase.verifyEqual(tau, 0);
            testCase.verifySize(dphi, [1 1]);
            
            % End point solution
            [Q,idx,tau,dphi] = pointProjection(obj, [4 0], [], DoPlot);
            testCase.verifyEqual(Q, [4 2]);
            testCase.verifyEqual(idx, 3);
            testCase.verifyEqual(tau, 3);
            testCase.verifySize(dphi, [1 1]);
        end%fcn
        
        function testInitalSolution(testCase, DoPlot)
        % Test initial solutions with a 2-waypoint path
        
            P0 = [0 0];
            P1 = [10 0];
            obj = PolygonPath.straight(P0, P1);
            
            % Initial point solution
            [Q,idx,tau,dphi] = pointProjection(obj, P0 + [0 1], [], DoPlot);
            verifyEqual(testCase, Q, P0);
            verifyEqual(testCase, idx, 1);
            verifyEqual(testCase, tau, 0);
            verifySize(testCase, dphi, [1 1]);
        end%fcn
        
        function testEndSolution(testCase, DoPlot)
        % Test end solutions with a 2-waypoint path
        
            P0 = [0 0];
            P1 = [10 0];
            obj = PolygonPath.straight(P0, P1);
            
            % End point solution
            [Q,idx,tau,dphi] = pointProjection(obj, P1 + [0 1], [], DoPlot);
            verifyEqual(testCase, Q, P1, 'RelTol',1e-14);
            verifyEqual(testCase, idx, 1);
            verifyEqual(testCase, tau, 1, 'RelTol',3e-16);
            verifySize(testCase, dphi, [1 1]);
        end%fcn
        
        function testMultipleSolutions(testCase, DoPlot)
            obj = PolygonPath.circle(3, [-pi pi]/2, 9);
            [Q,idx,tau,dphi] = pointProjection(obj, [0 0], [], DoPlot);
            
            N = 9; % One solution per path segment
            testCase.verifySize(Q, [N 2]);
            testCase.verifyEqual(idx, (1:N)');
            testCase.verifyEqual(tau, (0:N-1)'+0.5, 'AbsTol',1e-10);
            testCase.verifySize(dphi, [N 1]);
        end%fcn
        
        function testNoSolution(testCase, DoPlot)
            obj = PolygonPath.xy2Path(0:1, [0 0]);
            [Q,idx,tau,dphi] = pointProjection(obj, [1.5 0.6], [], DoPlot);
            testCase.verifySize(Q, [0 2]);
            testCase.verifySize(idx, [0 1]);
            testCase.verifySize(tau, [0 1]);
            testCase.verifySize(dphi, [0 1]);
        end%fcn
        
        function testCircuitPath(testCase, DoPlot)
            obj = PolygonPath.circle(3, [0 2*pi], 14);
            assert(obj.IsCircuit)
            [Q,idx,tau,dphi] = pointProjection(obj, [0 0], [], DoPlot);
            
            % One solution per path segment, at the center of each segment
            N = 14;
            testCase.verifySize(Q, [N 2]);
            testCase.verifyEqual(idx, (1:N)');
            testCase.verifyEqual(tau, (0:N-1)'+0.5, 'AbsTol',1e-15);
            testCase.verifyEqual(dphi, zeros(N,1));
        end%fcn
    end
    
end%class
