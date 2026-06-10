classdef Cart2FrenetTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        DoPlot = {false}
    end
    
    
    
    methods (Test)
        function testUniqueSolutions(testCase, DoPlot)
            obj = PolygonPath.xy2Path(1:4, [2 2 2 2]);
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([2 0], [], DoPlot);
            testCase.verifyEqual(sd, [1 2]);
            testCase.verifyEqual(Q, [2 2]);
            testCase.verifyEqual(idx, 2);
            testCase.verifyEqual(tau, 1);
            testCase.verifyEqual(dphi, 0);
        end%fcn
        
        function testTerminalSolutions(testCase, DoPlot)
            obj = PolygonPath.xy2Path(1:4, [2 2 2 2]);
            
            % Initial point solution
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([1 0], [], DoPlot);
            testCase.verifyEqual(sd, [0 2]);
            testCase.verifyEqual(Q, [1 2]);
            testCase.verifyEqual(idx, 1);
            testCase.verifyEqual(tau, 0);
            testCase.verifyEqual(dphi, 0);
            
            % End point solution
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([4 0], [], DoPlot);
            testCase.verifyEqual(sd, [3 2]);
            testCase.verifyEqual(Q, [4 2]);
            testCase.verifyEqual(idx, 3);
            testCase.verifyEqual(tau, 3);
            testCase.verifyEqual(dphi, 0);
        end%fcn
        
        function testMultipleSolutions(testCase, DoPlot)
            obj = PolygonPath.circle(3, [-pi pi]/2, 9);
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([0 0], [], DoPlot);
            
            N = 9; % One solution per path segment
            testCase.verifySize(sd, [N 2]);
            testCase.verifySize(Q, [N 2]);
            testCase.verifyEqual(idx, (1:N)');
            testCase.verifyTrue(all(idx <= numel(obj.x) - 1))
            testCase.verifyEqual(tau, (0:N-1)'+0.5, 'AbsTol',1e-10);
            testCase.verifyEqual(dphi, zeros(N,1), 'AbsTol',1e-15);
        end%fcn
        
        function testFallbackSolution(testCase, DoPlot)
            obj = PolygonPath.xy2Path([0 0.5 1], [0 0 0]);
            [P0,P1] = obj.termPoints();
            
            % Fallback initial point
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([-1 -1], [], DoPlot);
            testCase.verifyEqual(sd, [0 sqrt(2)]);
            testCase.verifyEqual(Q, P0');
            testCase.verifyEqual(idx, 1);
            testCase.verifyEqual(tau, 0);
            testCase.verifyEqual(dphi, pi/4);
            
            % Fallback end point
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([2 1], [], DoPlot);
            testCase.verifyEqual(sd, [1 -sqrt(2)]);
            testCase.verifyEqual(Q, P1');
            testCase.verifyEqual(idx, 2);
            testCase.verifyTrue(idx <= numel(obj.x) - 1);
            testCase.verifyEqual(tau, 2);
            testCase.verifyEqual(dphi, pi/4);
            
            % Fallback non-terminal point
            obj = PolygonPath.xy2Path([0 0.5 1], [0 0.5 0]);
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([0.5 1], [], DoPlot);
            testCase.verifyEqual(sd, [sqrt(0.5) -0.5]);
            testCase.verifyEqual(Q, [0.5 0.5]);
            testCase.verifyEqual(idx, 2);
            testCase.verifyTrue(idx <= numel(obj.x) - 1);
            testCase.verifyEqual(tau, 1);
            testCase.verifyEqual(dphi, pi/4);
        end%fcn
        
        function testCircuitPath(testCase, DoPlot)
            obj = PolygonPath.circle(3, [0 2*pi], 14);
            assert(obj.IsCircuit)
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([0 0], [], DoPlot);
            
            % One solution per path segment, at the center of each segment
            N = numel(obj.x) - 1;
            testCase.verifySize(sd, [N 2]);
            testCase.verifySize(Q, [N 2]);
            testCase.verifyEqual(idx, (1:N)');
            testCase.verifyTrue(all(idx <= numel(obj.x) - 1))
            testCase.verifyEqual(tau, (0:N-1)'+0.5, 'AbsTol',1e-15);
            testCase.verifyEqual(dphi, zeros(N,1), 'AbsTol',1e-15);
        end%fcn
        
        function testSignD(testCase, DoPlot)
            obj = PolygonPath.circle(1, [0 2*pi], 8);
            assert(obj.IsCircuit)
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([1 0.5], DoPlot);
            
            testCase.verifyTrue(sd(1,2) > 0);
            testCase.verifyTrue(sd(2,2) < 0);
        end%fcn
    end
    
end%class
