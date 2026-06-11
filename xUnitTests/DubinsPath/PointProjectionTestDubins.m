classdef PointProjectionTestDubins < matlab.unittest.TestCase
    
    properties (TestParameter)
        Dir = struct('Left',1, 'Right',-1)
        
        DoPlot = {false}
    end


    
    methods (Test)
        function testUniqueSolutions(testCase, DoPlot)
        % Test for a unique solution.
        
            obj = DubinsPath([0 0 0], [1 0], [pi 3], 2);
            POI = [1 4];
            [Q,idx,tau,dphi] = obj.pointProjection(POI, [], DoPlot);
            verifyEqual(testCase, Q, [2 4]);
            verifyEqual(testCase, idx, 2);
            verifyEqual(testCase, tau, 1 + 2/3);
            verifyEqual(testCase, dphi, 0);
        end%fcn
        
        function testInitalSolution(testCase, DoPlot)
        % Test initial point solution.
        
            obj = DubinsPath([0 0 0], [1 0 -1], [pi 3 pi], 2);
            
            [Q,idx,tau,dphi] = pointProjection(obj, [0 1], [], DoPlot);
            verifyEqual(testCase, Q, [0 0], 'absTol',2e-15);
            verifyEqual(testCase, idx, 1);
            verifyEqual(testCase, tau, 0);
            verifyEqual(testCase, dphi, 0);
        end%fcn
        
        function testEndSolution(testCase, DoPlot)
        % Test end point solution.
        
            obj = DubinsPath([0 0 0], [1 0 -1], [pi 3 pi], 2);
            
            [Q,idx,tau,dphi] = pointProjection(obj, [4 6], [], DoPlot);
            verifyEqual(testCase, Q, [4 7]);
            verifyEqual(testCase, idx, 3);
            verifyEqual(testCase, tau, 3);
            verifyEqual(testCase, dphi, 0);
        end%fcn
        
        function testMultipleSolutions(testCase, DoPlot)
            obj = DubinsPath([0 0 0], [1 0 -1 0 -1 0], [1.5*pi 2 1.5*pi 1 pi 2], 2);

            [Q,idx,tau,dphi] = pointProjection(obj, [2 5], [], DoPlot);
            
            QSet = [0.9142 3.9142; 0.5614 8.0517; 2 8.2426; 3.0467 8.14; 4.4142 5];
            tauSet = [1.3536; 2.813; 3.5858; 4.2048; 5.6213];
            testCase.verifyEqual(Q, QSet, 'AbsTol',5e-5);
            testCase.verifyEqual(idx, (2:6)');
            testCase.verifyEqual(tau, tauSet, 'AbsTol',5e-5);
            testCase.verifyEqual(dphi, zeros(size(tauSet)));
        end%fcn
        
        function testNoSolution(testCase, DoPlot)

            obj = DubinsPath([0 0 0], [1 0 -1], [pi 3 pi], 2);
            [Q,idx,tau,dphi] = pointProjection(obj, [-1 1], [], DoPlot);
            verifySize(testCase, Q, [0 2]);
            verifySize(testCase, idx, [0 1]);
            verifySize(testCase, tau, [0 1]);
            verifySize(testCase, dphi, [0 1]);
        end%fcn

        function testCircuitPath(testCase, Dir, DoPlot)

            % Create a path that is symmetric around (0,0)
            obj = DubinsPath([0 Dir*-2.5 0], [Dir 0 Dir 0 Dir], [pi 1 2*pi 1 pi], 2);
            assert(obj.IsCircuit)

            [Q,idx,tau,dphi] = pointProjection(obj, [0 0], [], DoPlot);

            % TODO: accept/discard repeated solution at initial/end point?
            N = 5;
            verifySize(testCase, Q, [N 2]);
            verifyEqual(testCase, idx, (1:N)');
            verifyEqual(testCase, tau, [0; 1.5; 2.5; 3.5; 5]);
            verifyEqual(testCase, dphi, zeros(N,1));
        end%fcn
    end
    
end%class
