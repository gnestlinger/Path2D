classdef Cart2FrenetTestDubins < matlab.unittest.TestCase
    
    properties (TestParameter)
        DoPlot = {false}
    end
    
    
    
    methods (Test)
        function testUniqueSolutions(testCase, DoPlot)
            
            r = 2;
            obj = DubinsPath([0 0 0], [1 0 -1], [pi 3 pi], r);
            
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([3 4], [], DoPlot);
            verifyEqual(testCase, sd(1), r*pi/2 + 2, 'AbsTol',1e-4);
            verifyEqual(testCase, sd(2), 1);
            verifyEqual(testCase, Q, [2 4]);
            verifyEqual(testCase, idx, 2);
            verifyEqual(testCase, tau, 1+2/3, 'AbsTol',1e-16);
            verifyEqual(testCase, dphi, 0);
        end%fcn
        
        function testInitialPointSolution(testCase, DoPlot)
        % Check for initial point solution
            
            r = 2;
            obj = DubinsPath([0 0 0], [1 0 -1], [pi 3 pi], r);
            
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([0 -1], [], DoPlot);
            verifyEqual(testCase, sd(:,1), 0);
            verifyEqual(testCase, sd(:,2), 1);
            verifyEqual(testCase, Q, [0 0], 'AbsTol',4e-16);
            verifyEqual(testCase, idx, 1);
            verifyEqual(testCase, tau, 0);
            verifyEqual(testCase, dphi, 0);
        end%fcn
        
        function testEndPointSolution(testCase, DoPlot)
        % Check for end point solution
            
            r = 2;
            obj = DubinsPath([0 0 0], [1 0 -1], [pi 3 pi], r);
            
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([4 8], [], DoPlot);
            verifyEqual(testCase, sd(:,1), r*pi + 3, 'AbsTol',2e-5);
            verifyEqual(testCase, sd(:,2), -1);
            verifyEqual(testCase, Q, [4 7]);
            verifyEqual(testCase, idx, 3);
            verifyEqual(testCase, tau, 3);
            verifyEqual(testCase, dphi, 0);
        end%fcn
        
        function testMultipleSolutions(testCase, DoPlot)
            
            r = 2;
            obj = DubinsPath([0 0 0], [1 0 -1 0 -1 0], [r*0.75*pi 1 r*0.75*pi 1 pi 1], r);
            
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([2.5 5], [], DoPlot);
            
            verifyEqual(testCase, sd, [...
                5.065942 1.889087;
                9.193818 2.655892;
                10.80345 2.535533;
                13.14361 2.820265;
                15.10190 2.621320], 'AbsTol',1e-5);
            verifyEqual(testCase, Q, [...
                1.164213 3.664213; ...
                0.966619 7.168527; ...
                2.500000 7.535533; ...
                4.636245 6.841291; ...
                5.121320 5.000000], 'AbsTol',1e-6);
            verifyEqual(testCase, idx, [2 3 4 5 6]');
            verifyEqual(testCase, tau, [1.353553 2.738782 3.378679 4.547122 5.535533]', 'AbsTol',1e-5);
            verifyEqual(testCase, dphi, zeros(5,1));
        end%fcn
        
        function testFallbackInitialPoint(testCase, DoPlot)
        % Fallback solution at initial point
            
            r = 2;
            obj = DubinsPath([0 0 0], [1 0 -1], [r*0.75*pi 1 r*0.75*pi], r);
            P0 = obj.termPoints();
            
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([-1 -1], [], DoPlot);
            verifyEqual(testCase, sd, [0 sqrt(2)]);
            verifyEqual(testCase, Q, P0(:)');
            verifyEqual(testCase, idx, 1);
            verifyEqual(testCase, tau, 0);
            verifyEqual(testCase, dphi, pi/4);
        end%fcn
        
        function testFallbackEndPoint(testCase, DoPlot)
        % Fallback solution at end point
            
            r = 2;
            
            % Adjust length of line such that y-component of endpoint is
            % integer valued
            dPhiCircle = 0.75*pi;
            dyCircle = r*sin(dPhiCircle - pi/2);
            dyLine = ceil(2*dyCircle) - 2*dyCircle;
            lLine = dyLine/sin(dPhiCircle);
            
            obj = DubinsPath([0 0 0], [1 0 -1], [r*dPhiCircle lLine r*dPhiCircle], r);
            [~,P1] = obj.termPoints();
            S = length(obj);
            
            [sd,Q,idx,tau,dphi] = obj.cart2frenet([3 7], [], DoPlot);
            verifyEqual(testCase, sd, [S 0]);
            verifyEqual(testCase, Q, P1(:)');
            verifyEqual(testCase, idx, 3);
            verifyEqual(testCase, tau, 3);
            verifyEqual(testCase, dphi, pi/2);
        end%fcn
        
        % function testFallbackNonTerminalPoint(testCase, DoPlot)
        % % Fallback non-terminal point
        %     % TODO
        % end%fcn
    end
    
end%class
