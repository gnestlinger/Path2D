classdef S2TauTestDubins < matlab.unittest.TestCase
    
    properties (TestParameter)
    end
    
    
    
    methods (Test)
        function testReturnValuesNonEmptyPath(testCase)
            
            obj = DubinsPath([0 0 0], [1 0 -1], [pi/4 1 pi/4], 2);
            s = [-1 -0.2 0 0.1 100 2 1.5 5 9 20]; %#ok<*PROP>
            [tau,idx] = obj.s2tau(s);
            
            [tau0,tau1] = obj.domain();
            tauSet = interp1([0; obj.cumlengths()], tau0:tau1, s, 'linear','extrap');
            verifyEqual(testCase, tau, tauSet, 'AbsTol',2e-14);
            
            verifyEqual(testCase, idx, uint32([1 1 1 1 3 3 2 3 3 3]));
        end%fcn
        
        function testReturnValuesOutOfBounds(testCase)
            obj = DubinsPath([0 0 0], [1 0 -1], [2 1 2], 2);
            
            s = [-1 7];
            N = obj.numel();
            [tau0,tau1] = obj.domain();
            tauSet = interp1([0; obj.cumlengths()], tau0:tau1, s, 'linear','extrap');
            
            [tau,idx] = obj.s2tau(s);
            verifyEqual(testCase, tau, tauSet);
            verifyEqual(testCase, idx, uint32([1 N]));
        end%fcn
    end
    
end%class
