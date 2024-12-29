classdef S2TauTestSpline < matlab.unittest.TestCase
    
    properties (TestParameter)
    end
    
    
    
    methods (Test)
        function testReturnValuesNonEmptyPath(testCase)
            obj = SplinePath([0 10], cat(3, [2;0], [0;0]));
            s = [-1 -0.2 0 0.1 100 2 4 5 9 20]; %#ok<*PROP>
            [tau,idx] = obj.s2tau(s);
            
            [tau0,tau1] = obj.domain();
            tauSet = interp1([0 obj.cumlengths()'], [tau0 tau1], s, 'linear','extrap');
            
            verifyEqual(testCase, tau, tauSet, 'AbsTol',1e-15);
            verifyEqual(testCase, idx, uint32([1 1 1 1 1 1 1 1 1 1]));
        end%fcn
        
        function testReturnValuesOutOfBounds(testCase)
            obj = SplinePath([0 2 5], cat(3, [2 1;1 0], [0 4;0 2]));
%             obj.plot('.')
            
            s = [-1  14];
            [tau,idx] = obj.s2tau(s);
            
            tauSet = interp1([0;obj.cumlengths()], obj.Breaks, s, 'linear', 'extrap');
            verifyEqual(testCase, tau, tauSet);
            verifyEqual(testCase, idx, uint32([1 2]));
        end%fcn
    end
    
end%class
