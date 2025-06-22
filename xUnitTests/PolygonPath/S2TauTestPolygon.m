classdef S2TauTestPolygon < matlab.unittest.TestCase
    
    properties (TestParameter)
    end
    
    
    
    methods (Test)
        function testReturnValuesNonEmptyPath(testCase)
            obj = PolygonPath.xy2Path([-3 1 8 34], [3 1 0 6]);
            s = [-1 -0.2 0 0.1 100 2 4 5 9 20];
            [tau,idx] = obj.s2tau(s);
            
            [tau0,tau1] = obj.domain();
            tauSet = interp1([0 obj.cumlengths()'], tau0:tau1, s, 'linear','extrap');
            
            verifyEqual(testCase, tau, tauSet, 'AbsTol',1e-15)
            verifyEqual(testCase, idx, uint32([1 1 1 1 3 1 1 2 2 3]));
        end%fcn
        
        function testReturnValuesOutOfBounds(testCase)
            obj = PolygonPath.xy2Path([0 4 5 11], [0 0 0 0]);
            
            s0 = -1;
            s1 = 14;
            s = [s0  s1];
            [tau,idx] = obj.s2tau(s);
            
            [tau0,tau1] = obj.domain();
            tauSet = interp1([0;obj.cumlengths()], tau0:tau1, s, 'linear', 'extrap');
            verifyEqual(testCase, tau, tauSet);
            verifyEqual(testCase, idx, uint32([1 3]));
        end%fcn
    end
    
end%class
