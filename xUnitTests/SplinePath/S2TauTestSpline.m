classdef S2TauTestSpline < matlab.unittest.TestCase
    
    properties (TestParameter)
        s = struct(...
            'emptyTau', [], ...
            'oneElmTau', 1.1, ...
            'vectorTau', linspace(-1,10,24), ...
            'ndTau', ones(10,2,3))
    end
    
    
    
    methods (Test)
        function testReturnValuesZeroLengthPath(testCase, s)
            % Non-empty but zero length path
            obj = SplinePath([0 0], cat(3, [1;2], [0;0]));
            [tau,idx] = obj.s2tau(s);
            verifyEqual(testCase, tau, nan(size(s)));
            verifyEqual(testCase, idx, zeros(size(s), 'uint32'));
        end%fcn
        
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
