classdef S2TauTestDubins < matlab.unittest.TestCase
    
    properties (TestParameter)
        s = struct(...
            'emptyTau', [], ...
            'scalarTau', 0, ...
            'vectorTau', linspace(-1,4,26), ...
            'ndTau', [ones(10,2,3); zeros(1,2,3)])
    end
    
    methods (Test)
        function testReturnValuesEmptyPath(testCase, s)
            % Empty path
            obj = DubinsPath();
            [tau,idx] = obj.s2tau(s);
            verifyEqual(testCase, tau, nan(size(s)));
            verifyEqual(testCase, idx, zeros(size(s), 'uint32'));
        end%fcn

        function testReturnValuesZeroLengthPath(testCase, s)

            % For non-empty s, at least one value should equal zero!
            assert(isempty(s) || any(s(:)==0))

            % Non-empty but zero-length path
            obj = DubinsPath([0 0 0], 0, 0, 2);
            [tau,idx] = obj.s2tau(s);
            
            tauSet = nan(size(s));
            tauSet(s==0) = 0;
            verifyEqual(testCase, tau, tauSet);
            
            idxSet = zeros(size(s), 'uint32');
            idxSet(s==0) = uint32(1);
            verifyEqual(testCase, idx, idxSet);
        end%fcn
        
        function testReturnValuesNonEmptyPath(testCase)

            obj = DubinsPath([0 0 0], [1 0 -1], [pi/4 1 pi/4], 2);
            s = [-1 -0.2 0 0.1 100 2 1.5 5 9 20]; %#ok<*PROP>
            [tau,idx] = obj.s2tau(s);

            [tau0,tau1] = obj.domain();
            tauSet = interp1([0; obj.cumlengths()], tau0:tau1, s, 'linear','extrap');
            verifyEqual(testCase, tau, tauSet, 'AbsTol',2e-14);
            
            verifyEqual(testCase, idx, uint32([1 1 1 1 3 3 2 3 3]));
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
