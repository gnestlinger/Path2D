classdef S2TauTestPolygon < matlab.unittest.TestCase
    
    properties (TestParameter)
        Path = struct(...
            'emptyPath', PolygonPath(), ...
            'oneElmPath', PolygonPath(1, 2, pi/4, 0), ...
            'nonEmptyPath', PolygonPath.xy2Path(0:10, zeros(1,11)))
        s = struct(...
            'emptyTau', [], ...
            'oneElmTau', 1.1, ...
            'vectorTau', linspace(-1,10,24), ...
            'ndTau', ones(10,2,3))
    end
    
    
    
    methods (Test)
        function testReturnValuesZeroLengthPath(testCase, s)
            % Empty path
            obj = PolygonPath();
            [tau,idx] = obj.s2tau(s);
            verifyEqual(testCase, tau, nan(size(s)));
            verifyEqual(testCase, idx, zeros(size(s), 'uint32'));
            
            % Non-empty but zero length path
            obj = PolygonPath(1,2,3,4);
            [tau,idx] = obj.s2tau(s);
            verifyEqual(testCase, tau, nan(size(s)));
            verifyEqual(testCase, idx, zeros(size(s), 'uint32'));
        end%fcn
        
        function testReturnValues(testCase)
            obj = PolygonPath.xy2Path([-3 1 8 34], [3 1 0 6]);
            s = [-1 -0.2 0 0.1 100 2 4 5 9 20]; %#ok<*PROP>
            [tau,idx] = obj.s2tau(s);
            
            [tau0,tau1] = obj.domain();
            cl = [0 obj.cumlengths()'];
            tauSet = interp1(cl, tau0:tau1, s, 'linear','extrap');
            
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
