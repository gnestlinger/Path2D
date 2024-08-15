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
        function testReturnSize(testCase, Path, s)
            [tau,idx,ds] = Path.s2tau(s);
            
            testCase.verifySize(tau, size(s));
            testCase.verifySize(idx, size(s));
            testCase.verifySize(ds, size(s));
        end%fcn
        
        function testReturnValuesZeroLengthPath(testCase, s)
            % Empty path
            obj = PolygonPath();
            [tau,idx,ds] = obj.s2tau(s);
            testCase.verifyEqual(tau, nan(size(s)));
            testCase.verifyEqual(idx, zeros(size(s), 'uint32'));
            testCase.verifyEqual(ds, nan(size(s)));
            
            % One-element path
            obj = PolygonPath(1,2,3,4);
            [tau,idx,ds] = obj.s2tau(s);
            testCase.verifyEqual(tau, nan(size(s)));
            testCase.verifyEqual(idx, zeros(size(s), 'uint32'));
            testCase.verifyEqual(ds, nan(size(s)));
        end%fcn
        
        function testReturnValues(testCase)
            obj = PolygonPath.xy2Path([-3 1 8 34], [3 1 0 6]);
            s = [-1 -0.2 0 0.1 100 2 4 5 9 20]; %#ok<*PROP>
            [tau,idx,ds] = obj.s2tau(s);
            
            [tau0,tau1] = obj.domain();
            cl = [0 obj.cumlengths()'];
            tauSet = interp1(cl, tau0:tau1, s, 'linear','extrap');
            dsSet  = s - cl(idx);
            
            testCase.verifyEqual(tau, tauSet, 'AbsTol',1e-15)
            testCase.verifyEqual(ds, dsSet)
        end%fcn
    end
    
end%class
