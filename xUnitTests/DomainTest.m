classdef DomainTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        PathEmpty = struct(...
            'PolygonPath', PolygonPath(), ...
            'SplinePath', SplinePath(), ...
            'DubinsPath', DubinsPath())
        
        PathZeroLength = struct(... % Non-empty but zero length
            'PolygonPath', PolygonPath(1, 1, 0, 0), ...
            'SplinePath', SplinePath([0 0], reshape([1 0; 0 0], [2 1 2])), ...
            'DubinsPathZeroLength', DubinsPath([1 0 pi], 0, 0, 2))
        
        PathNonEmpty = struct(...
            'PolygonPath', PolygonPath.xy2Path(0:10, zeros(1,11)), ...
            'SplinePath', SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0], [2 2 2])), ...
            'DubinsPath', DubinsPath([1 0 pi], [1 -1 0], [2*pi/2 2*pi 2], 2))
    end
    
    
    methods (Test)
        
        function testPathEmpty(testCase, PathEmpty)
            [tauL,tauU] = PathEmpty.domain();
            
            verifyEqual(testCase, tauL, NaN);
            verifyEqual(testCase, tauU, NaN);
        end%fcn
        
        function testPathZeroLength(testCase, PathZeroLength)
            [tauL,tauU] = PathZeroLength.domain();
            
            verifySize(testCase, tauL, [1 1]);
            verifySize(testCase, tauU, [1 1]);
            verifyEqual(testCase, tauL, tauU);
        end%fcn
        
        function testPathNonEmpty(testCase, PathNonEmpty)
            [tauL,tauU] = PathNonEmpty.domain();
            
            verifySize(testCase, tauL, [1 1]);
            verifySize(testCase, tauU, [1 1]);
            verifyTrue(testCase, tauL < tauU);
        end%fcn
        
    end
    
end%class
