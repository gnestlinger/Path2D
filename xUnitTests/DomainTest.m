classdef DomainTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        PathEmpty = struct(...
            'PolygonPath', PolygonPath(), ...
            'SplinePath', SplinePath());
        
        PathNonEmpty = struct(...
            'PolygonPath', PolygonPath.xy2Path(0:10, zeros(1,11)), ...
            'SplinePath', SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0], 2,2,2)));
    end
    
    
    methods (Test)
        
        function testPathEmpty(testCase, PathEmpty)
            [tauL,tauU] = PathEmpty.domain();
            
            verifyEqual(testCase, tauL, NaN);
            verifyEqual(testCase, tauU, NaN);
        end%fcn
        
        function testPathNonEmpty(testCase, PathNonEmpty)
            [tauL,tauU] = PathNonEmpty.domain();
            
            verifySize(testCase, tauL, [1 1]);
            verifySize(testCase, tauU, [1 1]);
            verifyTrue(testCase, tauL < tauU);
        end%fcn
        
    end
    
end%class
