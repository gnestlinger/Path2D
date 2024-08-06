classdef LengthTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        PathEmpty = struct(...
            'PolygonPath', PolygonPath(), ...
            'SplinePath', SplinePath() ...
            )
        
        PathNonEmpty = {...
            {PolygonPath.xy2Path(0:10, zeros(1,11)), 10};
            {SplinePath.pp2Path(...
            mkpp([-1 0 1 2], [0 1 -1; 1 -2 1; 0 1 0; 1 0 0; 0 1 1; 0 0 1], 2)), ...
            0.5*(2*sqrt(5) + asinh(2)) + 1}; % via wolframalpha ...
            }
    end
    
    
    methods (Test)
        
        function testPathEmpty(testCase, PathEmpty)
            verifyEqual(testCase, PathEmpty.length(), 0);
        end%fcn
        
        function testPathNonEmpty(testCase, PathNonEmpty)
            obj = PathNonEmpty{1};
            lSet = PathNonEmpty{2};
            verifyEqual(testCase, obj.length(), lSet, 'RelTol',0.001);
        end%fcn
        
    end
    
end%class
