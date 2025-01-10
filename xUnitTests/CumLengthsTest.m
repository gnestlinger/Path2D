classdef CumLengthsTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        PathEmpty = struct(...
            'PolygonPath', PolygonPath(), ...
            'SplinePath', SplinePath(), ...
            'DubinsPath', DubinsPath())
        
        PathNonEmpty = struct(...
            'PolygonPathZeroLength', PolygonPath(0, 0, 0, 0, 0), ...
            'SplinePathZeroLength', SplinePath([1 1], reshape([1 0;1 1], [2 1 2])), ...
            'DubinsPathZeroLength', DubinsPath([1 0 pi], 0, 0, 2), ...
            'PolygonPath', PolygonPath.xy2Path(0:10, zeros(1,11)), ...
            'SplinePath', SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0], 2,2,2)), ...
            'DubinsPath', DubinsPath([1 0 pi], [1 -1 0], [2*pi/2 2*pi 2], 2))
    end
    
    
    
    methods (Test)
        
        function testPathEmpty(testCase, PathEmpty)
            lens = PathEmpty.cumlengths();
            verifyEmpty(testCase, lens);
        end%fcn
        
        function testPathNonEmpty(testCase, PathNonEmpty)
            lens = PathNonEmpty.cumlengths();
            
            % For a non-empty path, we expect a size of N-by-1, where N is
            % the number of path segments.
            verifySize(testCase, lens, [PathNonEmpty.numel() 1]);
        end%fcn
        
    end
    
end%class
