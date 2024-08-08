classdef CumLengthsTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        PathEmpty = struct(...
            'PolygonPath', PolygonPath(), ...
            'SplinePath', SplinePath())
        
        PathNonEmpty = struct(...
            'PolygonPath', PolygonPath.xy2Path(0:10, zeros(1,11)), ...
            'SplinePath', SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0], 2,2,2)))
    end
    
    
    
    methods (Test)
        
        function testPathEmpty(testCase, PathEmpty)
            lens = PathEmpty.cumlengths();
            verifyEmpty(testCase, lens);
        end%fcn
        
        function testPathNonEmpty(testCase, PathNonEmpty)
            lens = PathNonEmpty.cumlengths();
            
            verifySize(testCase, lens, [PathNonEmpty.numel() 1]);
        end%fcn
        
    end
    
end%class
