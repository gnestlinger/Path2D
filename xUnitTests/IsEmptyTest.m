classdef IsEmptyTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        PathEmpty = struct(...
            'PolygonPath', PolygonPath(), ...
            'SplinePath', SplinePath(), ...
            'DubinsPath', DubinsPath())
        
        PathNonEmpty = struct(...
            'PolygonPath',PolygonPath(1, 2, 3, 4, 5), ...
            'SplinePath', SplinePath([0 1], zeros(2,1,2)), ...
            'DubinsPath', DubinsPath([0 0 0], 0, 1, 3))
    end
    
    
    methods (Test)
        
        function testIsEmptyTrue(testCase, PathEmpty)
            testCase.verifyTrue(PathEmpty.isempty())
        end%fcn
        
        function testIsEmptyFalse(testCase, PathNonEmpty)
            testCase.verifyFalse(PathNonEmpty.isempty())
        end%fcn
        
    end
    
end%class
