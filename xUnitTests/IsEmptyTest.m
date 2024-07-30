classdef IsEmptyTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        objEmpty = struct(...
            'PolygonPath', PolygonPath(), ...
            'SplinePath', SplinePath(), ...
            'DubinsPath', DubinsPath())
        
        objNonEmpty = struct(...
            'PolygonPath',PolygonPath(1, 2, 3, 4, 5), ...
            'SplinePath', SplinePath([0 1], zeros(2,1,2)), ...
            'DubinsPath', DubinsPath([0 0 0], 0, 1, 3))
    end
    
    
    methods (Test)
        
        function testIsEmptyTrue(testCase, objEmpty)
            testCase.verifyTrue(objEmpty.isempty())
        end%fcn
        
        function testIsEmptyFalse(testCase, objNonEmpty)
            testCase.verifyFalse(objNonEmpty.isempty())
        end%fcn
        
    end
    
end%class
