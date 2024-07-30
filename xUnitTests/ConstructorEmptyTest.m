classdef ConstructorEmptyTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'PolygonPath', PolygonPath(), ...
            'SplinePath', SplinePath());
    end
    

    methods (Test)

        function testConstructorInputLength(testCase, obj)
            testCase.verifyEqual(obj.numel(), 0)
        end%fcn
        
    end
    
end%class
