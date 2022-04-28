classdef EmptyTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'PolygonPath', PolygonPath.empty());
    end
    
    methods (Test)
        function testConstructorInputLength(testCase, obj)
            testCase.verifyEqual(numel(obj), 0)
        end%fcn
    end
    
end%class
