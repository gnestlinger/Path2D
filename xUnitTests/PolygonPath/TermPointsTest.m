classdef TermPointsTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'emptyPath', PolygonPath(), ...
            'nonEmptyPath', PolygonPath.xy2Path(0:10, zeros(1,11)));
    end
    
    methods (Test)
        function testSize(testCase, obj)
            [P0,P1] = obj.termPoints();
            
            testCase.verifySize(P0, [2 1]);
            testCase.verifySize(P1, [2 1]);
            
            if isempty(obj)
                set = [NaN; NaN];
                testCase.verifyEqual(P0, set) 
                testCase.verifyEqual(P1, set)
            end
        end%fcn
    end
    
end%class
