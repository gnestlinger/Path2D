classdef TermPointsTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'PolygonPathEmpty', PolygonPath(), ...
            'PolygonPathNonEmpty', PolygonPath.xy2Path(0:10, zeros(1,11)), ...
            'SplinePathEmpty', SplinePath(), ...
            'SplinePathNonEmpty', SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0], 2,2,2)));
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
