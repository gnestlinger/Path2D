classdef ReverseTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'emptyPath', PolygonPath(), ...
            'oneElmPath', PolygonPath(1, 2, pi/4, 0), ...
            'nonEmptyPath', PolygonPath.xy2Path([0 1 3 4 10], zeros(1,5)));
    end
    
    methods (Test)
        function testLength(testCase, obj)
            l0 = obj.length();
            l1 = obj.reverse().length();
            
            % Length is invariant 
            testCase.verifyEqual(l0, l1);
        end%fcn
        
        function testTerminalPoints(testCase, obj)
            [A0,A1] = obj.termPoints();
            [B0,B1] = obj.reverse().termPoints();
            
            % Terminal points are flipped
            testCase.verifyEqual(A0, B1);
            testCase.verifyEqual(A1, B0);
        end%fcn
    end
    
end%class
