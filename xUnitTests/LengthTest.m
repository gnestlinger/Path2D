classdef LengthTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'PolygonPathEmpty', PolygonPath(), ...
            'PolygonPathNonEmpty', PolygonPath.xy2Path(0:10, zeros(1,11)))
    end
    
    methods (Test)
        
        function testPath(testCase, obj)
            l = length(obj);
            
            if isempty(obj)
                testCase.verifyTrue(l == 0);
            else
                testCase.verifyTrue(l > 0);
            end
        end%fcn
        
    end
    
end%class
