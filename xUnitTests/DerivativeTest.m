classdef DerivativeTest < matlab.unittest.TestCase
    
    properties (TestParameter)
       	PathObj = struct(...
            'PolygonPathEmpty', PolygonPath([], [], [], []), ...
            'PolygonPathZeroLength', PolygonPath(1, 2, pi/4, 0), ...
            'PolygonPathNonEmpty', PolygonPath.xy2Path(0:10, zeros(1,11)))
    end
    
    
    
    methods (Test)
        
        function testDerivative(testCase, PathObj)
            
            objd = PathObj.derivative();
            
            % The domain stays the same
            [a,b] = PathObj.domain();
            [c,d] = objd.domain();
            verifyEqual(testCase, [a b], [c d]);
        end%fcn

    end
    
end%class
