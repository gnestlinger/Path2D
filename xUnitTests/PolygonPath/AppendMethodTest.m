classdef AppendMethodTest < matlab.unittest.TestCase
    
    methods (Test)
        function testNonDecreasingPathLength(testCase)
            x = 1:10;
			obj = PolygonPath.xy2Path(x, x);
			objA = obj.append(obj);
			
			testCase.verifyTrue(~any(diff(objA.getPathLengths() < 0)))
		end%fcn
		
    end
    
end%class