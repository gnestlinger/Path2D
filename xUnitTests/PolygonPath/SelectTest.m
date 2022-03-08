classdef SelectTest < matlab.unittest.TestCase
    
    methods (Test)		
		function testNonDecreasingPathLength(testCase)
			
            % Create circle with radius 3
			obj0 = PolygonPath.circle(3);
			obj1 = obj0.select(numel(obj0):-1:1);
			
			% Path length can not decrease
			verifyTrue(testCase, ~any(diff(obj1.getPathLengths()) < 0));
			
			verifyEqual(testCase, max(abs(obj1.head-flip(obj0.head))), 0);
            
		end%fcn
    end
    
end%class
