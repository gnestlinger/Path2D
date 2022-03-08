classdef ConstructorTest < matlab.unittest.TestCase
    
    methods (Test)
        function testConstructorInputLength(testCase)
            x = 1:10;
			
			testCase.verifyError(@() PolygonPath(x(1:end-1), x, x, x), ...
				'PolygonPath:Constructor:numelXYHC');
			
			testCase.verifyError(@() PolygonPath(x, x(1:end-1), x, x), ...
				'PolygonPath:Constructor:numelXYHC');
			
			testCase.verifyError(@() PolygonPath(x, x, x(1:end-1), x), ...
				'PolygonPath:Constructor:numelXYHC');
			
			testCase.verifyError(@() PolygonPath(x, x, x, x(1:end-1)), ...
				'PolygonPath:Constructor:numelXYHC');
		end%fcn
    end
    
end%class
