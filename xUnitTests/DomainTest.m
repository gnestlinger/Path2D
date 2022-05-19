classdef DomainTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'PolygonPathEmpty', PolygonPath(), ...
            'PolygonPathNonEmpty', PolygonPath.xy2Path(0:10, zeros(1,11)), ...
            'SplinePathEmpty', SplinePath(), ...
            'SplinePathNonEmpty', SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0], 2,2,2)));
    end
    
    methods (Test)
        
        function testPath(testCase, obj)
            [tauL,tauU] = domain(obj);
            
            testCase.verifySize(tauL, [1 1]);
            testCase.verifySize(tauU, [1 1]);
            
            if isempty(obj)
               testCase.verifyEqual(tauL, NaN) 
               testCase.verifyEqual(tauU, NaN)
            end
        end%fcn
        
    end
    
end%class
