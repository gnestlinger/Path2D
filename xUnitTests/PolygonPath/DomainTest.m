classdef DomainTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'emptyPath', PolygonPath(), ...
            'nonEmptyPath', PolygonPath.xy2Path(0:10, zeros(1,11)));
    end
    
    methods (Test)
        
        function testNonemptyPath(testCase, obj)
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
