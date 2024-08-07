classdef ConstructorEmptyTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        PathEmpty = struct(...
            'PolygonPath', PolygonPath(), ...
            'SplinePath', SplinePath(), ...
            'DubinsPath', DubinsPath());
    end
    

    methods (Test)
        
        function testConstructorNoArgs(testCase, PathEmpty)
        % Test that contructor with zero arguments returns an empty path
            
            verifyEqual(testCase, PathEmpty.numel(), 0);
            verifyTrue(testCase, PathEmpty.isempty());
        end%fcn
        
    end
    
end%class
