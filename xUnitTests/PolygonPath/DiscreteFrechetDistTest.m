classdef DiscreteFrechetDistTest < matlab.unittest.TestCase
    
    methods (Test)
        function testSimple(testCase)
            obj = PolygonPath.xy2Path([0 1 2 3], [0 1 2 3]);
            Q = [0 1 2 3; 4 5 6 7]';
            
            d = obj.discreteFrechetDist(Q);
            
            verifyEqual(testCase, d, 4)
        end%fcn
        
        function testUnequalNbrOfSamples(testCase)
            obj = PolygonPath.xy2Path([0 1 2 3], [0 1 2 3]);
            Q = [0  3; 4  4]';
            
            d = obj.discreteFrechetDist(Q);
            
            verifyEqual(testCase, d, 4)
        end%fcn
    end
    
end%class
