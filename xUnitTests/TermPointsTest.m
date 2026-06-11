classdef TermPointsTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        PathEmpty = struct(...
            'PolygonPath', PolygonPath(), ...
            'SplinePath', SplinePath(), ...
            'DubinsPath', DubinsPath())
        
        PathNonEmpty = {...
            {PolygonPath.xy2Path(0:10, zeros(1,11)), [0;0], [10;0]}; 
            {SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0], 2,2,2)), [0;0], [2;1]}; 
            {DubinsPath([1 -1 0], [1 -1 0], [2*pi/2 2*pi 2], 2), [1;-1], [7;-1]}; 
            }
    end
    
    methods (Test)
        
        function testSizeEmpty(testCase, PathEmpty)
            [P0,P1] = PathEmpty.termPoints();
                        
            excpected = [NaN; NaN];
            verifyEqual(testCase, P0, excpected) 
            verifyEqual(testCase, P1, excpected)
        end%fcn
        
        function testSizeNonEmpty(testCase, PathNonEmpty)
            obj = PathNonEmpty{1};
            P0Set = PathNonEmpty{2};
            P1Set = PathNonEmpty{3};
            [P0,P1] = obj.termPoints();
            
            verifyEqual(testCase, P0, P0Set);
            verifyEqual(testCase, P1, P1Set, 'AbsTol',3e-16);
        end%fcn
        
    end
    
end%class
