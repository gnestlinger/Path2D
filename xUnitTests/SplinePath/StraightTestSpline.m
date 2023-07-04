classdef StraightTestSpline < matlab.unittest.TestCase
    
    methods (Test)
        
        function testCreateStraight(testCase)
            
            P0 = [-1 4];
            P1 = [34 89];
            obj = SplinePath.straight(P0, P1);
            
            % Derivative via class method
            [p0,p1] = obj.termPoints();
            
            % Checks
            testCase.verifyEqual(P0(:), p0);
            testCase.verifyEqual(P1(:), p1);
            
        end%fcn
        
    end
    
end%class
