classdef StraightTestSpline < matlab.unittest.TestCase
    
    properties (TestParameter)
        P0 = {[0 0], [10 10], [-20 45], [-1e3 0]};
        
        P1 = {[0 0], [1; 1], [88 99]};
    end
    
    
    methods (Test)
        
        function testCreateStraight(testCase, P0, P1)
            
            sSet = hypot(P1(1) - P0(1), P1(2) - P0(2));
            obj = SplinePath.straight(P0, P1);
            
            % Derivative via class method
            [P0Act,P1Act] = obj.termPoints();
            sAct = obj.length();
            
            % Checks
            testCase.verifyEqual(P0(:), P0Act);
            testCase.verifyEqual(P1(:), P1Act);
            testCase.verifyEqual(sSet, sAct);
            
        end%fcn
        
    end
    
end%class
