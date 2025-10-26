classdef InterpTest < matlab.unittest.TestCase
    
    methods (Test)
        function testInterpStraight(testCase)
        % Test interpolation of a straight path.
            
            obj0 = PolygonPath.straight([0 0], [20 2]);
            [tau0,tau1] = obj0.domain();
            obj1 = obj0.interp(linspace(tau0, tau1, 100));
            
            % The terminal points must match
            [exp0,exp1] = obj0.termPoints();
            [act0,act1] = obj1.termPoints();
            verifyEqual(testCase, act0, exp0);
            verifyEqual(testCase, act1, exp1);
            
            % The length must match
            verifyEqual(testCase, obj1.length(), obj0.length())
            
        end%fcn
    end
    
end%class
