classdef AppendTestSpline < matlab.unittest.TestCase

    methods (Test)
        function testAppend(testCase)
            
            % Join two "arbitrary" single segment paths with different
            % polynomial degrees
            obj0 = SplinePath([0 1], reshape([1 2 1 0; 0 1 2 1], [2 1 4]));
            [~,P1]= obj0.termPoints();
            obj1 = SplinePath.straight(P1, P1 + [10; 5]);
            obj = obj0.append(obj1);
            
            % Evaluation of appended path must return the same values as
            % evaluation of the original paths
            tau = linspace(0, 1, 101);
            [xAct,yAct] = obj.eval(tau);
            [xExp,yExp] = obj0.eval(tau);
            verifyEqual(testCase, xAct, xExp);
            verifyEqual(testCase, yAct, yExp);
            
            [xAct,yAct] = obj.eval(tau + obj.Breaks(2));
            [xExp,yExp] = obj1.eval(tau);
            verifyEqual(testCase, xAct, xExp, 'AbsTol',2e-15);
            verifyEqual(testCase, yAct, yExp, 'AbsTol',2e-15);
        end%fcn
    end
    
end%class
