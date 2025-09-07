classdef Bezier2PathSpline < matlab.unittest.TestCase
    
    properties (TestParameter)
        CtrlPts = struct(...
            'SingleSegment', [0 0; 2 0; 8 1; 10 1]', ...
            'TwoSegments', cat(3, [0 0; 2 0; 5 1; 10 1]', [10 1; 15 1; 18 0; 20 0]'))
    end
    
    
    
    methods (Test)
        function testTerminalPoints(testCase, CtrlPts)
        % Test terminal points of path.
            
            P0Exp = CtrlPts(:,1,1);
            P1Exp = CtrlPts(:,end,end);
            
            obj = SplinePath.bezier2Path(CtrlPts);
            [P0,P1] = obj.termPoints();
            
            verifyEqual(testCase, P0, P0Exp);
            verifyEqual(testCase, P1, P1Exp);
        end%fcn
        
        function errorAtWrongDims(testCase, CtrlPts)
        % Test for error if first dimension of control points does not
        % equal two.
        
            P = permute(CtrlPts, [2 1 3]);
            verifyError(testCase, @() SplinePath.bezier2Path(P), 'SplinePath:bezier2Path')
        end%fcn
    end
    
end%class
