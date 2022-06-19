classdef EvalTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'emptyPath', PolygonPath([], [], [], []), ...
            'oneElmPath', PolygonPath(1, 2, pi/4, 0), ...
            'nonEmptyPath', PolygonPath.xy2Path(0:10, zeros(1,11)));
        
        tau = struct(...
            'empty', zeros(0,1), ...
            'scalar', 1, ...
            'vector', -10:0.5:10, ...
            'matrix', randn(10,10)*10, ...
            'nd', randn(10,3,4)*10);
    end
    
    methods (Test)
        
        function testReturnSizeNoArg(testCase, obj)
            [x,y,tauO,head,curv] = obj.eval();
            
            N = numel(obj);
            testCase.verifySize(x, [N 1]);
            testCase.verifySize(y, [N 1]);
            testCase.verifySize(tauO, [N 1]);
            testCase.verifySize(head, [N 1]);
            testCase.verifySize(curv, [N 1]);
        end%fcn
        
        function testReturnSize(testCase, obj, tau)
            [x,y,tauO,head,curv] = obj.eval(tau);
            
            N = numel(tau);
            testCase.verifySize(x, [N 1]);
            testCase.verifySize(y, [N 1]);
            testCase.verifySize(tauO, [N 1]);
            testCase.verifySize(head, [N 1]);
            testCase.verifySize(curv, [N 1]);
        end%fcn
        
        function testReturnValues(testCase, obj)
            t = -2:0.5:2;
            N = numel(t);
            [x,y] = obj.eval(t);
            if isempty(obj)
                testCase.verifyEqual(x, NaN(N, 1));
                testCase.verifyEqual(y, NaN(N, 1));
            elseif obj.numel() < 2
                testCase.verifyEqual(x, [NaN NaN NaN NaN 1 NaN NaN NaN NaN]');
                testCase.verifyEqual(y, [NaN NaN NaN NaN 2 NaN NaN NaN NaN]');
            else
                testCase.verifyEqual(x, [NaN NaN NaN NaN 0 0.5 1 1.5 2]');
                testCase.verifyEqual(y, [NaN NaN NaN NaN 0 0.0 0 0.0 0]');
            end
        end%fcn
        
    end
    
end%class
