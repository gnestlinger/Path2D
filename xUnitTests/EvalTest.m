classdef EvalTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'PolygonPathEmpty', PolygonPath([], [], [], []), ...
            'SplinePathEmpty', SplinePath(), ...
            'PolygonPathZeroLength', PolygonPath(1, 2, pi/4, 0), ...
            'SplinePathZeroLength', SplinePath([0 0], reshape([1 1; 1 2], [2 1 2])), ...
            'PolygonPathNonEmpty', PolygonPath.xy2Path(0:10, zeros(1,11)), ...
            'SplinePathNonEmpty', SplinePath([0 10], reshape([1 0; 0 0],  [2 1 2])));
        
        tau = struct(...
            'empty', zeros(0,1), ...
            'scalar', 1, ...
            'vector', -10:0.5:10, ...
            'matrix', randn(10,10)*10, ...
            'nd', randn(10,3,4)*10);
    end
    
    methods (Test)
        
        function testReturnSizeNoTau(testCase, obj)
            [x,y,tauO,head,curv] = obj.eval();
            
            N = numel(tauO);
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
            [x,y,tauOut,h,c] = obj.eval(t);
            
            % Text x, y, heading, curvature
            if isempty(obj)
                N = numel(t);
                testCase.verifyEqual(x, NaN(N,1));
                testCase.verifyEqual(y, NaN(N,1));
                testCase.verifyEqual(h, NaN(N,1));
                testCase.verifyEqual(c, NaN(N,1));
            elseif obj.length() < eps % Path defined at a single point
                testCase.verifyEqual(x, [NaN NaN NaN NaN 1 NaN NaN NaN NaN]');
                testCase.verifyEqual(y, [NaN NaN NaN NaN 2 NaN NaN NaN NaN]');
                testCase.verifyEqual(h, [NaN NaN NaN NaN pi/4 NaN NaN NaN NaN]');
                testCase.verifyEqual(c, [NaN NaN NaN NaN 0 NaN NaN NaN NaN]');
            else
                testCase.verifyEqual(x, [NaN NaN NaN NaN 0.0 0.5 1.0 1.5 2.0]');
                testCase.verifyEqual(y, [NaN NaN NaN NaN 0.0 0.0 0.0 0.0 0.0]');
                testCase.verifyEqual(h, [NaN NaN NaN NaN 0.0 0.0 0.0 0.0 0.0]');
                testCase.verifyEqual(c, [NaN NaN NaN NaN 0.0 0.0 0.0 0.0 0.0]');
            end
            
            % Test tau
            tSet = t(:);
            if obj.isempty()
                tSet(:) = NaN;
            else
                [tau0,tau1] = obj.domain();
                tSet(tSet<tau0 | tSet>tau1) = NaN;
            end
            testCase.verifyEqual(tauOut, tSet);
        end%fcn
        
    end
    
end%class
