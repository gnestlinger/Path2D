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
            [x,y,tauO,head,curv,curvDs] = obj.eval();
            
            N = numel(tauO);
            testCase.verifySize(x, [N 1]);
            testCase.verifySize(y, [N 1]);
            testCase.verifySize(tauO, [N 1]);
            testCase.verifySize(head, [N 1]);
            testCase.verifySize(curv, [N 1]);
            testCase.verifySize(curvDs, [N 1]);
        end%fcn
        
        function testReturnSize(testCase, obj, tau)
            [x,y,tauO,head,curv,curvDs] = obj.eval(tau);
            
            N = numel(tau);
            testCase.verifySize(x, [N 1]);
            testCase.verifySize(y, [N 1]);
            testCase.verifySize(tauO, [N 1]);
            testCase.verifySize(head, [N 1]);
            testCase.verifySize(curv, [N 1]);
            testCase.verifySize(curvDs, [N 1]);
        end%fcn
        
        function testReturnValues(testCase, obj)
            t = -2:0.5:2;
            [x,y,tauOut,h,c,dc] = obj.eval(t);
            
            % Text x, y, heading, curvature
            if isempty(obj)
                N = numel(t);
                testCase.verifyEqual(x, NaN(N,1));
                testCase.verifyEqual(y, NaN(N,1));
                testCase.verifyEqual(h, NaN(N,1));
                testCase.verifyEqual(c, NaN(N,1));
                testCase.verifyEqual(dc, NaN(N,1));
            elseif obj.length() < eps % Path defined at a single point
                testCase.verifyEqual(x, [NaN NaN NaN NaN 1 NaN NaN NaN NaN]');
                testCase.verifyEqual(y, [NaN NaN NaN NaN 2 NaN NaN NaN NaN]');
                testCase.verifyEqual(h, [NaN NaN NaN NaN pi/4 NaN NaN NaN NaN]');
                testCase.verifyEqual(c, [NaN NaN NaN NaN 0 NaN NaN NaN NaN]');
                testCase.verifyEqual(dc, [NaN NaN NaN NaN 0 NaN NaN NaN NaN]');
            else
                testCase.verifyEqual(x, [NaN NaN NaN NaN 0.0 0.5 1.0 1.5 2.0]');
                testCase.verifyEqual(y, [NaN NaN NaN NaN 0.0 0.0 0.0 0.0 0.0]');
                testCase.verifyEqual(h, [NaN NaN NaN NaN 0.0 0.0 0.0 0.0 0.0]');
                testCase.verifyEqual(c, [NaN NaN NaN NaN 0.0 0.0 0.0 0.0 0.0]');
                testCase.verifyEqual(dc, [NaN NaN NaN NaN 0.0 0.0 0.0 0.0 0.0]');
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
        
        function testExtrapolationPolygon(testCase, obj)
            
            if ~isa(obj, 'PolygonPath')
                return
            end
            
            % Evaluate inside and outside of the path's domain
            [tau0,tau1] = obj.domain();
            tauEval = [tau0-3 tau0-1 tau0:1:tau1 tau1+1 tau1+3];
            isTauExtrap = (tauEval < tau0) | (tauEval > tau1);
            N = numel(tauEval);
            
            [x,y,t,h,c,d] = obj.eval(tauEval, true);
            if obj.isempty() % Nothing to extrapolate
                xSet = NaN(N, 1);
                ySet = NaN(N, 1);
                tSet = NaN(N, 1);
                hSet = NaN(N, 1);
                cSet = NaN(N, 1);
                dSet = NaN(N, 1);
                
            elseif (obj.length() < eps)
                % A single point PolygonPath is not extrapolated
                xSet = NaN(N, 1);
                ySet = NaN(N, 1);
                tSet = NaN(N, 1);
                hSet = NaN(N, 1);
                cSet = NaN(N, 1);
                dSet = NaN(N, 1);
                
                xSet(~isTauExtrap) = obj.x;
                ySet(~isTauExtrap) = obj.y;
                tSet(~isTauExtrap) = tauEval(~isTauExtrap);
                hSet(~isTauExtrap) = obj.head;
                cSet(~isTauExtrap) = obj.curv;
                dSet(~isTauExtrap) = 0; % Zero path length -> set dCurv/dS to 0
                
            else
                xyhcdSet = interp1(0:obj.numel()-1, ...
                    [obj.x obj.y obj.head obj.curv gradient(obj.curv)./gradient(obj.cumlengths())], ...
                    tauEval, 'linear', 'extrap');
                xSet = xyhcdSet(:,1);
                ySet = xyhcdSet(:,2);
                tSet = tauEval(:);
                hSet = xyhcdSet(:,3);
                cSet = xyhcdSet(:,4);
                dSet = xyhcdSet(:,5);
            end
            
            %%% Checks
            testCase.verifyEqual(x, xSet);
            testCase.verifyEqual(y, ySet);
            testCase.verifyEqual(t, tSet);
            testCase.verifyEqual(h, hSet);
            testCase.verifyEqual(c, cSet);
            testCase.verifyEqual(d, dSet);
            
        end%fcn
        
        function testExtrapolationSpline(testCase, obj)
            
            if ~isa(obj, 'SplinePath')
                return
            end
            
            % Evaluate inside and outside of the path's domain
            [tau0,tau1] = obj.domain();
            tauEval = [tau0-3 tau0-1 tau0:1:tau1 tau1+1 tau1+3];
            N = numel(tauEval);
            
            [x,y,t,h,c,d] = obj.eval(tauEval, true);
            if obj.isempty() % Nothing to extrapolate
                xSet = NaN(N, 1);
                ySet = NaN(N, 1);
                tSet = NaN(N, 1);
                hSet = NaN(N, 1);
                cSet = NaN(N, 1);
                dSet = NaN(N, 1);
                
            else
                xySet = ppval(obj.mkpp(), tauEval)';
                xSet = xySet(:,1);
                ySet = xySet(:,2);
                tSet = tauEval(:);
                % Test path is a straight line, therefore we can get the
                % heading from x/y values and curvature/curvDs are zero
                hSet = atan2(gradient(ySet), gradient(xSet));
                cSet = zeros(numel(tauEval), 1);
                dSet = zeros(numel(tauEval), 1);
            end
            
            %%% Checks
            testCase.verifyEqual(x, xSet);
            testCase.verifyEqual(y, ySet);
            testCase.verifyEqual(t, tSet);
            testCase.verifyEqual(h, hSet);
            testCase.verifyEqual(c, cSet);
            testCase.verifyEqual(d, dSet);
            
        end%fcn
        
    end
    
end%class
