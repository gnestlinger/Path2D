classdef EvalTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'PolygonPathEmpty', PolygonPath([], [], [], []), ...
            'PolygonPathOneElm', PolygonPath(1, 2, pi/4, 0), ...
            'PolygonPathNonEmpty', PolygonPath.xy2Path(0:10, zeros(1,11)), ...
            'SplinePathEmpty', SplinePath(), ...
            'SplinePathNonEmpty', SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0], 2,2,2)));
        
        tau = struct(...
            'empty', zeros(0,1), ...
            'scalar', 0, ...
            'vector', 0:0.1:1, ...
            'matrix', rand(10,10), ...
            'nd', rand(10,3,4));
    end
    
    methods (Test)
        
        function testReturnSizeNoArg(testCase, obj)
            
			[x,y,tauO,head,curv] = obj.eval();
            
            testCase.verifyTrue(iscolumn(tauO));
            sz = size(tauO);
			testCase.verifySize(x, sz);
            testCase.verifySize(y, sz);
            testCase.verifySize(head, sz);
            testCase.verifySize(curv, sz);
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
        
    end
    
end%class
