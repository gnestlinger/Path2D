classdef SelectTestSpline < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'Stairstep',SplinePath(0:5, cat(3, [ones(1,5); zeros(1,5)], [0:4; 0:4])), ...
            'Continuous',SplinePath(0:5, cat(3, [ones(1,5); ones(1,5)], [0:4; 0:4])));
        
        idx = struct(...
            'SingleDouble', 2, ...
            'SingleLogical', [false true false false false], ...
            'MultipleDouble', [2 3 4], ...
            'MultipleLogical', [false true true true false]);
    end%properties
    
    
    methods (Test)
        
		function testIndex(testCase, obj, idx)
			
			objs = obj.select(idx);
            % Explicitly call class constructor for checking breaks and
            % coefficients
            objs = SplinePath(objs.Breaks, objs.Coefs);
%             plot([obj, objs], '.')
            
            % Check actual path values; due to the discontinuity of the
            % stairstep path's y-component, the end value is different 
            [xs,ys,taus] = objs.eval();
            [x0,y0,tau0] = obj.eval(taus);
            testCase.verifyEqual(xs, x0);
            testCase.verifyEqual(ys(1:end-1), y0(1:end-1));
            testCase.verifyEqual(taus, tau0);
			
			% Path length can not decrease
			testCase.verifyTrue(~any(diff(objs.cumlengths()) < 0));
		end%fcn
        
        function testErrSubscript(testCase, obj)
            testCase.verifyError(@() obj.select(obj.numel() + 1), ...
                'SplinePath:BadSubscript')
        end%fcn
        
        function testErrReordering(testCase, obj)
            testCase.verifyError(@() obj.select([2 1]), ...
                'SplinePath:PiecesReorderingUnsupported')
        end%fcn
        
        function testErrSkipping(testCase, obj)
            testCase.verifyError(@() obj.select([1 3]), ...
                'SplinePath:PiecesSkippingUnsupported')
        end%fcn
    end
    
end%class
