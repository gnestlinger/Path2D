classdef RDPTest < matlab.unittest.TestCase
% Test recursive and iterative implementations of Ramer-Douglas-Peucker
% algorithm.

    properties (TestParameter)
        epsilon = {0.02 0.1 0.5 1}
    end

    
    
    methods (TestClassSetup)
        function classSetup(testCase)
            orig = rng;
            addTeardown(testCase, @rng, orig)
            rng(0)
        end%fcn
    end

    methods (Test)
		function testClosedPath(testCase)
            
            % Create closed polygon path
            obj0 = PolygonPath.xy2Path(...
                [0:10, 10*ones(1,10), linspace(10,0,10)], ...
                [zeros(1,11), 1:10, linspace(10,0,10)]);
            assert(obj0.IsCircuit)
			xExp = [0 10 10 0];
            yExp = [0 0 10 0];
            
            obj1 = obj0.rdp(0.1);
            obj2 = obj0.rdpIter(0.1);
            
            verifyEqual(testCase, obj1.x, xExp(:))
            verifyEqual(testCase, obj1.y, yExp(:))
            verifyEqual(testCase, obj2.x, xExp(:))
            verifyEqual(testCase, obj2.y, yExp(:))
		end%fcn
        
        function testStraightNoisyPath(testCase, epsilon)
            
            % Create a straight path with added noise
            sigma = 0.5;
            x = (0:100) + randn(1,101)*sigma;
            y = (0:100) + randn(1,101)*sigma;
            obj0 = PolygonPath.xy2Path(x,y);
            
            obj1 = obj0.rdp(epsilon);
            obj2 = obj0.rdpIter(epsilon);
            
%             obj0.plot('.-', 'MarkerSize',10)
%             hold on
%             obj1.plot('o-')
%             obj2.plot('x-')
%             hold off
            
            verifyTrue(testCase, isequaln(obj1, obj2))
            
        end%fcn
    end
    
end%class
