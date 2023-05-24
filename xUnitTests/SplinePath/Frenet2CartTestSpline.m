classdef Frenet2CartTestSpline < matlab.unittest.TestCase
    
    methods (Test)
        
        function testFrenet2Cart(testCase)
            obj = SplinePath.pp2Path(mkpp([0 1 2 3], [1 0; 0 0; 1 1; 0 0; 1 2; 0 0], 2));
            [xy,Q,idx,tau] = obj.frenet2cart([0 1; 1 -1; 2.25 2; 3 -2], false);
            testCase.verifyEqual(xy, [0 1; 1 -1; 2.25 2; 3 -2]);
            testCase.verifyEqual(Q, [0 0; 1 0; 2.25 0; 3 0]);
            testCase.verifyEqual(idx, uint32([1 2 3 3])');
            testCase.verifyEqual(tau, [0 1 2.25 3]');
        end%fcn
        
        function testOutOfBound(testCase)
            obj = SplinePath(0:2, cat(3, [1 1;0 0], [0 1;1 1]));
            
            % Pre-path/post-path  solution
            [xy,Q,idx,tau] = obj.frenet2cart([-1 1; 3 -1], false);
            testCase.verifyEqual(xy, [-1 2; 3 0]);
            testCase.verifyEqual(Q, [-1 1; 3 1]);
            testCase.verifyEqual(idx, uint32([1; 2]));
            testCase.verifyEqual(tau, [-1; 3]);
        end%fcn
        
        function testCircuit(testCase)
            pp = load('circle.mat');
            obj = SplinePath.pp2Path(pp);
            assert(obj.IsCircuit)
            s = obj.length();
            [xy1,Q1,idx1,tau1] = obj.frenet2cart([0 1; 0 -1], false);
            [xy2,Q2,idx2,tau2] = obj.frenet2cart([s 1; s -1], false);
            
            % Test for periodic solutions due to closed path
            testCase.verifyTrue(isequal(size(xy1,1), size(Q1,1), size(idx1,1), size(tau1,1)));
            testCase.verifyEqual(xy1, xy2);
            testCase.verifyEqual(Q1, Q2);
            testCase.verifyEqual(idx1, idx2);
            testCase.verifyEqual(tau1, tau2);
        end%fcn
        
    end
    
end%class
