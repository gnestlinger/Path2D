classdef Frenet2CartTest < matlab.unittest.TestCase
    
    methods (Test)
        
        function testFrenet2Cart(testCase)
            obj = PolygonPath.xy2Path(0:3, [0 0 0 0]);
            [xy,Q,idx,tau] = obj.frenet2cart([0 1; 1 -1; 2.25 2; 3 -2], false);
            testCase.verifyEqual(xy, [0 1; 1 -1; 2.25 2; 3 -2]);
            testCase.verifyEqual(Q, [0 0; 1 0; 2.25 0; 3 0]);
            testCase.verifyEqual(idx, uint32([1 2 3 3])');
            testCase.verifyEqual(tau, [0 1 2.25 3]');
        end%fcn
        
        function testOutOfBound(testCase)
            obj = PolygonPath.xy2Path([0 1 3 4], [0 1 -1 0]);
            
            % Pre-path solution
            [xy,Q,idx,tau] = obj.frenet2cart([-1 1], false);
            testCase.verifyEqual(xy, [-sqrt(2) 0], 'AbsTol',1e-15);
            testCase.verifyEqual(Q, [-1/sqrt(2) -1/sqrt(2)]);
            testCase.verifyEqual(idx, uint32(1));
            testCase.verifyEqual(tau, -1/sqrt(2));
            
            % Post-path solution
            [xy,Q,idx,tau] = obj.frenet2cart([5*sqrt(2) -sqrt(2)], false);
            testCase.verifyEqual(xy, [6 0], 'AbsTol',1e-15);
            testCase.verifyEqual(Q, [5 1], 'AbsTol',1e-15);
            testCase.verifyEqual(idx, uint32(3));
            testCase.verifyEqual(tau, 4);
        end%fcn
        
        function testCircuit(testCase)
            obj = PolygonPath.circle(1, [0 2*pi], 23);
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
