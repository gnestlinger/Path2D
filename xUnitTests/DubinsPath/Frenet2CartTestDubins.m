classdef Frenet2CartTestDubins < matlab.unittest.TestCase
    
    properties (TestParameter)
        PathObj = {DubinsPath([0 0 0], [0 1 -1], [1 pi/2 pi/2], 2)}
        DSet = {-1 +1 3 -4}
    end


    
    methods (Test)
        
        function testFrenet2Cart(testCase, PathObj)

            s = [0 0.5 linspace(1, PathObj.length(), 6)];
            d = zeros(size(s));
            [xy,Q,idx,tau] = PathObj.frenet2cart([s; d]', false);

            % Test the distance from Q to xy
            dAct = hypot1Arg(xy - Q);
            verifyEqual(testCase, dAct, d(:));

            % Hard 
            xySet = [0 0; 0.5 0; 1 0; 1.6180 0.0979; 2.1756 0.382; ...
                2.6529 0.7896; 3.2104 1.0737; 3.8284 1.1716];
            verifyEqual(testCase, xy, xySet, 'AbsTol',5e-5);
            verifyEqual(testCase, Q, xy);
            verifyEqual(testCase, idx, uint32([1 1 2 2 2 3 3 3])');
            verifyEqual(testCase, tau, [0 0.5 1 1.4 1.8 2.2 2.6 3]');
        end%fcn

        function testFrenet2CartVariedD(testCase, PathObj, DSet)

            s = [0 0.5 linspace(1, PathObj.length(), 6)];
            d = DSet*ones(size(s));
            [xy,Q,idx,tau] = PathObj.frenet2cart([s; d]', false);

            % Test the distance from Q to xy
            dAct = hypot1Arg(xy - Q);
            verifyEqual(testCase, dAct, abs(d(:)), 'AbsTol',1e-15);

            % Segment index and tau are independent of DSet
            verifyEqual(testCase, idx, uint32([1 1 2 2 2 3 3 3])');
            verifyEqual(testCase, tau, [0 0.5 1 1.4 1.8 2.2 2.6 3]');
        end%fcn
        
        function testOutOfBound(testCase, PathObj)

            % Pre-path/post-path  solution
            [xy,Q,idx,tau] = PathObj.frenet2cart([-1 1; 5 1], true);
            testCase.verifyEqual(xy, [-1 2; 3 0]);
            testCase.verifyEqual(Q, [-1 1; 3 1]);
            testCase.verifyEqual(idx, uint32([1; 2]));
            testCase.verifyEqual(tau, [-1; 3]);
        end%fcn
        
        function testCircuit(testCase)
            % TODO
        end%fcn
        
    end
    
end%class

function d = hypot1Arg(x)
d = hypot(x(:,1), x(:,2));
end%fcn