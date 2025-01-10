classdef CircleTestSpline < matlab.unittest.TestCase
    
    properties (TestParameter)
        Params = {...
            {[-pi/2 0], 3};
            {[-pi/2 0] + 2*pi, 5};
            {[pi 2*pi], 13}}
    end
    
    
    methods (Test)
        function testCreateCircle(testCase, Params)
            
            R = 10;
            phi01 = Params{1};
            nbrSegments = Params{2};
            obj = SplinePath.circle(R, phi01, nbrSegments);
            
            % At the break points, the spline-circle should match the true
            % circle
            breaks = obj.Breaks';
            [x,y] = obj.eval(breaks);
            verifyEqual(testCase, x, R*cos(breaks));
            verifyEqual(testCase, y, R*sin(breaks), 'AbsTol',5e-16);
            
            verifyEqual(testCase, obj.numel(), nbrSegments);
        end%fcn
    end
    
end%class
