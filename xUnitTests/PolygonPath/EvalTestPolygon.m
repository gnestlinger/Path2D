classdef EvalTestPolygon < matlab.unittest.TestCase
    
    methods (Test)
        
        function testEvalSimple(testCase)
            obj = PolygonPath(...
                [0 1 1 0 0], ...
                [0 0 1 1 0], ...
                [0 pi/2 pi 1.5*pi 1.5*pi], ...
                zeros(1,5));
            
            tau = [-1 0 0.5 1 1.5 2 2.5 3 3.5 4 4.5];
            [x,y,~,h,c] = obj.eval(tau);
            [xSet,ySet,hSet,cSet] = getEvalSet(obj, tau);
            
            testCase.verifyEqual(x, xSet);
            testCase.verifyEqual(y, ySet);
            testCase.verifyEqual(h, hSet);
            testCase.verifyEqual(c, cSet);
        end%fcn
        
        function testEvalCircleNonMonotonic(testCase)
            obj = PolygonPath.clothoid(200, [0 1/100], 10, 'heald'); 
            
            [tau0,tau1] = obj.domain();
            tau = tau0:1:tau1;
            tau = tau([4 2 9 1 8 7 6 10 3 5]); % Non-monotonic path parameter
            
            [x,y,~,h,c] = obj.eval(tau);
            [xSet,ySet,hSet,cSet] = getEvalSet(obj, tau);
            
            testCase.verifyEqual(x, xSet);
            testCase.verifyEqual(y, ySet);
            testCase.verifyEqual(h, hSet);
            testCase.verifyEqual(c, cSet);
        end%fcn
        
    end
    
end%class


function [x,y,h,c] = getEvalSet(obj, tau)

[tau0,tau1] = obj.domain();
A = interp1(tau0:1:tau1, [obj.x obj.y obj.head obj.curv], tau);

x = A(:,1);
y = A(:,2);
h = A(:,3);
c = A(:,4);

end%fcn
