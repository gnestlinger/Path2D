classdef DerivativeTestSpline < matlab.unittest.TestCase
    
    properties (TestParameter)
        pp = struct(...
            'Default',spline(1:10, [0 3 7 5 2 9 7 3 6 10; 8 5 2 1 9 3 2 7 4 6]));
        
        nder = {0, 1, 2, 3, 4, 5};
    end
    
    
    methods (Test)
        
        function testNthDerivative(testCase, pp, nder)
            
            % Derivative via built-in POLYDER
            [n,m] = size(pp.coefs);
            coefsSet = zeros(n, max(1, m-nder));
            for i = 1:size(coefsSet, 1)
                tmp = pp.coefs(i,:);
                for j = 1:nder % Repeated polynomial derivative
                    tmp = polyder(tmp);
                end
                coefsSet(i,:) = tmp;
            end
            
            % Derivative via class method
            obj = SplinePath.pp2Path(pp);
            objd = obj.derivative(nder);
            coefsAct = objd.mkpp().coefs;
            
            % Checks
            testCase.verifyEqual(coefsAct, coefsSet);
            testCase.verifyEqual(pp.breaks, objd.Breaks);
            testCase.verifySize(obj.cumlengths(), size(obj.cumlengths()));
            
        end%fcn
        
        function testIsCurcuit(testCase)
            
            % 
            x = pi*(0:.5:2); 
            y = [0 1 0 -1  0 1 0; 
                 1 0 1  0 -1 0 1];
            pp = spline(x, y);
            
            % 
            obj = SplinePath.pp2Path(pp);
            objd = obj.derivative(2);
            
            % Checks
            testCase.verifyTrue(obj.IsCircuit);
            testCase.verifyFalse(objd.IsCircuit);
            
        end%fcn
        
    end
    
end%class
