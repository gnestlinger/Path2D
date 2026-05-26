classdef ConnectTestSpline < matlab.unittest.TestCase
    
    properties (TestParameter)
        P0Straight = {[0 0], [-10 -10], [10 10]}
        P1Straight = {[3 3], [-3 -3]}
        NbrDerivatives = {1, 2, 3, 4}
    end
    
    
    
    methods (TestClassSetup)
        function ClassSetup(testCase)
            orig = rng;
            testCase.addTeardown(@rng, orig)
            rng(123)
        end
    end
        
    methods (Test)
        function testConnectStraight(testCase, P0Straight, P1Straight)
        % When no differential boundary conditions are requested, the
        % solution is a straight line.
        
            obj = SplinePath.connect(P0Straight, P1Straight);
            coefsExp = [P1Straight' - P0Straight', P0Straight'];
            verifyEqual(testCase, obj.mkpp().coefs, coefsExp);    
        end%fcn
        
        function testConnectND(testCase, NbrDerivatives)
        % Test with N differential boundary conditions for P0/P1.
        
            P0 = rand(1 + NbrDerivatives, 2)*10 - 5;
            P1 = rand(1 + NbrDerivatives, 2)*10 - 5;
            obj = SplinePath.connect(P0, P1);
            
            % Check that actual derivatives match expected
            ppd = obj.mkpp();
            d0 = ppval(ppd, [0 1]);
            assertEqual(testCase, d0(:,1), P0(1,:)', 'AbsTol',0);
            assertEqual(testCase, d0(:,2), P1(1,:)', 'AbsTol',1e-11);
            for i = 1:NbrDerivatives
                ppd = ppder(ppd);
                assertEqual(testCase, ppval(ppd, 0), P0(i+1,:)', 'AbsTol',1e-15);
                assertEqual(testCase, ppval(ppd, 1), P1(i+1,:)', 'AbsTol',1e-9);
            end%for
        end%fcn
    end
    
end%class


function ppd = ppder(pp)

[~,coefs,L,order] = unmkpp(pp);
coefsd = zeros(L*2, order-1);
for i = 1:size(coefs, 1)
    coefsd(i,:) = polyder(coefs(i, :));
end%for

ppd = mkpp(pp.breaks, coefsd, 2);

end%fcn
