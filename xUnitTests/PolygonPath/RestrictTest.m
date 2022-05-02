classdef RestrictTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        tau0 = {[], 1, 7};
        tau1 = {[], 2, 6};
    end
    
    methods (Test)
        function testEmptyPath(testCase, tau0, tau1)
            obj = PolygonPath.empty();
            [tauL,tauU] = obj.domain();
            
            objr = obj.restrict(tau0, tau1);
            [tauLr,tauUr] = objr.domain();
            
            % The domain of empty paths is not changed
            testCase.verifyEqual(tauL, tauLr)
            testCase.verifyEqual(tauU, tauUr)
        end%fcn
        
        function testNonEmptyPath(testCase, tau0, tau1)
            obj = PolygonPath.xy2Path(0:10, zeros(11,1));
            [tauL,tauU] = obj.domain();
            
            if isempty(tau0)
                tau0 = tauL;
            end
            if isempty(tau1)
                tau1 = tauU;
            end
            
            if tau0 < tau1
                [objr,tau0,tau1] = obj.restrict(tau0, tau1);
                [tauLr,tauUr] = objr.domain();
                testCase.verifyEqual(tauUr - tauLr, min(tau1,tauU) - max(tau0,tauL))
            else
                testCase.verifyError(@() obj.restrict(tau0, tau1), ...
                    'PolygonPath:restrict:domain');
            end
        end%fcn
    end
    
end%class
