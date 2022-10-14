classdef RestrictTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        tau0 = {[], 1, 3.5, 7};
        tau1 = {[], 2, 4.2, 6};
    end
    
    methods (Test)
        function testEmptyPath(testCase, tau0, tau1)
            obj = PolygonPath();
            [tauL,tauU] = obj.domain();
            
            objr = obj.restrict(tau0, tau1);
            [tauLr,tauUr] = objr.domain();
            
            % The domain of empty paths is not changed
            testCase.verifyEqual(tauL, tauLr)
            testCase.verifyEqual(tauU, tauUr)
        end%fcn
        
        function testNonEmptyPath(testCase)
            obj = PolygonPath.xy2Path(-10:1:10, zeros(21,1));
            [tauL,tauU] = obj.domain();
            
            [objr,tauA,tauB] = obj.restrict(1, 19);
            testCase.verifyEqual(objr.x, (-9:9)');
            testCase.verifyEqual([tauA,tauB], [1,19]);
        end%fcn
        
        function testLowerDomainEmpty(testCase)
            obj = PolygonPath.xy2Path(-10:1:10, zeros(21,1));
            
            [objr,tauA,tauB] = obj.restrict([], 19);
            testCase.verifyEqual(objr.x, (-10:9)');
            testCase.verifyEqual([tauA,tauB], [0,19]);
        end%fcn
        
        function testUpperDomainEmpty(testCase)
            obj = PolygonPath.xy2Path(-10:1:10, zeros(21,1));
            
            [objr,tauA,tauB] = obj.restrict(1, []);
            testCase.verifyEqual(objr.x, (-9:10)');
            testCase.verifyEqual([tauA,tauB], [1,20]);
        end%fcn
        
        function testLowerDomainExceeding(testCase)
            obj = PolygonPath.xy2Path(-10:1:10, zeros(21,1));
            [tauL,tauU] = obj.domain();
            
            [objr,tauA,tauB] = obj.restrict(-1, 19);
            testCase.verifyEqual(objr.x, obj.x(1:end-1));
            testCase.verifyEqual([tauA,tauB], [tauL,tauB]);
        end%fcn
        
        function testUpperDomainExceeding(testCase)
            obj = PolygonPath.xy2Path(-10:1:10, zeros(21,1));
            [tauL,tauU] = obj.domain();
            
            [objr,tauA,tauB] = obj.restrict(1, 21);
            testCase.verifyEqual(objr.x, obj.x(2:end));
            testCase.verifyEqual([tauA,tauB], [tauA,tauU]);
        end%fcn
        
        function testDomainError(testCase)
            obj = PolygonPath.xy2Path(-10:1:10, zeros(21,1));
            testCase.verifyError(@() obj.restrict(2, 1), ...
                    'PolygonPath:restrict:domain');
        end%fcn
    end
    
end%class

