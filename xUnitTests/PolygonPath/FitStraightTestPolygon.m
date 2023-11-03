classdef FitStraightTestPolygon < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'StraightSlope0',PolygonPath.xy2Path(0:100, ones(101,1)), ...
            'StraightSlopePos',PolygonPath.xy2Path(0:100, (0:100)/2), ...
            'StraightSlopeNeg',PolygonPath.xy2Path(0:100, -(0:100)/4));    
    end
    
    
    methods (TestMethodSetup)
        function setRandNumber(~)
            rng(6);
        end%fcn
    end
    
    methods (TestMethodTeardown)
        function resetRandNbr(~)
            rng('default');
        end%fcn
    end
    
    
    methods (Test)
        
        function testFitValues(testCase, obj)
            [objStraight,e] = obj.fitStraight();
            
            testCase.verifyClass(objStraight, 'PolygonPath')
            
            % Error must be non-negative and "small"
            testCase.verifyGreaterThanOrEqual(e, 0);
            testCase.verifyLessThanOrEqual(e, 1e-20);
        end%fcn
        
        function testNoisyPath(testCase, obj)
            N = obj.numel();
            dx = randn(N, 1);
            dy = randn(N, 1);
            objNoisy = PolygonPath(obj.x + dx, obj.y + dy, obj.head, obj.curv);
            
            [objStraight,e] = objNoisy.fitStraight();
            
            testCase.verifyClass(objStraight, 'PolygonPath')
            
            % Error must be non-negative and "small"
            testCase.verifyGreaterThanOrEqual(e, 0);
            testCase.verifyLessThanOrEqual(e, 1.2);
%             plot([obj objNoisy objStraight])
        end%fcn
        
        function testInfSlope(testCase)
            obj0 = PolygonPath.xy2Path(repmat(10, [11 1]), [0:9 11]);
            [P0set,P1set] = obj0.termPoints();
            
            [objStraight,e] = obj0.fitStraight();
            [P0act,P1act] = objStraight.termPoints();
            
            testCase.verifyEqual(P0set, P0act, 'AbsTol',1e-14);
            testCase.verifyEqual(P1set, P1act, 'AbsTol',1e-14);
            
            % Error must be non-negative and "small"
            testCase.verifyGreaterThanOrEqual(e, 0);
            testCase.verifyLessThanOrEqual(e, 1e-20);
        end%fcn
        
    end
    
end%class
