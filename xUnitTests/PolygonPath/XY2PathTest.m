classdef XY2PathTest < matlab.unittest.TestCase
    
    properties(TestParameter)
        x = {1:1:10, rand(10,1)}
        y = {2:2:20, rand(10,1)}
    end
    
    
    methods (TestMethodSetup)
        function setRandNumber(~)
            rng(1);
        end%fcn
    end
    
    methods (TestMethodTeardown)
        function resetRandNbr(~)
            rng('default');
        end%fcn
    end
    
    
    methods(Test, ParameterCombination='exhaustive')
        
        function testXY2Path(testCase, x, y)
            [h,c] = originalAlgorithm(x, y);
            obj = PolygonPath.xy2Path(x, y);
            
            testCase.verifyEqual(obj.x, x(:));
            testCase.verifyEqual(obj.y, y(:));
            testCase.verifyEqual(obj.head, h(:));
            testCase.verifyEqual(obj.curv, c(:), 'AbsTol',1e-14);
        end%fcn
        
        function testArrayDims(testCase)
            exc = 'MATLAB:minrhs';
            a = 1:8;
            b = 1:9;
            testCase.assertError(@() PolygonPath(a, b), exc);
            testCase.assertError(@() PolygonPath(b, a), exc);
            testCase.assertError(@() PolygonPath(a', b'), exc);
            testCase.assertError(@() PolygonPath(b', a'), exc);
        end%fcn
        
    end
    
end%class


function [h,c] = originalAlgorithm(x, y)
gx = gradient(x(:));
gy = gradient(y(:));
h = unwrap(atan2(gy, gx));
c = (gx.*gradient(gy) - gradient(gx).*gy) ./ (gx.^2 + gy.^2).^1.5;
end%fcn
