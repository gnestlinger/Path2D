classdef XY2PathTest < matlab.unittest.TestCase
    
    properties(TestParameter)
        XY = {...
            {1:1:10, 2:2:20}; 
            {linspace(-2,2,100), linspace(-2,2,100).^2}}
    end

    
    
    methods(Test, ParameterCombination='exhaustive')
        function testXY2Path(testCase, XY)
            x = XY{1};
            y = XY{2};
            [h,c] = originalAlgorithm(x, y);
            obj = PolygonPath.xy2Path(x, y);
            
            verifyEqual(testCase, obj.x, x(:));
            verifyEqual(testCase, obj.y, y(:));
            verifyEqual(testCase, obj.head, h(:));
            verifyEqual(testCase, obj.curv, c(:), 'AbsTol',1e-15);
        end%fcn
        
        function testArrayDims(testCase)
            exc = 'MATLAB:minrhs';
            a = 1:8;
            b = 1:9;
            assertError(testCase, @() PolygonPath(a, b), exc);
            assertError(testCase, @() PolygonPath(b, a), exc);
            assertError(testCase, @() PolygonPath(a', b'), exc);
            assertError(testCase, @() PolygonPath(b', a'), exc);
        end%fcn
    end
    
end%class


function [h,c] = originalAlgorithm(x, y)
gx = gradient(x(:));
gy = gradient(y(:));
h = unwrap(atan2(gy, gx));
c = (gx.*gradient(gy) - gradient(gx).*gy) ./ (gx.^2 + gy.^2).^1.5;
end%fcn
