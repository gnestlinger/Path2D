classdef LengthTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        % Path instances with a length of either NaN, 0 or 1
        PathObj = struct(...
            'PolygonPathEmpty', PolygonPath(), ...
            'PolygonPathZeroLength', PolygonPath(1, 2, pi/4, 0), ...
            'PolygonPathUnitLength', PolygonPath.xy2Path(0:1, zeros(1,2)), ...
            'SplinePathEmpty', SplinePath(), ...
            'SplinePathZeroLength', SplinePath([0 0], reshape([1 1; 1 2], [2 1 2])), ...
            'SplinePathUnitLength', SplinePath([0 1], reshape([1 0; 0 0],  [2 1 2])), ...
            'DubinsPathEmpty', DubinsPath());
        
        % Define sizes for path parameter argument(s) to test
        TauSz = struct(...% 'Empty',{{}}, ...
            'Scalar',[1 1], ...
            'Row',[1 3], ...
            'Column',[3 1], ...
            'Matrix',[2 3], ...
            'Nd', [4 2 7])
        
        PathNonEmpty = {% Path object, expected length
            {PolygonPath.xy2Path(0:10, [1 -1 1 -1 1 -1 1 -1 1 -1 1]/2), ...
            sqrt(2)*10};
            {SplinePath(-2:1, ...
            cat(3, [0 0 0; 1 1 0], [1 1 1; -2 0 0], [-1 0 1; 1 0 1])), ...
            (sqrt(5) + asinh(2)/2) + 1}; % via Wolfram alpha ...
            {DubinsPath([1 2 pi], [1 -1 0], [1 2 3], 2), 6};
            }
    end
    
    
    
    methods (Test)
        function testReturnSize0Arg(testCase, PathObj)
        % Test return size of length().
            
            verifySize(testCase, PathObj.length(), [1 1]);
        end%fcn
        
        function testReturnSize1Arg(testCase, PathObj, TauSz)
        % Test return size of length(tau).
            
            tau = zeros(TauSz);
            verifySize(testCase, PathObj.length(tau), TauSz);
        end%fcn
        
        function testReturnSize2Arg(testCase, PathObj, TauSz)
        % Test return size of length(tau0,tau1).
            
            tau0 = zeros(TauSz);
            tau1 = zeros(TauSz);
            verifySize(testCase, PathObj.length(tau0, tau1), TauSz);
        end%fcn
        
        function testReturnValue0Arg(testCase, PathObj)
        % Test return value of length().
            
            [a,b] = PathObj.domain();
            if isnan(a)
                lExp = NaN;
            elseif b > a
                lExp = 1;
            else
                lExp = 0;
            end
            
            verifyEqual(testCase, PathObj.length(), lExp);
        end%fcn
        
        function testReturnValue1Arg(testCase, PathObj, TauSz)
        % Test return value of length(tau).
            
            [a,b] = PathObj.domain();
            if isnan(a)
                lExp = NaN;
            elseif b > a
                lExp = 1;
            else
                lExp = 0;
            end
            tau = repmat(b, TauSz);
            
            verifyEqual(testCase, PathObj.length(tau), repmat(lExp, TauSz));
        end%fcn
        
        function testReturnValue2Arg(testCase, PathObj, TauSz)
        % Test return value of length(tau0,tau1).
            
            [a,b] = PathObj.domain();
            if isnan(a)
                lExp = NaN;
            elseif b > a
                lExp = 1;
            else
                lExp = 0;
            end
            tau0 = repmat(a, TauSz);
            tau1 = repmat(b, TauSz);
            
            verifyEqual(testCase, PathObj.length(tau0, tau1), repmat(lExp, TauSz));
        end%fcn
        
        function testAccuracy0Arg(testCase, PathNonEmpty)
        % Test accuracy of length().
        
            obj = PathNonEmpty{1};
            lExp = PathNonEmpty{2};
            verifyEqual(testCase, obj.length(), lExp, 'RelTol',1e-7);
        end%fcn
        
        function testAccuracy1Arg(testCase, PathNonEmpty)
        % Test accuracy of length(tau).
            
            obj = PathNonEmpty{1};
            lExp = PathNonEmpty{2};
            
            [~,tau] = obj.domain();
            verifyEqual(testCase, obj.length(tau), lExp, 'RelTol',5e-7);
        end%fcn
        
        function testAccuracy2Args(testCase, PathNonEmpty)
        % Test accuracy of length(tau0, tau1).
        
            obj = PathNonEmpty{1};
            lExp = PathNonEmpty{2};
            
            [tau0,tau1] = obj.domain();
            len01 = obj.length(tau0, tau1);
            len10 = obj.length(tau1, tau0);
            
            verifyEqual(testCase, len01, lExp, 'RelTol',5e-7);
            verifyEqual(testCase, len10, lExp, 'RelTol',5e-7);
        end%fcn
        
        function testExtrap(testCase, PathNonEmpty)
        % Test length of non-empty path with two input arguments outside
        % domain.
        
            obj = PathNonEmpty{1};
%             lExp = PathNonEmpty{2};
            
            [tau0,tau1] = obj.domain();
            
            % Make sure path parameter exceeds domain
            tau0 = tau0 - 1;
            tau1 = tau1 + 1;
            
            verifyEqual(testCase, obj.length(tau0, tau1), nan);
            verifyEqual(testCase, obj.length(tau1, tau0), nan);
        end%fcn
        
        
        function errorsTauSizeMismatch(testCase, PathNonEmpty)
        % Test for error message if sizes of TAU0/TAU1 do not match.
        
            verifyError(testCase, @() PathNonEmpty{1}.length([0 1], [0;1]), ...
                'Path2D:SizeMismatch')
        end%fcn
    end
    
end%class
