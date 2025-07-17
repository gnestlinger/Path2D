classdef LengthTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        PathEmpty = struct(...
            'PolygonPath', PolygonPath(), ...
            'SplinePath', SplinePath() ...
            )
        
        Tau = {{}, {0}, {1}, {0 1}}
        
        PathNonEmpty = {% Path object, expected length
            {PolygonPath.xy2Path(0:10, zeros(1,11)), 10};
            {SplinePath.pp2Path(...
            mkpp([-1 0 1 2], [0 1 -1; 1 -2 1; 0 1 0; 1 0 0; 0 1 1; 0 0 1], 2)), ...
            0.5*(2*sqrt(5) + asinh(2)) + 1}; % via Wolfram alpha ...
            }
    end
    
    
    
    methods (Test)
        function testPathEmpty(testCase, PathEmpty, Tau)
        % Test length of empty path with zero/one/two input arguments.
        
            % An empty path always has zero length
            verifyEqual(testCase, PathEmpty.length(Tau{:}), 0);
        end%fcn
        
        function testPathNonEmpty(testCase, PathNonEmpty)
            obj = PathNonEmpty{1};
            lExp = PathNonEmpty{2};
            verifyEqual(testCase, obj.length(), lExp, 'RelTol',0.001);
        end%fcn
        
        function testPathNonEmptySingleArg(testCase, PathNonEmpty)
        % Test length of empty path with one input arguments.
            
            obj = PathNonEmpty{1};
            lExp = PathNonEmpty{2}; % Use as an upper bound
            
            [~,tau] = obj.domain();
            verifyEqual(testCase, obj.length(tau), lExp, 'RelTol',5e-7);
        end%fcn
        
        function testPathNonEmptyTwoArgs(testCase, PathNonEmpty)
        % Test length of non-empty path with two input arguments.
        
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
    end
    
end%class
