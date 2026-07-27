classdef SFGVFTest < matlab.unittest.TestCase
% TestSFGVF  Unit tests for Path2D.sfgvf
    
    properties (TestParameter)
        PathObj = struct(...
            'PolygonPathStraight',PolygonPath.straight([0 0], [100 10]), ...
            'SplinePathStraight',SplinePath.straight([0 0], [1 0]));
        
        NTau = struct(...
            'Scalar',uint8(1), ...
            'Vector',uint8(100))
    end
    
    
    methods (Test)
        function testReturnSizes(testCase, PathObj, NTau)
            
            % Query points (column vectors)
            X = [0; 1; 2];
            Y = [0; 0; 0];
            
            % Call sfgvf with default k (should default to [1 1])
            tau = PathObj.sampleDomain(NTau);
            [chi1,chi2,chi3] = PathObj.sfgvf(X, Y, tau);

            % Sizes
            testCase.verifySize(chi1, [size(X) numel(tau)]);
            testCase.verifySize(chi2, [size(X) numel(tau)]);
            testCase.verifySize(chi3, [size(X) numel(tau)]);
        end%fcn
    end

end%class
