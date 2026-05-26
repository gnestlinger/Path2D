classdef FrenetOffsetTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        PathEmpty = struct(...
            'PolygonPathEmpty',PolygonPath(), ...
            'PathZeroLength',PolygonPath(1, 1, 0, 0), ...
            'SplinePathEmpty',SplinePath(), ...
            'SplinePathZeroLength',SplinePath([0 0], reshape([1 0; 0 0], [2 1 2])))
        
        PathNonEmpty = struct(...
            'PolygonPath', PolygonPath.xy2Path(0:10, zeros(1,11)), ...
            'SplinePath', SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0], [2 2 2])))
        
        NbrOffsets = {0, 1, 10}
    end
    
    
    methods (Test)
        function testPathEmpty(testCase, PathEmpty, NbrOffsets)
            [a,b] = PathEmpty.domain();
            d = linspace(a, b, NbrOffsets);
            
            verifyError(testCase, @() PathEmpty.frenetOffset(d(:)), ...
                'Path2D:SingularDomain')
        end%fcn
        
        function testPathNonEmpty(testCase, PathNonEmpty, NbrOffsets)
            [a,b] = PathNonEmpty.domain();
            d = linspace(a, b, NbrOffsets);
            PathNew = PathNonEmpty.frenetOffset(d(:));
            
            verifyEqual(testCase, numel(PathNew.x), NbrOffsets)
        end%fcn
        
        function testPathNonEmptySD(testCase, PathNonEmpty, NbrOffsets)
        % Test with matrix-valued input argument.
        
            [a,b] = PathNonEmpty.domain();
            S = PathNonEmpty.length();
            sdForward = [...
                linspace(a, b, NbrOffsets); 
                linspace(0, S, NbrOffsets)]';
            PathNew = PathNonEmpty.frenetOffset(sdForward);
            
            verifyEqual(testCase, numel(PathNew.x), NbrOffsets)
            
            % Reverse "engineer" frenet-coordinates
            sdReverse = zeros(NbrOffsets, 2);
            for i = 1:NbrOffsets
                Pi = [PathNew.x(i) PathNew.y(i)];
                sd = PathNonEmpty.cart2frenet(Pi);
                if size(sd,1) > 1
                    % If there are multiple solutions, select the one which
                    % is closer to the expected value.
                    sSet = sdForward(i,1);
                    [~,theIdx] = min(abs(sSet - sd(:,1)));
                    sdReverse(i,:) = sd(theIdx,:);
                else
                    sdReverse(i,:) = sd;
                end
            end
            verifyEqual(testCase, sdForward(:,1), sdReverse(:,1), 'AbsTol',1e-15)
        end%fcn
    end
    
end%class
