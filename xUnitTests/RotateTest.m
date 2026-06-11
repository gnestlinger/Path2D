classdef RotateTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'PolygonPathEmpty', PolygonPath(), ...
            'PolygonPathNonEmpty', PolygonPath.xy2Path(1:10, repmat(2, [10,1])), ...
            'SplinePathEmpty', SplinePath(), ...
            'SplinePathNonEmpty', SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0], 2,2,2)), ...
            'DubinsPathEmpty', DubinsPath(), ...)
            'DubinsPathNonEmpty', DubinsPath([-1 1 pi/2], [1 -1 1], [1.4455 9.1741 1.4455], 2));
    end
    

    methods (Test)
        function testRotate(testCase, obj)
            
            % Rotate
            dphi = pi/2;
            objr = rotate(obj, dphi);
            
            [x,y,tau,h,c] = obj.eval();
            [xr,yr,taur,hr,cr] = objr.eval();
            
            % A point (x,y) rotated by pi/2 moves to (-y,x)
            verifyEqual(testCase, xr, -y, 'AbsTol',1e-12);
            verifyEqual(testCase, yr,  x, 'AbsTol',1e-12);
            verifyEqual(testCase, taur, tau, 'AbsTol',1e-15);
            verifyEqual(testCase, hr, h + dphi, 'AbsTol',1e-15);
            verifyEqual(testCase, cr, c);
        end%fcn
    end
    
end%class
