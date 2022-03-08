classdef RotateTest < matlab.unittest.TestCase
    
    methods (Test)
        function testRotate(testCase)
            
            % Straight path
            obj0 = PolygonPath.xy2Path(1:10, repmat(2, [10,1]));
			
            % Rotate
			dphi = pi/2;
			obj1 = rotate(obj0, dphi);
			
            % A point (x,y) rotate by pi/2 moves to (-y,x)
			verifyEqual(testCase, obj1.x, -obj0.y, 'AbsTol',1e-12);
			verifyEqual(testCase, obj1.y, +obj0.x, 'AbsTol',1e-12);
			verifyEqual(testCase, obj1.head, obj0.head + dphi);
			verifyEqual(testCase, obj1.curv, obj0.curv);
		end%fcn
    end
    
end%class
