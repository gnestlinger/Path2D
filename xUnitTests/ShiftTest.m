classdef ShiftTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'PolygonPathEmpty', PolygonPath([], [], [], []), ...
            'PolygonPathOneElm', PolygonPath(1, 2, pi/4, 0), ...
            'PolygonPathNonEmpty', PolygonPath.xy2Path(10:-1:0, zeros(1,11)), ...
            'SplinePathEmpty', SplinePath(), ...
            'SplinePathOneElm', SplinePath.pp2Path(mkpp([-1 2], [0 1 -1; 1 0 2], 2)));
        
        dP = {[1;1], [-1;-1], [10;-20]}
    end
    
    
    methods (Test)
        
        function testShift(testCase, obj, dP)
            
            [P0,P1] = obj.termPoints();
            
            objs = obj.shift(dP);
            [Q0,Q1] = objs.termPoints();
            if obj.isempty()
                verifyEqual(testCase, [P0 P1 Q0 Q1], NaN(2,4))
            else
                verifyEqual(testCase, Q0, P0+dP);
                verifyEqual(testCase, Q1, P1+dP);
            end
            
        end%fcn
        
        function testDefaultArg(testCase, obj)
        % No argument -> shift path so that initial point is at [0;0]
            
            [P0,P1] = obj.termPoints();
            
            objs = obj.shift();
            [Q0,Q1] = objs.termPoints();
            if obj.isempty()
                verifyEqual(testCase, Q0, [NaN; NaN]);
                verifyEqual(testCase, Q1, [NaN; NaN]);
            else
                verifyEqual(testCase, Q0, [0;0]);
                verifyEqual(testCase, Q1, P1-P0);
            end
            
        end%fcn
        
    end
    
end%class
