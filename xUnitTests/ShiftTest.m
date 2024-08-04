classdef ShiftTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'PolygonPathEmpty', PolygonPath([], [], [], []), ...
            'PolygonPathOneElm', PolygonPath(1, 2, pi/4, 0), ...
            'PolygonPathNonEmpty', PolygonPath.xy2Path(10:-1:0, zeros(1,11)), ...
            'SplinePathEmpty', SplinePath(), ...
            'SplinePathOneElm', SplinePath.pp2Path(mkpp([-1 2], [0 1 -1; 1 0 2], 2)), ...
            'DubinsPathEmpty', DubinsPath(), ...
            'DubinsPathOneElm',DubinsPath([-1 1 pi/2], 1, 2, 2), ...
            'DubinsPathNonEmpty', DubinsPath([-1 1 pi/2], [1 -1 1], [1.4455 9.1741 1.4455], 2));
        
        dP = {[1;1], [-1;-1], [10;-20]}
    end
    
    
    methods (Test)
        
        function testShift(testCase, obj, dP)
            
            [P0,P1] = obj.termPoints();
            
            objs = obj.shift(dP);
            [Q0,Q1] = objs.termPoints();
            if isempty(obj)
                testCase.verifyTrue(all(isnan([P0; P1; Q0; Q1])))
            else
                testCase.verifyEqual(Q0, P0+dP);
                testCase.verifyEqual(Q1, P1+dP, 'AbsTol',3e-16); % Tol added for Dubins path
            end
            
        end%fcn
        
        function testDefaultArg(testCase, obj)
        % No argument -> shift path so that initial point is at [0;0]
            
            [P0,P1] = obj.termPoints();
            
            objs = obj.shift();
            [Q0,Q1] = objs.termPoints();
            if ~isempty(obj)
                testCase.verifyEqual(Q0, [0;0]);
                testCase.verifyEqual(Q1, P1-P0);
            end
            
        end%fcn
        
    end
    
end%class
