classdef ShiftTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'PolygonPathEmpty', PolygonPath([], [], [], []), ...
            'PolygonPathOneElm', PolygonPath(1, 2, pi/4, 0), ...
            'PolygonPathNonEmpty', PolygonPath.xy2Path(10:-1:0, zeros(1,11)), ...
            'SplinePathEmpty', SplinePath(), ...
            'SplinePathOneElm', SplinePath.pp2Path(mkpp([-1 2], [0 1 -1; 1 0 2], 2)));
    end
    
    
    methods (Test)
        
        function testShift(testCase, obj)
            
            [P0,P1] = obj.termPoints();
            
            P = [1; 1];
            objs = obj.shift(P);
            [Ps0,Ps1] = objs.termPoints();
            if isempty(obj)
                testCase.verifyTrue(all(isnan([P0; P1; Ps0; Ps1])))
            else
                testCase.verifyEqual(Ps0, P0+P);
                testCase.verifyEqual(Ps1, P1+P);
            end
            
        end%fcn
        
        function testDefaultArg(testCase, obj)
            
            [P0,P1] = obj.termPoints();
            
            objs = obj.shift();
            [Ps0,Ps1] = objs.termPoints();
            if ~isempty(obj)
                testCase.verifyEqual(Ps0, [0;0]);
                testCase.verifyEqual(Ps1, P1-P0);
            end
            
        end%fcn
        
    end
    
end%class
