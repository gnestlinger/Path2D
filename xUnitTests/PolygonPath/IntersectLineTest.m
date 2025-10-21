classdef IntersectLineTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        Offset = {0 1e1 1e2 1e3 1e4 1e5}
        
        DPhi = {0 -2*pi}
    end
    
    
    
    methods (Test)
        function testNoIntersection(testCase)
            
            obj0 = PolygonPath.xy2Path([0 1], [0 0]);
            [act,tau] = intersectLine(obj0, [0 4], 2, false);
            exp = zeros(0,2);
            verifyEqual(testCase, act, exp);
            verifyEqual(testCase, tau, zeros(0,1));
        end%fcn
        
        function testSingleIntersection(testCase)
            
            obj0 = PolygonPath.xy2Path([-10 -2 0 4 10], [0 0 0 3 3]);
            [act,tau] = intersectLine(obj0, [5 0], pi/2, false);
            exp = [5 3];
            verifyEqual(testCase, act, exp);
            verifyEqual(testCase, tau, 3+1/6);
        end%fcn
        
        function testMultipleIntersections(testCase)
            
            obj0 = PolygonPath.xy2Path([0 5 8 8 5 0], [0 0 0 2 2 2]);
            [act,tau] = intersectLine(obj0, [6 0], pi/2, false);
            exp = [6 0; 6 2];
            verifyEqual(testCase, act, exp, 'AbsTol',1e-12);
            verifyEqual(testCase, tau, [1+1/3;3+2/3]);
            
        end%fcn
        
        function testIntersectionInitPoint(testCase, Offset, DPhi)
            
            obj0 = PolygonPath.xy2Path([0 2 10 20] + Offset, [0 0 1 1] + Offset);
            
            % Intersection with initial point
            [act,tau] = intersectLine(obj0, [2 -2] + Offset, 3*pi/4 + DPhi, false);
            verifyEqual(testCase, act, [0 0] + Offset, 'AbsTol',1e-12);
            verifyEqual(testCase, tau, 0, 'AbsTol',1e-15);
        end
        
        function testIntersectionEndPoint(testCase, Offset, DPhi)
            
            obj0 = PolygonPath.xy2Path([-10 0 2 10] + Offset, [1 0 0 1] + Offset);
            
            % Intersection with end point
            [act,tau] = intersectLine(obj0, [10 0] + Offset, pi/2 + DPhi, false);
            verifyEqual(testCase, act, [10 1] + Offset, 'AbsTol',1e-12);
            verifyEqual(testCase, tau, 3);
        end%fcn
        
        function testIntersectionWithWaypoint(testCase, Offset)
        % Test the intersection of a line with a non-terminal waypoint of
        % the path. This situation can cause redundant solutions, i.e. end
        % of segment k and start of segmen i+1.
        
            obj0 = PolygonPath.xy2Path([-1 0 1 2] + Offset, [0 0 1 2] + Offset);
            
            [act,tau] = intersectLine(obj0, [2 0] + Offset, 3*pi/4, false);
            
            verifyEqual(testCase, act, [1 1] + Offset, 'AbsTol',1e-12);
            verifyEqual(testCase, tau, 2);
        end%fcn
        
        function testSignReturnsZero(testCase)
        % Test where sign() returns zero.
        
            obj0 = PolygonPath.xy2Path(0:4, 0:4);
            
            [act,tau] = intersectLine(obj0, [0 2], 0, false);
            
            verifyEqual(testCase, act, [2 2]);
            verifyEqual(testCase, tau, 2);
        end%fcn
        
        function testTouchingIntersection(testCase)
            
            obj0 = PolygonPath.xy2Path([-1 1 2], [0.1 pi -exp(1)]);
            
            [act,tau] = intersectLine(obj0, [0 pi], 0, false);
            
            verifyEqual(testCase, act, [1 pi]);
            verifyEqual(testCase, tau, 1);
        end%fcn
        
        function testForUniqueSolutions(testCase)
            
            d = load('testLineIntersection.mat');
            obj0 = PolygonPath.fromStruct(d.Path);
            x = d.x;
            y = d.y;
            h = d.h;
            
            idx = [];
            for i = 1:numel(x)
                [act,tau] = intersectLine(obj0, [x(i) y(i)], h(i) + pi/2, false);
                
                if size(act,1) > 1 || numel(tau) > 1
                    idx = [idx; i];
                end
            end
            
            verifyEmpty(testCase, idx, 'Redundant solutions!');
        end%fcn
    end
    
end%class
