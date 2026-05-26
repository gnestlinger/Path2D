classdef IntersectLineTestDubins < matlab.unittest.TestCase
    
    properties (TestParameter)        
        PathObj = {...
            DubinsPath([0 0 0], [1 0 -1], [pi 1 pi], 1);
            DubinsPath([0 0 pi], [-1 0 1], [pi 1 pi], 1)}
    end
    
    
    
    methods (Test)
        function testNoIntersection(testCase, PathObj)
        % Test for no intersections.
            
            [act,tau] = PathObj.intersectLine([5 0], pi/2, false);
            exp = zeros(0,2);
            verifyEqual(testCase, act, exp);
            verifyEqual(testCase, tau, zeros(0,1));
        end%fcn
        
        function testSingleIntersectionLine(testCase, PathObj)
        % Test for a single intersection with a single line segment.
            
            x0 = -0.5*sign(cos(PathObj.InitialAng));
            [xyAct,tauAct] = PathObj.intersectLine([x0 0], pi/2, false);
            xyExp = [x0 2];
            verifyEqual(testCase, xyAct, xyExp, 'AbsTol',3e-16);
            verifyEqual(testCase, tauAct, 1.5);
        end%fcn
        
        function testMultipleIntersectionsCircle(testCase, PathObj)
        % Test for a two intersections with a arc segment.

            r = PathObj.TurningRadius;
            x0 = 0.5*r*sign(cos(PathObj.InitialAng));
            [xyAct,tauAct] = PathObj.intersectLine([x0 0], pi/2, false);
            xyExp = [x0 x0; [-r r].*sin(pi/3) + r]';
            verifyEqual(testCase, xyAct, xyExp, 'AbsTol',1e-15);
            verifyEqual(testCase, tauAct, [1/6; 5/6], 'AbsTol',2e-16);
            
        end%fcn
        
        function testIntersectionAllSegments(testCase, PathObj)
        % Test intersections with all three segments (L-S-L / R-S-R style)
        % Expect that enabling circle intersections returns three points and that
        % the single-line intersection (checked in testSingleIntersectionLine) is
        % included among them.

            sig = sign(cos(PathObj.InitialAng));
            x0 = 1*sig;

            [xyAct,tauAct] = PathObj.intersectLine([x0 0], pi/2 + sig*atan(3/4), false);

            xyExp = [sig*[0.74787 -0.5 -1.74787]; 0.33616 2 3.66383]';
            verifyEqual(testCase, xyAct, xyExp, 'AbsTol',1e-5);
            verifyEqual(testCase, tauAct, [0.26893; 1.5; 2.73107], 'AbsTol',1e-5);
        end%fcn

        
%         function testIntersectionEndPoint(testCase, Offset, DPhi)
%             
%             obj0 = PolygonPath.xy2Path([-10 0 2 10] + Offset, [1 0 0 1] + Offset);
%             
%             % Intersection with end point
%             [act,tau] = intersectLine(obj0, [10 0] + Offset, pi/2 + DPhi, false);
%             verifyEqual(testCase, act, [10 1] + Offset, 'AbsTol',1e-12);
%             verifyEqual(testCase, tau, 3);
%         end%fcn
        
%         function testIntersectionWithWaypoint(testCase, Offset)
%         % Test the intersection of a line with a non-terminal waypoint of
%         % the path. This situation can cause redundant solutions, i.e. end
%         % of segment k and start of segmen i+1.
%         
%             obj0 = PolygonPath.xy2Path([-1 0 1 2] + Offset, [0 0 1 2] + Offset);
%             
%             [act,tau] = intersectLine(obj0, [2 0] + Offset, 3*pi/4, false);
%             
%             verifyEqual(testCase, act, [1 1] + Offset, 'AbsTol',1e-12);
%             verifyEqual(testCase, tau, 2);
%         end%fcn
        
%         function testSignReturnsZero(testCase)
%         % Test where sign() returns zero.
%         
%             obj0 = PolygonPath.xy2Path(0:4, 0:4);
%             
%             [act,tau] = intersectLine(obj0, [0 2], 0, false);
%             
%             verifyEqual(testCase, act, [2 2]);
%             verifyEqual(testCase, tau, 2);
%         end%fcn
%         
%         function testTouchingIntersection(testCase)
%             
%             obj0 = PolygonPath.xy2Path([-1 1 2], [0.1 pi -exp(1)]);
%             
%             [act,tau] = intersectLine(obj0, [0 pi], 0, false);
%             
%             verifyEqual(testCase, act, [1 pi]);
%             verifyEqual(testCase, tau, 1);
%         end%fcn
    end
    
end%class
