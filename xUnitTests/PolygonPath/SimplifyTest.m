classdef SimplifyTest < matlab.unittest.TestCase
    
    methods (Test)
        function testLine(testCase)
        % Test simplification of a straight path.
            
            % Create a line with intermediate points
            obj0 = PolygonPath.xy2Path(0:10, 0:10);
            
            [obj1,keep] = obj0.simplify();
            
            % The terminal points must match
            [exp0,exp1] = obj0.termPoints();
            [act0,act1] = obj1.termPoints();
            verifyEqual(testCase, act0, exp0);
            verifyEqual(testCase, act1, exp1);
            
            % The length must match
            verifyEqual(testCase, obj1.length(), obj0.length())
            
            % Exactly two points, i.e. one segment, must survive
            verifyEqual(testCase, obj1.numel(), 1)
            
            verifySize(testCase, keep, [obj0.numel()+1 1])
        end%fcn
        
        function testKeepAllStairs(testCase)
        % Test stair-like path which cannot be simplified.
        
            % Create a path where each line segment contains only two
            % points -> no simplification possible
            obj0 = PolygonPath(...
                [0 1 1 2 2 3 3 4 4], ...
                [0 0 1 1 2 2 3 3 4], ...
                [0 pi/2 0 pi/2 0 pi/2 0 pi/2 0], ...
                zeros(1,9));
            
            [obj1,keep] = obj0.simplify();
            
            verifyEqual(testCase, obj0, obj1)
            verifySize(testCase, keep, [obj0.numel()+1 1])
        end%fcn
        
        function testKeepAllCircle(testCase)
        % Test circular path which cannot be simplified.
        
            % Create a path where each line segment contains only two
            % points -> no simplification possible
            R = 1;
            N = 100;
            phi = linspace(0, pi/2, N);
            obj0 = PolygonPath(...
                R*cos(phi), ...
                R*sin(phi), ...
                phi + pi/2, ...
                repmat(1/R, [N 1]));
            
            [obj1,keep] = obj0.simplify();
            
            verifyEqual(testCase, obj0, obj1)
            verifySize(testCase, keep, [obj0.numel()+1 1])
        end%fcn
    end
    
end%class
