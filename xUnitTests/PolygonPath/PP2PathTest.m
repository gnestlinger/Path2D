classdef PP2PathTest < matlab.unittest.TestCase
    
    properties
        % Piecewise polynomial representing abs(), the first
        % segment has a slope of -45°, the second of +45°
        ppAbs = mkpp([-2 0 2], [0 1 -2; 0 -1 2; 0 1 0; 0 1 0], 2);
    end
    
    methods (Test)		
		function testHeadingFromPP(testCase)
            
            pp = testCase.ppAbs;
            N = 100; % Samples per segment
            
            t = linspace(pp.breaks(1), pp.breaks(2)-eps, N);
			obj0 = PolygonPath.pp2Path(pp, t);
			verifyEqual(testCase, obj0.head, -pi/4*ones(N,1), 'AbsTol',1e-16);
            
            t = linspace(pp.breaks(2), pp.breaks(3), N);
			obj0 = PolygonPath.pp2Path(pp, t);
			verifyEqual(testCase, obj0.head, +pi/4*ones(N,1), 'AbsTol',1e-16);
            
		end%fcn
        
        function testCurvatureFromPP(testCase)
			
            pp = testCase.ppAbs;
            N = 100; % Samples per segment
            
            t = linspace(pp.breaks(1), pp.breaks(2)-eps, N);
			obj0 = PolygonPath.pp2Path(pp, t);
			verifyEqual(testCase, obj0.curv, zeros(N,1), 'AbsTol',1e-16);
            
            t = linspace(pp.breaks(2), pp.breaks(3), N);
			obj0 = PolygonPath.pp2Path(pp, t);
            verifyEqual(testCase, obj0.curv, zeros(N,1), 'AbsTol',1e-16);
            
		end%fcn
    end
    
end%class
