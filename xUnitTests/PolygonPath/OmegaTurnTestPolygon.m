classdef OmegaTurnTestPolygon < matlab.unittest.TestCase
    
    properties (TestParameter)
        r = {1 5 10};    
        w = {-10 -5 -1 1 5 10};
        N = {6, 10, 100};
        phi0 = {0, pi/2, -pi/2}
    end
    
    
    methods (Test)
        
        function testPathProperties(testCase, r, w, N)
                
            if abs(w) >= 2*r
                testCase.verifyThat(@() PolygonPath.omegaTurn(r, w, N), ...
                    matlab.unittest.constraints.Throws(?MException))
                return
            end
            
            obj = PolygonPath.omegaTurn(r, w, N);
            
            % Number of path elements
            testCase.verifyEqual(numel(obj.x), N);
            
            % Path length
            s = obj.cumlengths();
            verifySize(testCase, s, [N-1 1]);
            verifyTrue(testCase, all(diff(s) > 0));
            verifyTrue(testCase, s(1) > 0);
            
            % Terminal points
            [A,B] = obj.termPoints();
            verifyEqual(testCase, A, [0;0], 'AbsTol',1e-20);
            verifyEqual(testCase, B, [0;w], 'AbsTol',1e-14);
            
        end%fcn
        
        function testInitialHeading(testCase, r, w, N, phi0)
            %TODO
        end%fcn
        
    end
    
end%class
