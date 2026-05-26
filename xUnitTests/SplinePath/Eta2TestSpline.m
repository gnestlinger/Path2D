classdef Eta2TestSpline < matlab.unittest.TestCase
% TestEta2  Unit tests for SplinePath.eta2
%
% These tests assume endpoint configuration vectors are provided as
% 4-by-(N+1) arrays: [x; y; phi; k] per column.

    methods (Test)
        function testOutputShape(testCase)
        % Two-segment example (3 breakpoints)
            P = [0   1   2; % x
                0   0.5 0;  % y
                0   0   0;  % phi (heading)
                0   0   0]; % k (curvature)
            eta = [1.0; 0.8; 0.5; -0.3];
            
            sp = SplinePath.eta2(P, eta);
            
            % Expect N = #segments = 2, polynomial order 5 -> 6 coeffs
            NExp = 2;
            testCase.verifyEqual(sp.numel(), NExp);
            [~,Ns,Np] = size(sp.Coefs);
            testCase.verifyEqual(Ns, NExp);
            testCase.verifyEqual(Np, 6);
            testCase.verifyEqual(sp.Breaks, 0:NExp);
        end%fcn
        
        function testG2ContinuityAtBreaks(testCase)
        % Create three endpoint configurations forming two segments with
        % non-trivial headings/curvatures so curvature continuity can be
        % checked.
            P = [0   1   2;   % x
                0   0.5 1.0;  % y
                0   0.2 0.4;  % phi
                0.0 0.1 0.0]; % k
            eta = [0.8; 0.6; 0.4; -0.2];
            
            sp = SplinePath.eta2(P, eta);
            
            % Evaluate at the middle breakpoint (tau = 1)
            tauL = 1 - 1e-7; % just left of break
            tauR = 1 + 1e-7; % just right of break
            
            [xL,yL,~,phiL,curvL] = sp.eval(tauL);
            [xR,yR,~,phiR,curvR] = sp.eval(tauR);
            
            % Position continuity
            posErr = hypot(xL - xR, yL - yR);
            testCase.verifyLessThan(posErr, 1e-6, ...
                sprintf('Position discontinuity at break: %g', posErr));
            
            % Heading continuity (wrap-aware)
            wrapDiff = @(a,b) abs(angdiff(a,b));
            headErr = wrapDiff(phiL, phiR);
            testCase.verifyLessThan(headErr, 1e-6, ...
                sprintf('Heading discontinuity at break: %g', headErr));
            
            % Curvature continuity
            curvErr = abs(curvL - curvR);
            testCase.verifyLessThan(curvErr, 1e-5, ...
                sprintf('Curvature discontinuity at break: %g', curvErr));
        end%fcn
        
        function testSymmetricEtaMatchesFullForm(testCase)
        % Construct a short path and show that passing a 2-element eta_
        % yields the same spline as explicitly expanded 4-element eta.
            P = [0  2;   % x
                0  1;   % y
                0  0;   % phi
                0  0];  % k
            a = 1.2;
            b = 0.7;
            
            % Symmetric shortcut
            eta_short = [a; b]; 
            sp_short = SplinePath.eta2(P, eta_short);
            
            % Explicit expansion used internally
            eta_full = [a; a; b; -b]; 
            sp_full  = SplinePath.eta2(P, eta_full);
            
            % Breaks and coefficients should match exactly
            testCase.verifyEqual(sp_short.Coefs, sp_full.Coefs);
            testCase.verifyEqual(sp_short.Breaks, sp_full.Breaks);
        end%fcn
    end
end%class

% Small helper: angle difference that handles wrapping
function d = angdiff(a,b)
d = mod(a - b + pi, 2*pi) - pi;
end
