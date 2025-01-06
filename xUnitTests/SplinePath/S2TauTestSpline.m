classdef S2TauTestSpline < matlab.unittest.TestCase
    
    properties (TestParameter)
    end
    
    
    
    methods (Test)
        function testReturnValuesStraightPath(testCase)
        % Test return values on a straight path.
        
            % Straight path of length 20
            obj = SplinePath([0 10], cat(3, [2;0], [0;0]));
            
            % Test also extrapolation
            s = [-1 -0.2 0 0.1 100 2 4 5 9 20];
            [tau,idx] = obj.s2tau(s);
            
            % Since the path is a line segment, we can use linear
            % interpolation for s/tau mapping
            [tau0,tau1] = obj.domain();
            tauExp = interp1([0 obj.length], [tau0 tau1], s, 'linear','extrap');
            
            verifyEqual(testCase, tau, tauExp, 'AbsTol',2e-16);
            verifyEqual(testCase, idx, uint32([1 1 1 1 1 1 1 1 1 1]));
        end%fcn
        
        function testReturnValuesQuadratic(testCase)
        % Test return values on an alternating quadratic polynomial.
        
            % Get test path and expected values
            [obj,sExp,tauExp,idxExp] = quadraticSpline(101);
             
            [tau,idx] = obj.s2tau(sExp);
            verifyEqual(testCase, tau, tauExp, 'AbsTol',2e-6, 'RelTol',0.05);
            verifyEqual(testCase, idx, idxExp);
        end%fcn
        
        function testReturnValuesQuadraticUnsorted(testCase)
        % Test return values on an alternating quadratic polynomial for
        % unsorted values of s.
        
            % Get test path and expected values
            [obj,sExp,tauExp,idxExp] = quadraticSpline(101);
            
            % Randomly rearrange s/tau/index 
            reorder = randperm(numel(sExp));
            sExp = sExp(reorder);
            tauExp = tauExp(reorder);
            idxExp = idxExp(reorder);
            
            [tau,idx] = obj.s2tau(sExp);
            verifyEqual(testCase, tau, tauExp, 'AbsTol',2e-6, 'RelTol',0.05);
            verifyEqual(testCase, idx, idxExp);
        end%fcn
        
        function testReturnValuesOutOfBounds(testCase)
            obj = SplinePath([0 2 5], cat(3, [2 1;1 0], [0 4;0 2]));
%             obj.plot('.')
            
            s = [-1  14];
            [tau,idx] = obj.s2tau(s);
            
            tauSet = interp1([0;obj.cumlengths()], obj.Breaks, s, 'linear', 'extrap');
            verifyEqual(testCase, tau, tauSet, 'AbsTol',1e-12);
            verifyEqual(testCase, idx, uint32([1 2]), 'AbsTol',1e-12);
        end%fcn
    end
    
end%class


function [obj,s,tau,idx] = quadraticSpline(N)

% Create alternating quadratic spline path
breaks = [-2 2 6];
coefs = [1 -4 0];
obj = SplinePath(breaks, reshape([0 1 -2; coefs; 0 1 2; -coefs], [2 2 3]));

% The length for a quadratic polynomial f(x) = x^2 can be computed in
% closed form:
%   Wolfram alpha: Integral sqrt(1+4*t^2)
fhTrueL = @(x) 0.5*x.*sqrt(4*x.^2+1) - 0.25*log(sqrt(4*x.^2+1) - 2*x);

% Given the closed form for s(x) and using x(tau) = tau, we can compute a
% mapping [tau s(tau)]. Compute s/tau for first segment explicitly:
s1 = fhTrueL(linspace(breaks(1), breaks(2), N)') - fhTrueL(breaks(1));
s = [s1; s1(2:end) + s1(end)];
tau = linspace(breaks(1), breaks(end), numel(s))';

idx = zeros(size(tau), 'uint32');
idx(tau <= breaks(3)) = 2;
idx(tau < breaks(2)) = 1;

end%fcn

function tau = s2tauRef(obj, s)

ppd = obj.derivative.mkpp();
breaks = unmkpp(ppd);

tau = zeros(size(s));
fhLen = @(t) sqrt(sum(ppval(ppd, t).^2, 1));
for i = 1:numel(s)
    fun = @(t1) abs(quadgk(fhLen, breaks(1), t1) - s(i));
    tau(i) = fminbnd(fun, breaks(1), breaks(end));
end

end%fcn
