function [x,y] = clothoidQuad(s, A)
%CLOTHOIDQUAD   Numerical integration of clothoid.
%   [X,Y] = CLOTHOIDQUAD(S,A) calculates the clothoid points Y(X) for
%   the vector of curve length S using the chlothoid parameter A>0.
% 

% Check input arguments
s = s(:);
if any(diff(s) == 0)
    error('Input argument S must not contain repeated elements!');
elseif abs(sum(sign(diff(s)))) ~= length(s)-1
    error('Input argument S must be strictly monotonically decreasing/increasing!');
end%if
assert(A > 0, 'Clothoid parameter A has to be positive!');


% Pre-allocation
N = length(s);
x(1,N) = 0;
y(1,N) = 0;
% xy(2,N) = 0;

% Clothoid integrands
fresnelC = @(t) cos(pi/2*t.^2);
fresnelS = @(t) sin(pi/2*t.^2);
fresneCS = @(t) [cos(pi/2*t.^2); sin(pi.*t.^2./2)];

s = s/(sqrt(pi)*A);

% Numerical integration
donePercentOld = 0;
fprintf('    ');
if s(1) ~= 0
    x(1) = quadl(fresnelC, 0, s(1));
    y(1) = quadl(fresnelS, 0, s(1));
%     xy(:,1) = integral(fresneCS, 0, s(1), 'ArrayValued',true);
end
for i = 2:N
    x(i) = x(i-1) + quadl(fresnelC, s(i-1), s(i));
    y(i) = y(i-1) + quadl(fresnelS, s(i-1), s(i));
%     xy(:,i) = xy(:,i-1) + integral(fresneCS, s(i-1), s(i), 'ArrayValued',true);
    donePercent = i/N*100;
    if (donePercent-donePercentOld) > 9.99
        fprintf('%d%% ', round(donePercent));
        donePercentOld = donePercent;
    end%if
end%for
fprintf('\n');

x = x*A*sqrt(pi);
y = y*A*sqrt(pi);
% xy = xy*A*sqrt(pi);

end%fcn
