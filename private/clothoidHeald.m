function [x,y] = clothoidHeald(s, A)
%CLOTHOIDHEALD  Heald approximation of clothoid.
% 
%   According to "Rational Approximations for the Fresnel Integrals",
%   Heald, 1985. See also "Sketching Piecewise Clothoid Curves", McCrae &
%   Singh, 2008.
% 
Rt = (0.506*s + 1)./(1.79*s.^2 + 2.054*s + sqrt(2));
At = 1./(0.803*s.^3 + 1.886*s.^2 + 2.524*s + 2);
x = A*sqrt(pi)*(0.5 - Rt.*sin(pi/2*(At - s.^2)));
y = A*sqrt(pi)*(0.5 - Rt.*cos(pi/2*(At - s.^2)));

end%fcn
