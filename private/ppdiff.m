function dpp = ppdiff(pp, polyDeg)%#codegen
%PPDIFF		Derivative of piecewise polynomial.
%	DPP = PPDIFF(PP) returns the piecewise polynomial derivative DPP of
%	piecewise polynomial PP.
% 
%	See also MKPP, UNMKPP, PPVAL.


% Extract PP details
[breaks,coefs,~,polyOrd,dim] = unmkpp(pp); % POLYORD = POLYDEG+1

% For code generation purpose, polynomial degree can be set via input
% argument to a compile-time constant (The dimension of COEFS that
% determines the order must be fixed-size.).
if nargin < 2
	polyDeg = polyOrd - 1;
end%if


% Derivative of polynomial coefficients
if polyDeg > 0
	dcoefs = bsxfun(@times, coefs(:,1:polyDeg), double(polyDeg):-1:1);
else
	dcoefs = coefs*0;
end%if

% Create PP with derivative polynomial coefficients
dpp = mkpp(breaks, dcoefs, dim);

end%fcn
