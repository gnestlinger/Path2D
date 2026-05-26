function dp = polyDiff(p)%#codegen
%POLYDIFF	Polynomial differentiation supporting code generation.
%	DP = POLYDIFF(P) calculates the polynomial derivative DP of polynomial
%	whose coefficients are given in the vector P in descending order.
% 
% See also POLYDER.

% Order of polynomial p
NP = numel(p) - 1;

if NP > 0
	% Multiply polynomial coefficients with factors resulting from
	% differentiation. Coefficient of order 0 drops out due to
	% differentiation.
	dp = p(1:end-1);
	dp = dp .* reshape(NP:-1:1, size(dp));
else
	dp = 0;
end%if

end%fcn
