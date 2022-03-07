function [xc,yc,R,e] = fitCircle_Kasa(x,y)
%FITCIRCLE_KASA		Fit a circle to set of points.
%	[XC,YC,R,E] = FITCIRCLE_KASA(X,Y) fits a circle with center(XC,YC) and
%	radius R to a set of given points (X,Y) minimizing the error E.
%
%	The error E is calculated according to:
%			  N
%	 E = 1/N*SUM[(R_i^2 - R^2)^2]
%			 i=1
%
%	This method is based on I. Kasa, "A curve fitting procedure and its
%	error analysis", IEEE Trans. Inst. Meas., Vol. 25, pages 8-14, (1976)

if numel(x) ~= numel(y)
	error('X and Y must have the same number of elements!');
end%if
% create system of equations
A = [...
	sum(x)		sum(y)		numel(x);...
	sum(x.^2)	sum(x.*y) sum(x);...
	sum(x.*y) sum(y.^2)	sum(y)];
b = [...
	sum(x.^2 + y.^2);...
	sum(x.^3 + x.*y.^2);...
	sum(x.^2.*y + y.^3)];

% solve it
xOpt = A\b;

% get single parameters out of solution
xc	= xOpt(1)/2;
yc	= xOpt(2)/2;
R	= sqrt(xOpt(3) + xc^2 + yc^2);

% calculate error
Ri = sqrt((x-xc).^2 - (y-yc).^2);
e = sum((Ri.^2 - R^2).^2)/numel(x);

end%fcn