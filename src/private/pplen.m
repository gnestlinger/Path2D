function [l,err] = pplen(ppd, tau0, tau1)
%PPLEN  Length of piecewise polynomial path.
%   L = PPLEN(PPD,TAU0,TAU1) computes the arc length of a piecewise
%   polynomial curve PP with derivative PPD, from PP(TAU0) to PP(TAU1).
%
%   NOTE: PPD is assumed to be derivative of the arc PP w.r.t. the path
%   parameter TAU.


fun = @(tau) sqrt(sum(ppval(ppd, tau).^2, 1));
[l,err] = quadgk(fun, tau0, tau1);
l = abs(l);

end%fcn
