function [xy,tau] = lineSegXCircle(xyPath, C, r)
%LINESEGXCIRCLE     Intersection of a line segment and a circle.
%   Detailed explanation goes here

idxs = (1:size(xyPath,1))';

xyPath = bsxfun(@minus, xyPath, C(:)');
dxy = diff(xyPath, 1, 1);

% Each path segment is written as a line 
%   P(t) = P0 + t*(P1-P0) 
% from its initial point P0 to its end point P1, where t = [0,1]. Using the
% implicit equation 
%   x^2 + y^2 = r^2 
% of a circle, we get 
%   [x0 + t(x1-x0)]^2 + [y0 + t(y1-y0)]^2 = r^2
% which requires solving a quadratic polynomial 
%   a*t^2 + b*t + c = 0
a = sum(dxy.^2, 2);
b = 2*sum(xyPath(1:end-1,:).*dxy, 2);
c = sum(xyPath(1:end-1,:).^2, 2) - r^2;
discriminant = b.^2 - 4*a.*c;

%%% Case 1: Discriminant > 0
% We have two solutions from the quadratic equation (per segment), i.e. a
% secant line.
isSecant = (discriminant > 0);
xi = sqrt(discriminant(isSecant));
tauSecant = 0.5*[...
    (-b(isSecant) + xi)./a(isSecant); ...
    (-b(isSecant) - xi)./a(isSecant)];
idxSecant = repmat(idxs(isSecant), [2 1]);
isValidSec = ~((tauSecant < 0) | (tauSecant > 1));

%%% Case 2: Discriminant = 0
% We have one solution from the quadratic equation (per segment), i.e. a
% tangent line.
isTangent = ~((discriminant < 0) | isSecant); % (discriminant == 0)
tauTangent = -0.5*b(isTangent)./a(isTangent);
idxTangent = idxs(isTangent);
isValidTan = ~(tauTangent < 0) & (tauTangent < 1);

%%% Case 3: Discriminant < 0
% Quadratic formula has complex solutions -> No intersections

% Combined set of solutions
tauLoc = [tauSecant(isValidSec); tauTangent(isValidTan)];
segIdx = [idxSecant(isValidSec); idxTangent(isValidTan)];

% Set return values
tau = sort(segIdx - 1 + tauLoc, 'ascend');
xy = interp1(xyPath, tau + 1);
xy = bsxfun(@plus, xy, C(:)');

% At most two intersections per path segment!
assert(size(xy, 1) <= (size(xyPath, 1)-1)*2)
assert(size(xy, 1) == size(tau, 1))

end%fcn
