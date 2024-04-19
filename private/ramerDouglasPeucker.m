function [xr,yr,idx] = ramerDouglasPeucker(x, y, eps)
%RDP    Ramer-Douglas-Peucker point reduction.
%    [XR,YR] = RDP(X,Y,EPS) applies the Ramer-Douglas-Peuker point
%    reduction algorithm to waypoints (X,Y) with parameter EPS. None of
%    the removed waypoints has a distance greater than EPS to the
%    resulting path!
%    
%    [___,IDX] = RDP(X,Y,EPS) returns an array IDX so that XR = X(IDX) and
%    YR = Y(IDX).
%

%    NOTE: This implementation was inspired by dpsimplify.m by
%    Wolfgang Schwanghart found at MathWorks File Exchange.

% Initialize a logical array indicating which waypoints to keep
N = numel(x);
keepIdx = true(1, N);

% Recursively set indexes of waypoints that can be discarded to
% false
dprec(1, N);

function dprec(idx0, idx1)
	d = perpendicularDistance(x(idx0:idx1), y(idx0:idx1));
	[val_max,idx_max] = max(abs(d));
	if val_max > eps
		% Split waypoints at IDX_SPLIT and call recursion with those two
		% resulting segments until we end in the else statement.
		idx_split = idx_max + idx0 - 1;
		dprec(idx0, idx_split);
		dprec(idx_split, idx1);
	else
		 if idx0 ~= idx1-1
			keepIdx(idx0+1:idx1-1) = false;
		 end%if
	end%if
end%fcn

idx = find(keepIdx);
xr = x(idx);
yr = x(idx);

end%fcn


function d = perpendicularDistance(x, y)
% Calculate the perpendicular distance for all points to the line through
% the terminal points. See:
% https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
x1 = x(1);
y1 = y(1);
dx = x(end) - x1;
dy = y(end) - y1;
h = hypot(dx, dy);
if h ~= 0
    d = abs(dx*(y-y1) - dy*(x-x1))/h;
else
    d = hypot(x-x1, y-y1);
end

end%fcn
