function d = perpDist(x, y)
%PERPDIST   Perpendicular distance to line through terminal points.
%   D = PERPDIST(X,Y) calculates the perpendicular distance for all points
%   (X,Y) to the line going through [X(1) Y(1)] and [X(end) Y(end)].

% See:
% https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points

x0 = x(1);
y0 = y(1);
dx = x(end) - x0;
dy = y(end) - y0;
h = hypot(dx, dy);
if h ~= 0
    d = abs(dx*(y-y0) - dy*(x-x0))/h;
else
    d = hypot(x - x0, y - y0);
end

end%fcn
