function [POI,tau] = lineLineIntersection(P0, P1, Q0, Q1, doPlot)
%LINELINEINTERSECTION   Intersection of two lines.
%   POI = LINELINEINTERSECTION(P0,P1,Q0,Q1) returns the point of
%   intersection POI of the two lines connecting the points P0/P1 and
%   Q0/Q1.
% 
%   [___,TAU] = LINELINEINTERSECTION(___) returns the path parameters TAU =
%   [TAUP; TAUQ] for the corresponding line segments P0/P1 and Q0/Q1.
% 

dP = P1(:) - P0(:);
A = [dP Q0(:)-Q1(:)];

if rcond(A) > eps
    tau = A\(Q0(:) - P0(:));
    POI = P0(:) + dP*tau(1);
else
    tau = [NaN; NaN];
    POI = [NaN; NaN];
end


if (nargin > 4) && doPlot
    ax = gca;
    npState = get(ax, 'NextPlot');
    plot(ax, [P0(1) P1(1)], [P0(2) P1(2)], 'DisplayName','Line P0-P1')
    set(ax, 'NextPlot','add')
    plot(ax, [Q0(1) Q1(1)], [Q0(2) Q1(2)], 'DisplayName','Line Q0-Q1')
    plot(ax, POI(1), POI(2), 'rx')
    set(ax, 'XGrid','on', 'YGrid','on', 'NextPlot',npState);
end%if

end%fcn
