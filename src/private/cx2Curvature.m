function [curv,hyp] = cx2Curvature(d1x, d1y, d2x, d2y)
% (x' y'' - x'' y')/(x'^2 + y'^2)^(3/2)
hyp = hypot(d1x, d1y);
curv = (d1x.*d2y - d2x.*d1y) ./ (hyp.*hyp.*hyp);
end%fcn
