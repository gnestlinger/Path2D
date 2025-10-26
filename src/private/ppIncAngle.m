function phi = ppIncAngle(pp, ppd, P, tau)
% Return included angle PHI between point P and tangent of piecewise polynomial
% for a certain path parameter TAU.

w = P(:) - ppval(pp, tau);
dxy = ppval(ppd, tau);
phi = atan2(w(2), w(1)) - atan2(dxy(2), dxy(1));

end%fcn
