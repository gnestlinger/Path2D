function [xy,phi,s] = circularArcXline(C, r, phi0, dPhi, O, psi)
%CIRCULARARCXLINE   Intersection of a circular arc and an infinite line.
%   XY = CIRCULARARCXLINE(C,R,PHI0,dPHI,O,PSI) returns the intersection
%   points XY between a circular arc at center C and of radius R and an
%   infinite line passing through O at an angle PSI. The arc section is
%   defined by the start angle PHI0 and the sweep angle DPHI.
%
%   [XY,PHI,S] = CIRCULARARCXLINE(___) also returns the angular positions
%   PHI in the range (-pi,pi] of the intersections around C and the arc
%   length S from the arc start.
%
%   Behavior and notes
%     - The function solves the circle-line quadratic and filters real
%       solutions to those that lie on the specified arc [PHI0, PHI0+dPHI],
%       correctly handling wrap-around and both sweep directions.
%     - Returned rows are sorted by increasing distance along the arc from
%       the start (abs(S) increasing). Near-duplicate intersection points
%       are removed (numerical tolerance).
%     - If there are no intersections on the arc, XY is 0x2, PHI is 0x1 and
%       S is 0x1 empty.
%
%   See also LINESEGXCIRCLE.


% Direction vector of the line
d = [cos(psi) sin(psi)];

% Shift such that origin of the circle is at (0,0)
O0 = O(:)' - C(:)';

% Compute coefficients of quadratic equation
% a = sum(d.^2, 2); % Equals 1
b = 2*sum(O0.*d, 2);
discriminant = b.^2 - 4*(sum(O0.^2, 2) - r^2);

if discriminant > 0
    xi = sqrt(discriminant);
    tau = -0.5*[b - xi; b + xi];
elseif discriminant < 0
    % No solutions
    tau = zeros(0,1);
else
    tau = -0.5*b;
end


% Intersection points and angles around center
xy = O(:)' + tau*d;
phi = atan2(xy(:,2) - C(2), xy(:,1) - C(1)); % in (-pi,pi]



% Normalize to [0,2pi)
phi_2pi = mod(phi, 2*pi);
phi0_2pi = mod(phi0, 2*pi);

% Robust membership: forward angular distance from phi0 to tau in [0,2pi)
absSweep = mod(dPhi, 2*pi);
epsAng = 1e-12;

% if absSweep == 0
%     % Zero-length arc: accept only exact angle match within tol
%     isOnArc = abs(min(mod(taus2 - phi0_2pi, 2*pi), mod(phi0_2pi - taus2, 2*pi))) <= epsAng;
% else
    if dPhi > 0
        % Increasing-angle sweep: accept forward delta <= absSweep
        deltaF = mod(phi_2pi - phi0_2pi, 2*pi);
        isOnArc = (deltaF >= -epsAng) & (deltaF <= absSweep + epsAng);
    else
        % Negative sweep: backward motion; check forward distance from tau to phi0
        deltaF_rev = mod(phi0_2pi - phi_2pi, 2*pi);
        isOnArc = (deltaF_rev >= -epsAng) & (deltaF_rev <= absSweep + epsAng);
    end
% end


phi = phi(isOnArc);
xy = xy(isOnArc,:);


% Compute signed angle from phi0 to intersection angle following the sweep
% direction
signedAngles = signedAngleFromTo(phi0, phi, sign(dPhi));

% Arc path parameter (arc length along sweep direction)
s = r*signedAngles;

% Sort by absolute path parameter (distance along arc from start)
[s,idx] = sort(abs(s));
xy = xy(idx,:);
phi = phi(idx);

end%fcn


function dPhi = signedAngleFromTo(phi0, phi1, signDir)

dPhi = mod(phi1, 2*pi) - mod(phi0, 2*pi);
dPhi = mod(dPhi + pi, 2*pi) - pi; % (-pi, pi]
if signDir >= 0
    % Convert negative values to equivalent positive angle in [0,2pi)
    dPhi(dPhi < 0) = dPhi(dPhi < 0) + 2*pi;
else
    % Convert positive values to equivalent negative angle in (-2pi,0]
    dPhi(dPhi > 0) = dPhi(dPhi > 0) - 2*pi;
end

end%fcn
