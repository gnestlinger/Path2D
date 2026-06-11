function [xy,tau] = lineSegXline(xyPath, O, psi)
%LINESEGXLINE   Intersection of a line segment and an infinite line.
%   Detailed explanation goes here


% Shift by line origin O and rotate so that the line is horizontal -> We
% can find intersections by checking where the path's y-components equals
% zero!
R = rotmat2D(psi);
xyPath = bsxfun(@minus, xyPath, O(:)')*R;
xPath = xyPath(:,1);
yPath = xyPath(:,2);

% Find segment indexes where the paths y-component (A) changes sign or (B)
% equals zero using sign(), which returns 0 only for inputs that are
% exactly equal to zero. We try to catch values almost equal to zero via a
% magic threshold.
signsA = int8(sign(yPath)); 
signsB = abs(yPath) <= eps(O(1));
signsA(signsB) = int8(0);
idxs0 = find([abs(diff(signsA)) > 1; false] | signsB);
idxs0 = min(idxs0, size(xyPath,1) - 1);

if isempty(idxs0) % No intersection of path/line
    xy = zeros(0, 2);
    tau = zeros(0,1);
else
    % End index can not exceed number of path samples since indexes were
    % obtained using DIFF!
    idxsE = idxs0 + 1;
    x0F = [xPath(idxs0), xPath(idxsE)];
    y0Fd = diff([yPath(idxs0), yPath(idxsE)], 1, 2);
    x = xPath(idxs0) - yPath(idxs0) .* diff(x0F, 1, 2)./y0Fd; 

    % Undo transformation. Due to the above rotation/shift, the
    % intersections y-component is zero. Therefore, only the x-component
    % needs to be rotated.
%     xy = (R * [x';zeros(1,numel(x))] + repmat(O(:), [1,numel(x)]))';
    xy = [R(1,1)*x + O(1), R(2,1)*x + O(2)];

    % Since we assume linear interpolation between waypoints, the local
    % path segment parameter can be computed from x or y
%     tauLocal = (x - x0F(1))/diff(x0F);
    tauLocal = -yPath(idxs0)./y0Fd;
    tau = idxs0 - 1 + tauLocal;
end%if

% % Alternative approach using matrix inversion
% Q1 = O(:) + [cos(psi); sin(psi)];
% tau = zeros(0,1);
% for i = 1:size(xyPath, 1)
%    P0 = xyPath(i,:); 
%    P1 = xyPath(i+1,:); 
%    [~,tauPQ] = lineLineIntersection(P0, P1, O, Q1);
%    taui = tauPQ(1);
%    if taui >= 0 && taui <= 1
%        tau = [tau; taui + i - 1];
%    end
% end
            
end%fcn
