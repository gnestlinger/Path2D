function P = exHlpErrMdlPurePursuit(refPath, pose, LAD)
%ERRMDLPUREPURSUIT  Pure Pursuit path tracking point.
%
%   P = ERRMDLPUREPURSUIT(REFPATH,POSE,LAD) returns the look-ahead point P
%   on the path for a vehicle pose POSE and a look-ahead distance LAD.

xyVhcl = [pose(1) pose(2)];   

% Find all circle/path intersections
[~,tauCand] = refPath.intersectCircle(xyVhcl, LAD, false);

% From multipe solutions select the one closest in forward direction
[~,~,~,tauVhcl] = refPath.cart2frenet(xyVhcl);

if numel(tauVhcl) ~= 1
    % TODO: Handle the case of no or non-unique solutions.
    error('Non-unique solution!')
end

tau = NaN;
for i = 1:numel(tauCand)
    taui = tauCand(i);
    if taui > tauVhcl
        tau = tauCand(i);
        break
    end
end
[xLAD,yLAD] = refPath.eval(tau);

P = [xLAD yLAD];

end%fcn
