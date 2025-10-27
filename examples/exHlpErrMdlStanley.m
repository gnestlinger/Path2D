function P = exHlpErrMdlStanley(refPath, pose)
%ERRMDLSTANLEY  Stanley path tracking point.
%
%   P = ERRMDLSTANLEY(REFPATH,POSE) returns the look-ahead point P on the
%   path for a vehicle pose POSE.

xyVhcl = [pose(1) pose(2)];   

[P,~,~,tau] = refPath.pointProjection(xyVhcl, [], false);
if numel(tau) == 1
else
    % TODO: Handle the case of no or non-unique solutions.
    error('Non-unique solution!')
end

end%fcn
