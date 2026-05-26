function idx = ramerDouglasPeucker(x, y, epsilon)
%RDP    Ramer-Douglas-Peucker point reduction.
%    IDX = RDP(X,Y,EPS) applies the Ramer-Douglas-Peuker point reduction
%    algorithm to the waypoints (X,Y) with parameter EPS. None of the
%    removed waypoints has a distance greater than EPS to the resulting
%    path. 
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
	d = perpDist(x(idx0:idx1), y(idx0:idx1));
	[val_max,idx_max] = max(abs(d));
	if val_max > epsilon
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

idx = (keepIdx);

end%fcn
