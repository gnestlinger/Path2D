function [r1,r2] = scaleTangentToAxis(xLimits,yLimits,xy,phi)
%SCALETANGENTTOAXIS		Scale length of tangent to axis limits.
%   [R1,R2] = SCALETANGENTTOAXIS(XLIMITS,YLIMITS,XY,PHI) calculates the
%   lengths R1 and R2 for the tangent at point XY with angle PHI, so that
%   the tangent does not exceed given limits XLIMITS = [xMin xMax] and
%   YLIMITS = [yMin yMax].
%	Length R1 counts in the direction of PHI, whereas R2 counts in
%	the opposite direction.
%
%	The intended usage is for plotting something like tangents in existing
%	plot figures maintaining the current axis limits.


	% assign inputs to meaningful variables
	xMin = xLimits(1);
	xMax = xLimits(2);
	yMin = yLimits(1);
	yMax = yLimits(2);
	xT = xy(1);
	yT = xy(2);


	%%% map phi to [-pi,+pi) (plus/minus pi)
	if phi >= pi
		phi_pmPi = phi - 2*pi;
	else
		phi_pmPi = phi;
	end%if


	%%% reduce problem to angles of [-pi/2,+pi/2]
	if phi_pmPi <= pi/2 && phi_pmPi >= -pi/2
		% keep the original value
		phi_pmPi_ = phi_pmPi;
		isPhiReversed = false;
	else
		% consider the opposite direction of phi which is whithin
		% [-pi/2,+pi/2]
		phi_pmPi_ = mod(phi_pmPi - pi,pi); % ??? consider the sign of x in y for mod(x,y)
		isPhiReversed = true;
	end%if


	%%% calculate the equation of a line at [xT yT] for phi
	% y(xT) = yT -> k*xT + d = yT -> d = yT - k*xT
	k1 = tan(phi_pmPi_); % tangents slope
	d = yT - k1*xT; % tangents offset
	y_x = @(x) k1*x + d; % tangents equation of a line


	%%% calculate the plot-range [xL,xR] depending on the sign of the
	%%% tangents slope
	if k1 >= 0

		if y_x(xMax) <= yMax
			xR = xMax;
		else
			xR = (yMax-d)/k1;
		end%if

		if y_x(xMin) >= yMin
			xL = xMin;
		else
			xL = (yMin-d)/k1;
		end%if

	else

		if y_x(xMax) >= yMin
			xR = xMax;
		else
			xR = (yMin-d)/k1;
		end%if

		if y_x(xMin) <= yMax
			xL = xMin;
		else
			xL = (yMax-d)/k1;
		end%if

	end%if


	%%% calculate the tangent lengths
	r1 = +sqrt((xR-xT)^2 + (y_x(xR)-y_x(xT))^2);
	r2 = -sqrt((xL-xT)^2 + (y_x(xL)-y_x(xT))^2);

	% fix tangent lengths if the slope is +-inf
	if phi_pmPi_ == +pi/2
		r1 = yMax - yT;
		r2 = yMin - yT;
	elseif phi_pmPi == -pi/2
		r1 = -(yMin - yT);
		r2 = -(yMax - yT);
	end%if


	%%% switch R1/R2 and their signs if the opposite problem was considered
	if isPhiReversed
		dummy = r1;
		r1 = -r2;
		r2 = -dummy;
	end%if

end%fcn