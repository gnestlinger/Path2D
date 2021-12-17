classdef (InferiorClasses = {?matlab.graphics.axis.Axes}) Path2D
%PATH2D		Represent 2D paths.
%	This class is abstract and can not be instantiated!
%	
%	Represent paths in the planar x/y coordinate frame. It provides path
%	operations that can be especially usefull for automated driving and
%	robotics applications.
% 
%	Path2D properties:
%	IsCircuit - Indicates if path is a circuit.
% 
%	Path2D methods:
%	append - Append paths.
%	cart2frenet - Convert cartesian point to frenet coordinates.
%	frenet2cart - Convert frenet point to cartesian coordinates.
%	getPathLength - Get lengths of path segments.
%	pointProjection - Point projection on path.
%	resample - Resample path.
%	numel - Number of path elements.
% 
%	Path2D visualization methods:
%	plot - Plot the path.
% 
%	Path2D static methods:
%	pp2Path - Construct path object from piecewise polynomial.
%	xy2Path - Construct path object from cartesian coordinates.
%
%	See also .

	properties
		% ISCIRCUIT - Logical indicating if path is a circuit.
		IsCircuit = false
	end
	
	
	
	methods
		function obj = Path2D()
		%PATH2D		Construct a PATH2D class instance.
			
		end%Constructor
		
		
		function [h,ax] = plot(objORax, varargin)
		%	PLOT(OBJ) plots the path OBJ in terms of x over y.
		%	
		%	PLOT(OBJ,S) additionally applies the line specification S.
		%
		%	PLOT(...,NAME,VALUE) specifies line properties using one or
		%	more Name,Value pair arguments.
		%
		%	PLOT(AX,...) plots into the axes with handle AX.
		%	
		%	[H,AX] = PLOT(...) returns the handle H to lineseries objects
		%	and axes handle AX.
		%	
		%	The line specification S is a character string supported by the
		%	standard PLOT command. For example
		%		PLOT(OBJ, 'LineWidth',2, 'Color',[.6 0 0]) 
		%	will plot a dark red line  using a line width of 2 points.
		%
			
			if isa(objORax, 'matlab.graphics.axis.Axes')
				ax = objORax;
				obj = varargin{1};
				opts = varargin(2:end);
			elseif isa(objORax, 'Path2D')
				ax = gca;
				obj = objORax;
				opts = varargin;
			else
				error('This should not happen!')
			end%if
			
			wp = getWaypoints(obj);
			[h,ax] = plot(ax, wp, opts{:});
			
		end%fcn
	end%methods
	
	
	methods (Abstract)
		
		% APPEND	Append paths.
		%	OBJ = append(OBJ0,OOBJ1,...,OBJN) appends paths OBJ to OBJN in
		%	the given order creating path OBJ.
		obj = append(obj0, varargin)
		
		% CART2FRENET	Cartesian point to frenet with respect to path.
		%	SD = CART2FRENET(OBJ,XY) converts point XY in cartesian
		%	coordinates to frenet coordinates SD with respect to the path
		%	OBJ. Point XY is a two-element vector .
		%
		%	[SD,Q,IDX,LAMBDA] = CART2FRENET(___) also return 
		%	 - The cartesian point Q that corresponds to the frenet point SD.
		%	 - An index IDX indicating the path segment Q relates to.
		%	 - The path parameter LAMBDA.
		[sd,Q,idx,lambda] = cart2frenet(obj, cart)
		
		% FRENET2CART	Frenet point to cartesian with respect to path.
		%   XY = FRENET2CART(OBJ,SD) converts points SD in frenet
		%   coordinates to cartesian coordinates XY with respect to the
		%   path OBJ. Matrix SD is of size n-by-2.
		% 
		%	[SD,Q,IDX] = FRENET2CART(___) returns 
		%	 - The Point Q that corresponds to the frenet points SD.
		%	 - An index IDX referring to the path segment Q relates to.
		[x,y] = frenet2cart(obj, s, d)
		
		% GETPATHLENGTH		Path length.
		%	S = GETPATHLENGTH(OBJ)
		s = getPathLength(obj)
		
		% GETWAYPOINTS	Get sampled path waypoints.
		%	WP = GETWAYPOINTS(OBJ,N) returns WAYPOINTS object WP containing
		%	N waypoints.
		wp = getWaypoints(obj, N)
		
		% POINTPROJECTION	Point projection.
		%	Q = POINTPROJECTION(OBJ,POI) returns the orthogonal projection
		%	Q of POI onto the path OBJ.
		%
		%	[Q,IDX,LAMBDA] = POINTPROJECTION(OBJ,POI) also returns the path
		%	segment IDX and path path parameter LAMBDA related to Q.
		[Q,idx,lambda] = pointProjection(obj, poi)
		
		% RESAMPLE	Resample path.
		%	OBJ = RESAMPLE(OBJ,N) resamples path OBJ over path length to N
		%	sample points.
		%
		%	OBJ = RESAMPLE(__,ARGS) specify interpolation settings via
		%	ARGS.
		obj = resample(obj, N, varargin)
		
		% NUMEL		Number of path elements.
		%	N = NUMEL(OBJ) returns the number of path elements, e.g. 
		%	 - The number of waypoints for path defined by waypoints.
		%	 - The number of segments for path defined as a spline.
		N = numel(obj)
		
	end%methods
	
	
	methods (Abstract, Static)
		
		% PP2PATH	Path object from piecewise polynomial.
		%	OBJ = PP2PATH(PP,TAU)
		%
		%	See also MKPP.
		obj = pp2Path(pp, tay, polyDeg)
		
		% XY2PATH	Path object from cartesian coordinates.
		%	OBJ = XY2PATH(X,Y)
		obj = xy2Path(x, y)
		
	end%methods
	
end%class
