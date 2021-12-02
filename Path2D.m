classdef Path2D
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
		
		% POINTPROJECTION	Point projection.
		[Q,idx,lambda] = pointProjection(obj, poi)
		
		% RESAMPLE	Resample path.
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
