classdef (InferiorClasses = {?matlab.graphics.axis.Axes}) SplinePath < Path2D
%SPLINEPATH     Spline path.
%   Path representation using splines.
% 
%	SPLINEPATH properties:
%	breaks - Cartesian x-coordinate.
%	coefs - Cartesian y-coordinate.
% 
%	SPLINEPATH methods:
%	SplinePath - Constructor.
%	See superclasses.
% 
%	SPLINEPATH static methods:
%	See superclasses.
% 
%	See also PATH2D.

%#ok<*EVLC> % Using eval and evalc with two arguments is not recommended ..

	properties
		Breaks = [0 1]
        Coefs = zeros(2,0,4) % path dimension by # of segments by # order
	end
	
	
	
	methods
		
		function obj = SplinePath(breaks, coefs)
		%SPLINEPATH     Create spline path object.
		%	OBJ = SPLINEPATH(X,Y,HEAD,CURV) 
		%
		%	OBJ = SPLINEPATH(___,ISCIRCUIT) set to true if the path is a
		%	circuit.
		%
		
            if nargin < 1
				% Return with default values
				return
            end
			
            assert(size(coefs,1) == 2, 'Coefficients must be 2D!');
			assert(isequal(numel(breaks)-1, size(coefs,2)), ...
                'Breaks and spline segments do not match!');
			obj.Breaks = breaks;
			obj.Coefs = coefs;
			
			if nargin < 3
				obj.IsCircuit = false;
			else
				obj.IsCircuit = isCircuit;
			end%if
			
		end%Constructor
		
		function obj = append(obj, obj2)
            
            breaks = obj.Breaks;
            obj.Breaks = [breaks, obj2.Breaks(2:end) + breaks(end)];
            
            coefs = obj.Coefs; % TODO: handle non-matching degree
            obj.Coefs = cat(2, obj.Coefs, obj2.Coefs);
			
		end%fcn
		
		function [sd,Q,idx,lambda] = cart2frenet(obj, poi, doPlot)
			
		end%fcn
        
        function [x,y,head,curv] = eval(obj, tau)
            
            if nargin < 2
                breaks = obj.Breaks;
                dbreaks = diff(breaks);
                tau = linspace(breaks(1), breaks(end), 1e3);
            end
            pp = mkpp(obj.Breaks, obj.Coefs, 2);
            xy = ppval(pp, tau);
            x = xy(1,:)';
            y = xy(2,:)';
            
            if nargout > 2
                ppD1 = ppdiff(pp);
                xyD1 = ppval(ppD1, tau);
                head = cx2Heading(xyD1(1,:), xyD1(2,:))';
                
                if nargout > 3
                    ppD2 = ppdiff(ppD1);
                    xyD2 = ppval(ppD2, tau);
                    curv = cx2Curvature(xyD1(1,:), xyD1(2,:), xyD2(1,:), xyD2(2,:))';
                end
            end%if
            
        end%fcn
		
		function [xy,Q,idx] = frenet2cart(obj, sd)
            
		end%fcn
        
        function [tauL,tauU] = getDomain(obj)
            tauL = obj.Breaks(1);
            tauU = obj.Breaks(end);
        end%fcn
		
		function s = getPathLengths(obj, idx0, idx1)
            
		end%fcn
        
        function [xy,tau,errFlag] = intersectLine(obj, O, psi)
        end%fcn
        
        function [xy,tau,errFlag] = intersectCircle(obj, C, r)
        end%fcn
        
        function flag = isempty(obj)
            flag = (numel(obj) < 1);
        end%fcn
        
		function s = length(obj, idx0, idx1)
			
		end%fcn
		
		function [Q,idx,lambda] = pointProjection(obj, poi)
			
		end%fcn
		
		function n = numel(obj)
			n = size(obj.Coefs, 2); % Return number of spline segments
		end%fcn
        
        function obj = reverse(obj)
			
		end%fcn
        
        function obj = rotate(obj, phi)
            
            narginchk(1, 2);
            
            if nargin < 2
                tau0 = getDomain(obj);
                [~,~,phi] = eval(obj, tau0);
            end%if
            
            R = rotmat2D(-phi);
            for i = 1:builtin('numel', obj)
                obji = obj(i);
                coefs = obji.Coefs;
                for ii = 1:numel(obji)
                   coefs(:,ii,:) = R*squeeze(coefs(:,ii,:));
                end
                obj(i).Coefs = coefs;
            end%for
            
		end%fcn
        
        function obj = select(obj, idxs)
            
            tmp = unique([idxs(:), idxs(:)+1]);
            obj = SplinePath(obj.Breaks(tmp), obj.Coefs(:,idxs,:));
            
        end%fcn
		
        function obj = shift(obj, P)
			
			% Handle input arguments
			narginchk(2, 2);
			
			if numel(P) ~= 2 || ~isnumeric(P)
				error(['Method SHIFTBY requires a numeric input',...
					' argument with two elements.']);
			end%if
			
		end%fcn
        
        function [P0,P1] = termPoints(obj)
            
            [tau0,tau1] = getDomain(obj);
            [x,y] = eval(obj, [tau0,tau1]);
            P0 = [x(1); y(1)];
            P1 = [x(2); y(2)];
            
        end%fcn
        
		function s = toStruct(obj)
			s = struct('breaks',obj.Breaks, 'coefs',obj.Coefs);
		end%fcn
		
	end%methods
	
	
    methods (Access = private)
        
    end%methods
	
	
	methods (Static)
        
        function obj = ll2Path(lat, lon)%#codegen
		end%fcn
		
		function obj = pp2Path(pp)
            [breaks,coefs,nbrSeg,polyOrd,dim] = unmkpp(pp);
			obj = SplinePath(breaks, reshape(coefs, [2,[],polyOrd]));
		end%fcn
		
		function obj = xy2Path(x, y)
		end%fcn
		
		function obj = fromStruct(s)
			obj = SplinePath(s.breaks, s.coefs);
		end%fcn
		
		function c = getBusDef()
			BusName = 'SBus_SplinePath';
			HeaderFile = '';
			Description = '';
			BusElements = {...
				{'breaks',      100, 'double',	-1, 'real', 'Sample', 'Variable', [], [], '', ''},...
				{'coefs', [2,100,4], 'double',	-1, 'real', 'Sample', 'Variable', [], [], '', ''},...
				};
			c = {{BusName,HeaderFile,Description,BusElements}};
		end%fcn
		
	end%methods
	
end%class
