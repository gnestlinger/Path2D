classdef (InferiorClasses = {?matlab.graphics.axis.Axes}) SplinePath < Path2D
%SPLINEPATH     Spline path.
%   Path representation using splines.
% 
%   SPLINEPATH properties:
%   Breaks - Cartesian x-coordinate.
%   Coefs - Cartesian y-coordinate.
% 
%   SPLINEPATH methods:
%   SplinePath - Constructor.
%   mkpp - Create piecewise polynomial structure.
%   See superclasses.
% 
%   SPLINEPATH static methods:
%   See superclasses.
% 
%   See also PATH2D.

%#ok<*EVLC> % Using eval and evalc with two arguments is not recommended ..

    properties (SetAccess = private)
        Breaks = 0
        Coefs = zeros(2,0,4) % path dimension by # of segments by # order
    end
    
	
	
	methods
		
		function obj = SplinePath(breaks, coefs, isCircuit)
		%SPLINEPATH     Create spline path object.
		%	OBJ = SPLINEPATH(BREAKS,COEFS) creates a spline path OBJ with
		%	breaks BREAKS of size 1-by-(N+1) and coefficients COEFS of size
		%	2-by-N-by-K, where N is the number of spline segments and K-1
		%	the polynomial degree.
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
                [P0,P1] = termPoints(obj);
                obj.IsCircuit = (norm(P1 - P0) < 1e-5);
			else
				obj.IsCircuit = isCircuit;
			end%if
			
		end%Constructor
		
        function obj = append(obj, obj2)
            
            breaks = obj.Breaks;
            obj.Breaks = [breaks, breaks(end) + obj2.Breaks(2:end)];
            
            coefs = obj.Coefs; % TODO: handle non-matching degree
            [~,~,k] = size(coefs);
            [~,n2,k2] = size(obj2.Coefs);
            
            assert(k2 <= k, 'Degree of path to append must not exceed degree of initial path!')
            obj.Coefs = cat(2, obj.Coefs, cat(3, zeros(2,n2,k-k2), obj2.Coefs));
            
        end%fcn
		
		function [sd,Q,idx,tau] = cart2frenet(obj, poi, doPlot)
			
		end%fcn
        
        function [tauL,tauU] = domain(obj)
            tauL = obj.Breaks(1);
            tauU = obj.Breaks(end);
        end%fcn
        
        function [x,y,head,curv] = eval(obj, tau)
            
            if nargin < 2
                % M samples per spline segment
                M = 100;
                N = numel(obj);
                
                breaks = obj.Breaks;
                tau = coder.nullcopy(zeros(N*M , 1));
                tau(1:M) = linspace(breaks(1), breaks(2), 100);
                for i = 2:N
                    % To avoid repeated break entries, overwrite end break
                    % of preceeding segment with initial (and equal) break
                    % of current segment
                    ii = (i-1)*M;
                    jj = ii + M;
                    tau(ii:jj) = linspace(breaks(i), breaks(i+1), M+1);
                end
            end
            
            pp = mkpp(obj);
            xy = ppval(pp, tau)';
            x = xy(:,1);
            y = xy(:,2);
            
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
		
		function s = getPathLengths(obj, idx0, idx1)
            
		end%fcn
        
        function [xy,tau,errFlag] = intersectLine(obj, O, psi)
        end%fcn
        
        function [xy,tau,errFlag] = intersectCircle(obj, C, r)
        end%fcn
        
        function flag = isempty(obj)
            flag = (numel(obj) < 1);
        end%fcn
        
		function s = length(obj)
            
		end%fcn
		
        function pp = mkpp(obj)
        %MKPP   Create piecewise polynomial struct.
        %   PP = MKPP(OBJ) creates piecewise polynomial structure PP from
        %   path OBJ.
        %
        %   See also PPVAL, MKPP, UNMKPP.
            pp = mkpp(obj.Breaks, obj.Coefs, 2);
        end%fcn
        
        function [Q,idx,tau] = pointProjection(obj, poi, doPlot)
            
            pp = mkpp(obj);
            ppd = ppdiff(pp);
            fh = @(tau) abs(pointAngle(obj, pp, ppd, poi, tau) - pi/2);
            [tau0,tau1] = domain(obj);
            [tau,fval,exitflag,output] = fminbnd(fh, tau0, tau1);
            
            [x,y] = eval(obj, tau);
            Q = [x;y];
            
            idx = find(tau > obj.Breaks, 1, 'last');
            
            if (nargin > 2) && doPlot
                plot(obj, 'Marker','.')
                hold on
                plot(poi(1), poi(2), 'rx')
                plot(Q(1), Q(2), 'ro')
                hold off
            end%if
            
		end%fcn
        
        function d = pathDistance(obj, P, tau)
            
            pp = mkpp(obj);
            xy = ppval(pp, tau);
            d = hypot(xy(1,:) - P(1), xy(2,:) - P(2));
            
        end%fcn
        
        function phi = pointAngle(obj, pp, ppd, P, tau)
            
            xy = ppval(pp, tau);
            dxy = ppval(ppd, tau);
            w = P(:) - xy;
            phi = atan2(w(2), w(1)) - atan2(dxy(2), dxy(1));
            
        end%fcn
		
		function n = numel(obj)
			n = size(obj.Coefs, 2); % Return number of spline segments
		end%fcn
        
        function obj = restrict(obj, tau0, tauF)
            
            assert(tau0 < tauF)
            
            % Find the lower/upper index so that restricted domain is
            % covered
            [tauL,tauU] = obj.domain();
            taus = obj.Breaks;
            if isempty(tau0) || (tau0 < tauL)
                idx0 = 1;
            else
                idx0 = find(tau0 >= taus, 1, 'last');
            end
            if isempty(tauF) || (tauF > tauU)
                idxF = numel(obj);
            else
                idxF = find(tauF >= taus, 1, 'last');
            end%if
            
            obj = obj.select(idx0:idxF);
            obj.Breaks(1) = tau0;
            obj.Breaks(end) = tauF;
            
        end%fcn
        
        function obj = reverse(obj)
			
		end%fcn
        
        function obj = rotate(obj, phi)
            
            narginchk(1, 2);
            
            if nargin < 2
                tau0 = domain(obj);
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
            
            [tau0,tau1] = domain(obj);
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
            assert(dim == 2)
			obj = SplinePath(breaks, reshape(coefs, 2, nbrSeg, polyOrd));
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
