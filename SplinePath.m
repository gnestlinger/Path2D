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

    properties (SetAccess = private)
        Breaks = 0
        Coefs = zeros(2,0,4) % path dimension by # of segments by # order
    end
    
    properties (Access = private)
        % ArcLengths - Cumulative length of spline segments
        ArcLengths = 0;
    end
	
	
	methods
		
        function obj = SplinePath(breaks, coefs, s, isCircuit)
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
            
            assert(size(coefs,1) == 2, 'Spline must be 2D valued!');
            assert(isequal(numel(breaks)-1, size(coefs,2)), ...
                'Number of breaks and spline segments does not match!');
            obj.Breaks = breaks;
            obj.Coefs = coefs;
            
            if (nargin < 3) || isempty(s)
                pp = mkpp(breaks, coefs, 2);
                N = obj.numel();
                s = coder.nullcopy(zeros(N+1,1));
                s(1) = 0;
                for i = 1:N
                    tau = linspace(breaks(i), breaks(i+1), 1000);
                    dxy = diff(ppval(pp, tau), 1, 2);
                    s(i+1) = s(i) + sum(hypot(dxy(1,:), dxy(2,:)));
                end
            end
            assert(numel(s) == numel(breaks));
            obj.ArcLengths = s - s(1); % Make sure first element is zero!
            
            
            if nargin < 4
                obj = obj.setIsCircuit(1e-5);
            else
                obj.IsCircuit = isCircuit;
            end%if
            
        end%Constructor
        
        function obj = append(obj, obj2)
            
            breaks = obj.Breaks;
            coefs = obj.Coefs;
            arcLen = obj.ArcLengths;
            
            obj.Breaks = [breaks, breaks(end) + obj2.Breaks(2:end)];
            
            [~,~,k] = size(coefs);
            [~,n2,k2] = size(obj2.Coefs);
            assert(k2 <= k, 'Degree of path to append must not exceed degree of initial path!')
            obj.Coefs = cat(2, coefs, cat(3, zeros(2,n2,k-k2), obj2.Coefs));
            
            [~,P1] = obj.termPoints();
            P0 = obj2.termPoints();
            ds = sqrt( sum((P0 - P1).^2) );
            obj.ArcLengths = [arcLen; obj2.ArcLengths(2:end) + arcLen(end) + ds];
            
        end%fcn
        
        function [sd,Q,idx,tau] = cart2frenet(obj, xy, doPlot)
            
        end%fcn
        
        function [tauL,tauU] = domain(obj)
            if isempty(obj)
                tauL = NaN;
                tauU = NaN;
            else
                tauL = obj.Breaks(1);
                tauU = obj.Breaks(end);
            end
        end%fcn
        
        function [x,y,tau,head,curv] = eval(obj, tau)
            
            if nargin < 2
                breaks = obj.Breaks;
                
                % M samples per spline segment + 1 sample for end point
                M = 100;
                N = obj.numel();
                tau = coder.nullcopy(zeros(N*M + 1, 1));
                for i = 1:N
                    % To avoid repeated break entries, overwrite end break
                    % of preceeding segment with initial (and equal) break
                    % of current segment
                    ii = (i-1)*M + 1;
                    jj = ii + M;
                    tau(ii:jj) = linspace(breaks(i), breaks(i+1), M+1);
                end
                tau(end) = breaks(end);
            else
                tau = tau(:);
            end%if
            
            if isempty(obj)
                N = numel(tau);
                tau(:) = NaN;
                x = NaN(N, 1);
                y = NaN(N, 1);
                head = NaN(N, 1);
                curv = NaN(N, 1);
                return
            end
            
            pp = obj.mkpp();
            
            % Avoid PPVAL returning 3D arrays for empty evaluation sites by
            % applying (:)
            xy = ppval(pp, tau(:))';
            x = xy(:,1);
            y = xy(:,2);
            
            if nargout > 3
                ppD1 = ppdiff(pp);
                xyD1 = ppval(ppD1, tau);
                head = cx2Heading(xyD1(1,:), xyD1(2,:))';
                
                if nargout > 4
                    ppD2 = ppdiff(ppD1);
                    xyD2 = ppval(ppD2, tau);
                    curv = cx2Curvature(xyD1(1,:), xyD1(2,:), xyD2(1,:), xyD2(2,:))';
                end
            end%if
            
        end%fcn
        
        function [xy,Q,idx] = frenet2cart(obj, sd)
            
        end%fcn
		
		function s = getPathLengths(obj, idx0, idx1)
            s = obj.ArcLengths;
		end%fcn
        
        function [xy,tau,errFlag] = intersectLine(obj, O, psi)
        end%fcn
        
        function [xy,tau,errFlag] = intersectCircle(obj, C, r)
        end%fcn
        
        function flag = isempty(obj)
            flag = (obj.numel() < 1);
        end%fcn
        
		function s = length(obj, tau)
            
            if nargin < 1
                s = obj.ArcLengths(end);
            else
                idx = find(tau > obj.Breaks, 1, 'last');
                taus = linspace(obj.Breaks(idx), tau, 100);
                xy = diff(ppval(obj.mkpp(), taus), 1, 2);
                s = obj.ArcLengths(idx) + hypot(xy(1,:), xy(2,:));
            end%if
            
		end%fcn
		
        function pp = mkpp(obj)
        %MKPP   Create piecewise polynomial struct.
        %   PP = MKPP(OBJ) creates piecewise polynomial structure PP from
        %   path OBJ.
        %
        %   See also PPVAL, MKPP, UNMKPP.
        
            assert(~isempty(obj), 'Path must be non-empty!')
            pp = mkpp(obj.Breaks, obj.Coefs, 2);
        end%fcn
        
        function [Q,idx,tau] = pointProjection(obj, poi, doPlot)
            
            pp = obj.mkpp();
            ppd = ppdiff(pp);
            fh = @(tau) abs(ppIncAngle(pp, ppd, poi, tau) - pi/2);
            [tau0,tau1] = obj.domain();
            [tau,fval,exitflag,output] = fminbnd(fh, tau0, tau1);
            
            [x,y] = obj.eval(tau);
            Q = [x y];
            
            idx = find(tau > obj.Breaks, 1, 'last');
            
            if (nargin > 2) && doPlot
                plot(obj, 'Marker','.')
                hold on
                plot(poi(1), poi(2), 'rx')
                plot(Q(1), Q(2), 'ro')
                hold off
            end%if
            
        end%fcn
        
        function [Q,idx,tau] = pointProjectionAll(obj, poi, doPlot)
            
            pp = mkpp(obj);
            ppd = ppdiff(pp);
            fh = @(tau) abs(ppIncAngle(pp, ppd, poi, tau) - pi/2);
            
            N = numel(obj);
            taus = coder.nullcopy(zeros(N,1));
            fvals = coder.nullcopy(zeros(N,1));
            flags = coder.nullcopy(zeros(N,1));
            breaks = obj.Breaks;
            tau0 = breaks(1);
            for i = 1:N
                tau1 = breaks(i+1);
                [taui,fval,exitflag,output] = fminbnd(fh, tau0, tau1);
                taus(i) = taui;
                fvals(i) = fval;
                flags(i) = exitflag;
                tau0 = tau1;
            end
            
            isValid = (fvals < 1);
            
            % Set return arguments
            [x,y] = obj.eval(taus(isValid));
            Q = [x y];
            idx = find(isValid);
            tau = taus(isValid);
            
            if (nargin > 2) && doPlot
                plot(obj, 'Marker','.')
                hold on
                plot(poi(1), poi(2), 'rx')
                plot(Q(:,1), Q(:,2), 'ro')
                hold off
            end%if
            
        end%fcn
        
        function d = pathDistance(obj, P, tau)
            
            pp = obj.mkpp();
            xy = ppval(pp, tau);
            d = hypot(xy(1,:) - P(1), xy(2,:) - P(2));
            
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
                idxF = obj.numel();
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
                tau0 = obj.domain();
                [~,~,~,phi] = obj.eval(tau0);
                phi = -phi;
            end%if
            
            R = rotmat2D(phi);
            for i = 1:builtin('numel', obj)
                obji = obj(i);
                coefs = obji.Coefs;
                for ii = 1:obj.numel() % Loop over segments
                   coefs(:,ii,:) = R*squeeze(coefs(:,ii,:));
                end
                obj(i).Coefs = coefs;
            end%for
            
        end%fcn
        
        function obj = select(obj, idxs)
            
            idxsc = idxs(:);
            tmp = unique([idxsc, idxsc+1]);
            s = obj.ArcLengths(tmp);
            obj = SplinePath(obj.Breaks(tmp), obj.Coefs(:,idxs,:), s - s(1));
            
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
            
            [tau0,tau1] = obj.domain();
            [x,y] = obj.eval([tau0,tau1]);
            P0 = [x(1); y(1)];
            P1 = [x(2); y(2)];
            
        end%fcn
        
		function s = toStruct(obj)
			s = struct(...
                'breaks',obj.Breaks, ...
                'coefs',obj.Coefs, ...
                'lengths', obj.ArcLengths);
		end%fcn
		
	end%methods
	
	
    methods (Access = private)
        
    end%methods
	
	
	methods (Static)
        
        function obj = ll2Path(lat, lon) %#ok<STOUT,INUSD>
            error('Not implemented!')
		end%fcn
		
		function obj = pp2Path(pp)
            [breaks,coefs,nbrSeg,polyOrd,dim] = unmkpp(pp);
            assert(dim == 2)
			obj = SplinePath(breaks, reshape(coefs, 2, nbrSeg, polyOrd));
		end%fcn
		
		function obj = xy2Path(x, y) %#ok<STOUT,INUSD>
            error('Not implemented!')
		end%fcn
		
		function obj = fromStruct(s)
			obj = SplinePath(s.breaks, s.coefs, s.lengths);
		end%fcn
		
		function c = getBusDef()
			BusName = 'SBus_SplinePath';
			HeaderFile = '';
			Description = '';
			BusElements = {...
				{'breaks',      101, 'double',	-1, 'real', 'Sample', 'Variable', [], [], '', ''},...
				{'coefs', [2,100,4], 'double',	-1, 'real', 'Sample', 'Variable', [], [], '', ''},...
                {'lengths',     101, 'double',	-1, 'real', 'Sample', 'Variable', [], [], '', ''},...
				};
			c = {{BusName,HeaderFile,Description,BusElements}};
		end%fcn
		
	end%methods
	
end%class
