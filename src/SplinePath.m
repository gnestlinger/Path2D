classdef (InferiorClasses = {?matlab.graphics.axis.Axes}) SplinePath < Path2D
%SPLINEPATH     Spline path.
%   Path representation using polynomial splines.
% 
%   SPLINEPATH properties:
%   Breaks - Break points.
%   Coefs - Polynomial coefficients.
% 
%   SPLINEPATH methods:
%   SplinePath - Constructor.
%   derivative - Derivative of path.
%   mkpp - Create piecewise polynomial structure.
%   See superclasses.
% 
%   SPLINEPATH static methods:
%   bezier2Path - Spline path from Bezier control points.
%   See superclasses.
% 
%   See also PATH2D, MKPP.

    properties (SetAccess = private)
        % Breaks - Break points
        %   Specified as a vector of length N+1.
        Breaks = 0
        
        % Coefs - Polynomial coefficients
        %   Specified as an array of size 2-by-N-by-Np, i.e. x/y-by-number
        %   of pieces-by-polynomial coefficients in descending order. E.g.
        %   coefs(1,i,:) contains the local x-coefficients for the interval
        %   [breaks(i) breaks(i+1)].
        Coefs = zeros(2,0,4)
    end
    
    
    
    methods
        function obj = SplinePath(breaks, coefs, s, isCircuit)
        %SPLINEPATH     Create spline path object.
        %   OBJ = SPLINEPATH(BREAKS,COEFS) creates a spline path OBJ with
        %   breaks BREAKS of size 1-by-(N+1) and coefficients COEFS of size
        %   2-by-N-by-K, where N is the number of spline segments and K-1
        %   the polynomial degree.
        %   
        %   OBJ = SPLINEPATH(___,S) sets the cumulative lengths of the
        %   spline segments.
        %
        %   OBJ = SPLINEPATH(___,S,ISCIRCUIT) set to true if the path is a
        %   circuit.
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
                s = obj.calcArcLength(breaks, coefs);
            end
            assert(numel(s) == size(coefs,2));
            obj.ArcLengths = s(:);
            
            if nargin < 4
                obj = obj.setIsCircuit(1e-5);
            else
                obj.IsCircuit = isCircuit;
            end%if
            
        end%Constructor
        
        function obj = append(obj, obj2)
            
            if obj.isempty()
                ds = 0;
            else
                [~,P1] = obj.termPoints();
                dP = obj2.termPoints() - P1;
                ds = hypot(dP(1), dP(2)) + obj.length();
            end
            
            breaks = obj.Breaks;
            coefs = obj.Coefs;
            breaks2 = obj2.Breaks;
            coefs2 = obj2.Coefs;
            
            obj.Breaks = [breaks, breaks(end) + breaks2(2:end) - breaks2(1)];
            
            [~,n1,k1] = size(coefs);
            [~,n2,k2] = size(coefs2);
            obj.Coefs = cat(2, ...
                cat(3, zeros(2,n1,k2-k1), coefs), ...
                cat(3, zeros(2,n2,k1-k2), coefs2));
            
            obj.ArcLengths = [obj.ArcLengths; obj2.ArcLengths + ds];
            
        end%fcn
        
        function [sd,Q,idx,tau,dphi] = cart2frenet(obj, xy, phiMax, doPlot)
            
            if nargin < 4
                doPlot = false;
                if nargin < 3
                    phiMax = [];
                end
            end
            
            [Q,idx,tau,dphi] = obj.pointProjection(xy, phiMax, doPlot);
            if isempty(Q) % Find the break point closest to point of interest
                % SInce we index into breaks to obtain tau, breaks must be
                % a column vector (such as tau from pointProjection()) to
                % have no conflicting array sizes in code-gen.
                breaks = obj.Breaks(:);
                [x,y] = obj.eval(breaks);
                [~,idx] = min(hypot(x - xy(1), y - xy(2)));
                Q = [x(idx) y(idx)];
                tau = breaks(idx);
                if idx(1) == numel(breaks) - 1
                    % Avoid out of range indexing
                    idx = idx(1) - 1; 
                end
            end%if
            
            % Get the orientation vector related with Q to calculate the
            % sign of distance from point of interest to path
            ppd = ppdiff(obj.mkpp());
            u = ppval(ppd, tau)';
            
            % Get sign via z-component of cross product U x (Q-XY)
            qp = bsxfun(@minus, Q, xy(:)');
            signD = sign(crossz(u, qp));
            
            sd = [obj.idxTau2s(idx, tau), ...
                signD.*hypot(qp(:,1), qp(:,2))];
            
            if isempty(dphi)
                ux = u(:,1);
                uy = u(:,2);
                dx = qp(:,1);
                dy = qp(:,2);
                dphi = abs(pi/2 - abs(atan2(ux.*dy - uy.*dx, ux.*dx + uy.*dy)));
            end
            
        end%fcn
        
        function obj = clear(obj)
            obj.Breaks = 0;
            obj.Coefs(:,1:end,:) = [];
            obj.ArcLengths = zeros(0,1);
            obj.IsCircuit = false;
        end%fcn
        
        function obj = derivative(obj, n)
        %DERIVATIVE     Derivative of path.
        %   OBJ = DERIVATIVE(OBJ) returns the derivative of path OBJ.
        %
        %   OBJ = DERIVATIVE(OBJ,N) returns the N-th derivative of path
        %   OBJ.
        %
            
            if nargin < 2
                n = 1;
            end
            
            if (n < 1) || obj.isempty()
                return
            end
            
            c = obj.Coefs;
            [~,Ns,Np] = size(c); % Number of pieces and polynomial order
            
            if n < 1
                return;
            elseif ~(n < Np)
                obj.Coefs = zeros(2,Ns,1);
            else
                % Remove polynomial coefficients due to derivative
                cd = c(:,:,1:end-n);
                
                % Initialize powers for first derivative; additional
                % derivatives are added in the for-loop
                powers = Np-1:-1:n;
                for i = 2:n
                    powers = powers .* (Np-i:-1:n-i+1);
                end%for
                cd = bsxfun(@times, cd, reshape(powers, [1 1 Np-n]));
                obj.Coefs = cd;
            end%if
            
            obj = obj.setIsCircuit(1e-5);
            
            % Update the path length after setting derivative coefficients
            s = obj.calcArcLength();
            obj.ArcLengths = s;
            
        end%fcn
        
        function [tauL,tauU] = domain(obj)
            if obj.isempty()
                tauL = NaN;
                tauU = NaN;
            else
                tauL = obj.Breaks(1);
                tauU = obj.Breaks(end);
            end
        end%fcn
        
        function [x,y,tau,head,curv,curvDs] = eval(obj, tau, extrapolate)
            
            if nargin < 3
                extrapolate = false;
            end
            
            if nargin < 2
                if obj.isempty()
                    tau = zeros(0,1);
                else
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
                end
            else
                % Avoid PPVAL returning 3D arrays for empty evaluation
                % sites by applying (:)
                tau = tau(:);
            end%if
            
            if obj.isempty()
                N = numel(tau);
                tau(:) = NaN;
                x = NaN(N, 1);
                y = NaN(N, 1);
                head = NaN(N, 1);
                curv = NaN(N, 1);
                curvDs = NaN(N, 1);
                return
            end
            
            % Set tau to NaN outside spline domain so that corresponding
            % return values are NaN too.
            if ~extrapolate
                tau((tau < obj.Breaks(1)) | (tau > obj.Breaks(end))) = NaN;
            end
            
            % Make use of PPVAL for spline evaluation
            pp = obj.mkpp();
            xy = ppval(pp, tau)';
            x = xy(:,1);
            y = xy(:,2);
            
            if nargout > 3
                ppD1 = ppdiff(pp);
                xyD1 = ppval(ppD1, tau)';
                xD1 = xyD1(:,1);
                yD1 = xyD1(:,2);
                head = cx2Heading(xD1, yD1);
                % Evaluation of a piecewise constant with only one piece at
                % NaN, ppval returns the constant. Therefore, set to NaN
                % manually.
                head(isnan(tau)) = NaN; 
                
                if nargout > 4
                    ppD2 = ppdiff(ppD1);
                    xyD2 = ppval(ppD2, tau)';
                    [curv,hyp] = cx2Curvature(xD1, yD1, xyD2(:,1), xyD2(:,2));
                    curv(isnan(tau)) = NaN;
                    
                    if nargout > 5
                        xyD3 = ppval(ppdiff(ppD2), tau)';
                        hyp3  = hyp.*hyp.*hyp;
                        curvDs = (xD1.*xyD3(:,2) - xyD3(:,1).*yD1)./(hyp3.*hyp) ...
                            + 3*(xD1.*xyD2(:,1) + yD1.*xyD2(:,2)).*curv./hyp3;
                        % Nan values are propagated from curvature result
%                         curvDs(isnan(tau)) = NaN; 
                    end
                end
            end%if
            
        end%fcn
        
        function tau = findMaxCurvature(obj, ~)
        %FINDMAXCURVATURE   Find all local extrema of path curvature.
        %   TAU = FINDMAXCURVATURE(OBJ) returns the path parameterss TAU
        %   for which the path's curvature is an extremum.
        %
        %   This implementation does not rely on polynomial root-finding,
        %   instead the solutions are found by considering finite
        %   differences.
        
        % d(Curvature)/dtau = 1/(...)*(...
        %   y^(3)*x'^3 + y'^2 (3x''*y'' - x^(3)*y') + x'*y'*(3x''^2 - 3y''^2 + y^(3)*y') + x'^2*(-x^(3)y' - 3x''y'')
        %   )
            
            breaks = obj.Breaks;
            Ns = obj.numel();
            coder.varsize('tau', [inf 1], [true false])
            tau = zeros(0,1);
            b0 = breaks(1);
            for i = 1:Ns % Loop over spline segments
                b1 = breaks(i+1);
                [~,~,taui,~,curv] = obj.eval(linspace(b0, b1, 1e3));
                dc = diff(curv);
                
                % This condition misses extrema at the terminal points of
                % path segments
                isExtremum = [false; diff(sign(dc))~=0; false];
                tau = [tau; sort(taui(isExtremum), 'ascend')]; %#ok<AGROW>
                
                b0 = b1;
            end%for
            
        end%fcn
        
        function tau = findZeroCurvature(obj, ~)
        %FINDZEROCURVATURE  Find path parameter w.r.t to zero curvature.
        %   TAU = FINDZEROCURVATURE(OBJ) returns the path parameters TAU
        %   for which the path's curvature evaluates to zero (apart from
        %   rounding errors resulting from polynomial root finding).
        %
            
            breaks = obj.Breaks;
            
            objD1 = obj.derivative();
            objD2 = derivative(objD1);
            coefsD1x = permute(objD1.Coefs(1,:,:), [2 3 1]);
            coefsD1y = permute(objD1.Coefs(2,:,:), [2 3 1]);
            coefsD2x = permute(objD2.Coefs(1,:,:), [2 3 1]);
            coefsD2y = permute(objD2.Coefs(2,:,:), [2 3 1]);
            
            Ns = obj.numel();
            N1 = size(coefsD1x, 2);
            N2 = size(coefsD2x, 2);
            coder.varsize('tau', Ns*(N1+N2-1))
            tau = [];
            b0 = breaks(1);
            for i = 1:Ns % Loop over spline segments
                b1 = breaks(i+1);
                
                % Curvature = (x'*y'' - x''*y')/(x'^2 + y'^2)^1.5
                dx = coefsD1x(i,:);
                dy = coefsD1y(i,:);
                ddx = coefsD2x(i,:);
                ddy = coefsD2y(i,:);
                numi = conv2(dx, ddy) - conv2(ddx, dy);
                taui = getRealRootsWithinBounds(numi, @lt, b1-b0);
                tau = [tau; sort(taui, 'ascend') + b0]; %#ok<AGROW>
                
                b0 = b1;
            end%for
            
        end%fcn
        
        function [xy,Q,idx,tau] = frenet2cart(obj, sd, doPlot)
            
            % Get the indexes referring to the path segments according to
            % the frenet coordinates s-values
            sEval = sd(:,1);
            [tau,idx] = obj.s2tau(sEval);
            
            [x,y] = obj.eval(tau, true);
            n = ppval(ppdiff(obj.mkpp()), tau)';
            Q = [x y];
            xy = Q + bsxfun(@times, [-n(:,2) n(:,1)], sd(:,2)./sqrt(sum(n.^2, 2)));
            
            if (nargin > 2) && doPlot
                obj.plot('DisplayName','SplinePath');
                hold on
                [xb,yb] = obj.eval(obj.Breaks);
                plot(xb, yb, 'b.', 'MarkerSize',10, 'DisplayName','Breaks');
                plot(xy(:,1), xy(:,2), 'o', 'DisplayName','xy');
                plot(Q(:,1), Q(:,2), 'kx', 'DisplayName','Q');
                hold off
                legend('-DynamicLegend')
            end%if
            
        end%fcn
        
        function [xy,tau,errFlag] = intersectCircle(obj, C, r, doPlot)
            
            obj = obj.shift(-C);
            breaks = obj.Breaks;
            coefs = obj.Coefs;
            
            % There are at most 2*(Nc-1) real roots (i.e. solutions) per
            % piece
            [~,Ns,Nc] = size(coefs);
            coder.varsize('tau', 2*Ns*(Nc-1));
            tau = zeros(0,1);
            
            b0 = breaks(1);
            for i = 1:Ns-1
                coef = permute(coefs(:,i,:), [1 3 2]);
                equi = conv2(coef(1,:), coef(1,:)) + conv2(coef(2,:), coef(2,:));
                equi(end) = equi(end) - r^2;
                
                b1 = breaks(i+1);
                ri = getRealRootsWithinBounds(equi, @lt, b1-b0);
                tau = [tau; ri + b0]; %#ok<AGROW>
                b0 = b1;
            end%for
            
            % The last piece is defined over the closed interval [b0,b1]
            coef = permute(coefs(:,end,:), [1 3 2]);
            equi = conv2(coef(1,:), coef(1,:)) + conv2(coef(2,:), coef(2,:));
            equi(end) = equi(end) - r^2;
            ri = getRealRootsWithinBounds(equi, @le, breaks(end)-b0);
            tau = [tau; ri + b0];
            
            tau = sort(tau, 'ascend');
            errFlag = isempty(tau);
            [x,y] = obj.eval(tau);
            xy = [x+C(1) y+C(2)];
            
            if (nargin > 3) && doPlot
                [~,ax] = plot(obj.shift(C));
                hold on
                
                phi = 0:pi/1000:2*pi;
                plot(ax, r*cos(phi)+C(1), r*sin(phi)+C(2), 'DisplayName','Circle')
                
                plot(ax, xy(:,1), xy(:,2), 'kx', 'DisplayName','Intersections')
                hold off
            end%if
            
        end%fcn

        function [xy,tau,errFlag] = intersectLine(obj, O, psi, doPlot)
            
            obj1 = obj.shift(-O).rotate(-psi);
            
            breaks = obj1.Breaks;
            coefsY = permute(obj1.Coefs(2,:,:), [2 3 1]);
            
            % There are at most Nc-1 real roots (i.e. solutions) per piece
            [Ns,Nc] = size(coefsY);
            coder.varsize('tau', Ns*(Nc-1));
            tau = zeros(0,1);
            
            % The first pieces except the last are defined over half open
            % intervals [b0,b1)
            b0 = breaks(1);
            for i = 1:Ns-1
                b1 = breaks(i+1);
                ri = getRealRootsWithinBounds(coefsY(i,:), @lt, b1-b0);
                tau = [tau; ri + b0]; %#ok<AGROW>
                b0 = b1;
            end%for
            
            % The last piece is defined over the closed interval [b0,b1]
            ri = getRealRootsWithinBounds(coefsY(end,:), @le, breaks(end)-b0);
            tau = [tau; ri + b0];
            
            tau = sort(tau, 'ascend');
            errFlag = isempty(tau);
            [x,y] = obj.eval(tau);
            xy = [x,y];
            
            if (nargin > 3) && doPlot
                [~,ax] = plot(obj);
                hold on
                
                [r1,r2] = scaleTangentToAxis(xlim, ylim, O, psi);
                Pstart  = [O(1) + r2*cos(psi); O(2) + r2*sin(psi)];
                Pstop   = [O(1) + r1*cos(psi); O(2) + r1*sin(psi)];
                h = plot(gca, [Pstart(1) Pstop(1)], [Pstart(2) Pstop(2)], ...
                    'DisplayName','Line');
                plot(O(1), O(2), 'o', 'Color',get(h,'Color'), 'DisplayName','O')
                
                plot(ax, xy(:,1), xy(:,2), 'kx', 'DisplayName','Intersections')
                hold off
            end%if
            
        end%fcn
        
        function pp = mkpp(obj)
        %MKPP   Create piecewise polynomial struct.
        %   PP = MKPP(OBJ) creates piecewise polynomial structure PP from
        %   path OBJ.
        %
        %   See also PPVAL, MKPP, UNMKPP.
        
            assert(~obj.isempty(), 'Path must be non-empty!')
            pp = mkpp(obj.Breaks, obj.Coefs, 2);
        end%fcn
        
        function [Q,idx,tau,fval] = pointProjection(obj, poi, ~, doPlot)
        %POINTPROJECTION    Finds the orthogonal projection on the path.
        %   POINTPROJECTION finds the orthogonal projection of a point POI
        %   with respect the spline path by calculating the roots of a
        %   polynomial of order 2N-2, where N is the order of the spline. 
        %
        %   See also PATH2D/POINTPROJECTION, ROOTS.
            
            Px = poi(1);
            Py = poi(2);
            breaks = obj.Breaks;
            coefsX = permute(obj.Coefs(1,:,:), [2 3 1]);
            coefsY = permute(obj.Coefs(2,:,:), [2 3 1]);
            [Ns,Nc] = size(coefsX);
            
            % There are at most 2Nc-2 solutions per piece
            coder.varsize('tau', 'idx', 'pv', Ns*(2*Nc-2));
            tau = zeros(0,1);
            idx = zeros(0,1);
            pv = zeros(0,1);
            b0 = breaks(1);
            for i = 1:Ns-1
                b1 = breaks(i+1);
                
                poly = calcPointProjPolynomial(Px, Py, ...
                    coefsX(i,:), coefsY(i,:), Nc);
                ri = getRealRootsWithinBounds(poly, @lt, b1-b0);
                riSorted = sort(ri, 'ascend');
                tau = [tau; riSorted + b0]; %#ok<AGROW>
                idx = [idx; repmat(i, size(ri))]; %#ok<AGROW
                pv = [pv; polyval(poly, riSorted)]; %#ok<AGROW
                
                b0 = b1;
            end%for
            
            % The last piece is defined over the closed interval [b0,b1]
            b1 = breaks(end);
            poly = calcPointProjPolynomial(Px, Py, ...
                coefsX(end,:), coefsY(end,:), Nc);
            ri = getRealRootsWithinBounds(poly, @le, b1-b0);
            riSorted = sort(ri, 'ascend');
            tau = [tau; riSorted + b0];
            idx = [idx; repmat(Ns, size(ri))];
            pv = [pv; polyval(poly, riSorted)]; 
            
            % Find and remove repeated solutions based on a magic number
            idxRepeated = find(diff(tau) < 1e-5);
            for i = numel(idxRepeated):-1:1
                idxi = idxRepeated(i);
                if pv(idxi) < pv(idxi+1)
                    tau(idxi+1) = [];
                    idx(idxi+1) = [];
                else
                    tau(idxi) = [];
                    idx(idxi) = [];
                end
            end%for
            
            Q = ppval(obj.mkpp(), tau)';
            fval = zeros(size(tau));
            
            if (nargin > 3) && doPlot
                [~,ax] = obj.plot();
                hold on
                [x0,y0] = obj.eval(obj.Breaks);
                plot(x0, y0, 'g.', 'MarkerSize',12, 'DisplayName','Breaks');
                plot(poi(1), poi(2), 'ro', 'DisplayName','PoI')
                plot(Q(:,1), Q(:,2), 'kx', 'DisplayName','Q')
                legend('-DynamicLegend', 'Location','best'); % For ~2014b
                plot(...
                    [Q(:,1)'; repmat([poi(1) NaN], size(Q,1),1)'],...
                    [Q(:,2)'; repmat([poi(2) NaN], size(Q,1),1)'], 'k:'); 
                hold off
                a = legend(ax); % For ~2019b
                a.String(5:end) = '';
            end%if
            
        end%fcn
        
        function n = numel(obj)
            n = size(obj.Coefs, 2); % Return number of spline segments
        end%fcn
        
        function obj = restrict(obj, tau0, tauF)
            
            assert(tau0 < tauF)
            
            % Find the lower/upper index so that restricted domain is
            % covered
            [tauL,tauU] = obj.domain();
            breaks = obj.Breaks;
            if isempty(tau0) || (tau0 < tauL)
                idx0 = 1;
            else
                idx0 = find(tau0 >= breaks, 1, 'last');
            end
            if isempty(tauF) || (tauF > tauU)
                idxF = obj.numel();
            else
                idxF = find(tauF > breaks, 1, 'last');
            end%if
            
            obj = obj.select(idx0:idxF);
            obj.Breaks(1) = tau0;
            obj.Breaks(end) = tauF;
            
        end%fcn
        
        function obj = reverse(obj)
            error('Not implemented!')
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
                coefs = obj(i).Coefs;
                for ii = 1:obj.numel() % Loop over segments
                   coefs(:,ii,:) = R*permute(coefs(:,ii,:), [1 3 2]);
                end
                obj(i).Coefs = coefs;
            end%for
            
        end%fcn
        
        function [tau,idx] = s2tau(obj, s)
            
            if obj.isempty() || (obj.length() < eps) % Zero-length path
                tau = nan(size(s));
                idx = zeros(size(s), 'uint32');
                if ~obj.isempty()
                    theIdx = abs(s) < eps;
                    tau(theIdx) = 0;
                    idx(theIdx) = 1;
                end
                return
            end
            
            N = obj.numel();
            S = obj.ArcLengths;
            if obj.IsCircuit
                s = mod(s, S(end));
            end
            
            % Get the segment index
            [~,idx] = histc(s, [-inf; S; inf]); %#ok<HISTC>
            idx = min(uint32(idx), N);
            
            breaks = obj.Breaks;
            pp = obj.mkpp();
            tau = coder.nullcopy(zeros(size(s)));
            S = circshift(S, 1);
            S(1) = 0;
            for i = 1:N % Loop over spline segments
                if any(idx == i)
                    taui = linspace(breaks(i), breaks(i+1), 1000);
                    dxy = diff(ppval(pp, taui), 1, 2);
                    ds = cumsum(hypot(dxy(1,:), dxy(2,:))); 
                    tau(idx == i) = interp1([0 ds] + S(i), taui, s(idx == i), ...
                        'linear','extrap');
                end
            end
            
        end%fcn
        
        function obj = select(obj, idxs)
            
            assert(max(idxs) - 1 < obj.numel(), 'SplinePath:BadSubscript', ...
                'Path contains only %d pieces!', obj.numel())
            
            if isa(idxs, 'logical')
                idxs_ = find(idxs(:));
            else
                idxs_ = idxs(:);
            end
            
            assert(all(diff(idxs_) > 0), 'SplinePath:PiecesReorderingUnsupported', ...
                'Reordering of pieces is not supported!')
            assert(all(diff(idxs_) < 2), 'SplinePath:PiecesSkippingUnsupported', ...
            'Skipping pieces is not supported!')
            
            obj.Breaks = obj.Breaks([idxs_; idxs_(end) + 1]);
            obj.Coefs = obj.Coefs(:,idxs_,:);
            
            s = obj.ArcLengths(idxs_);
            obj.ArcLengths = s - s(1);
            
        end%fcn
        
        function obj = shift(obj, P)
            
            % Handle input arguments
            narginchk(1, 2);
            
            if nargin < 2
                P = -obj(1).termPoints();
            end%if
            
            if numel(P) ~= 2 || ~isnumeric(P)
                error(['Method SHIFTBY requires a numeric input',...
                    ' argument with two elements.']);
            end%if
            
            % BUILTIN is supported for code-generation starting with R2017b
            for i = 1:builtin('numel', obj)
                obj(i).Coefs(:,:,end) = bsxfun(@plus, obj(i).Coefs(:,:,end), P(:));
            end%for
            
        end%fcn
        
        function [P0,P1] = termPoints(obj)
            
            [tau0,tau1] = obj.domain();
            [x,y] = obj.eval([tau0,tau1]);
            P0 = [x(1); y(1)];
            P1 = [x(2); y(2)];
            
        end%fcn
        
        function s = toStruct(obj)
            s = struct(...
                'breaks',obj.Breaks', ...
                'coefs',obj.Coefs, ...
                'lengths',obj.ArcLengths);
        end%fcn
    end%methods
    
    
    methods (Access = private)
        function s = idxTau2s(obj, idx, tau)
        % IDXTAU2S  Lenghts from path segment IDX and path parameter TAU.
        
            tau0 = obj.Breaks(idx)';
%             assert(all(tau0 < tau));
            
            stmp = [0; obj.ArcLengths];
            s = stmp(idx);
            for i = 1:numel(tau)
                taui = linspace(tau0(i), tau(i), 100);
                xy = diff(ppval(obj.mkpp(), taui), 1, 2);
                s(i) = s(i) + sum(hypot(xy(1,:), xy(2,:)));
            end
        end%fcn
        
        function s = calcArcLength(obj, breaks, coefs)
            
            if nargin < 2
                breaks = obj.Breaks;
                coefs = obj.Coefs;
            end
            
            N = size(coefs, 2);
            pp = mkpp(breaks, coefs, 2);
            s = coder.nullcopy(zeros(N,1));
            for i = 1:N
                tau = linspace(breaks(i), breaks(i+1), 1000);
                dxy = diff(ppval(pp, tau), 1, 2);
                s(i) = sum(hypot(dxy(1,:), dxy(2,:)));
            end
            s = cumsum(s);
            
        end%fcn
    end
    
    
    methods (Access = {?Path2D})
        function s = lengthImpl(obj, tau0, tau1)
            if nargin < 3
                tau1 = tau0;
                tau0(:) = obj.Breaks(1);
            end
            
            assert(isequal(size(tau0), size(tau1)), ...
                    'Path2D:SizeMismatch', ...
                    'Path parameter argument sizes mismatch!')
            
            ppd = ppdiff(obj.mkpp());
            s = coder.nullcopy(zeros(size(tau0)));
            for i = 1:numel(s)
                s(i) = pplen(ppd, tau0(i), tau1(i));
            end
        end%fcn
    end%methods
    
    
    methods (Static)
        function obj = pp2Path(pp, varargin)
        % PP2PATH    Path object from piecewise polynomial.
        %   OBJ = SplinePath.PP2PATH(PP) instantiates the path OBJ
        %   from piecewise polynomial struct PP.
        %   
        %   OBJ = SplinePath.PP2PATH(PP, VARARGIN) sets additional
        %   constructor arguments via VARARGIN.
        %
        %   See also SPLINEPATH/SPLINEPATH, MKPP.
        
            [breaks,coefs,nbrSeg,polyOrd,dim] = unmkpp(pp);
            assert(dim == 2, 'Spline must be 2D valued!')
            obj = SplinePath(breaks, reshape(coefs, 2, nbrSeg, polyOrd), varargin{:});
        end%fcn
        
        function obj = bezier2Path(P)
        % BEZIER2PATH   Path object from Bezier control.
        %   OBJ = SplinePath.BEZIER2PATH(P) instantiates the path OBJ from
        %   an array P of Bezier control points. P is of size
        %   2-by-NP-by-NO, where NP is the number of spline segments and NO
        %   the polynomial order.
        %
        %   Example:
        %    SplinePath.bezier2Path(cat(3, ...
        %       [0 0; 2 0; 5 1; 10 1]', [10 1; 15 1; 18 0; 20 0]'))
            
            [nr,nc,np] = size(P);
            assert(nr == 2, 'SplinePath:bezier2Path', 'Control points must be 2D!')
            
            coefs = zeros(size(P));
            for i = 0:nc-1
                bi = nchoosek(nc-1, i);
                for k = 0:nc-1-i
                    c = bi*nchoosek(nc-1-i, k)*(-1)^k;
                    coefs(:,i+k+1,:) = coefs(:,i+k+1,:) + P(:,i+1,:)*c;
                end
            end
            obj = SplinePath(0:np, permute(flip(coefs, 2), [1 3 2]));
            
        end%fcn
        
        function obj = circle(r, phi01, N)
        %CIRCLE     Create circle.
        %
        %   Reasonable circle fits are obtained for N >= 4 spline segments.
        %
        %   See also Path2D/circle, SPLINE.
            
            if nargin < 3
                N = 3;
            end    
            if nargin < 2
                phi01 = [0; 2*pi];
            end
            
            % This will become the breaks vector
            x = linspace(phi01(1), phi01(2), N+1);
            
            % Define sample points as well as the terminal derivatives
            dy = @(phi) [-sin(phi); cos(phi)];
            y = cat(2, dy(phi01(1)), [cos(x); sin(x)], dy(phi01(2)));
            
            % Create the circle
            pp = spline(x, r*y);
            obj = SplinePath.pp2Path(pp);
            
%             h = obj.plot('LineWidth',1, 'DisplayName','Spline');
%             [xBreaks,yBreaks] = obj.eval(obj.Breaks);
%             hold on
%             plot(xBreaks, yBreaks, 'o', 'Color',get(h, 'Color'), 'DisplayName','Breaks')
%             N = 100; % Samples per spline segment
%             phi = linspace(0, 2, N*obj.numel())*pi;
%             plot(r*cos(phi), r*sin(phi), 'r--', 'DisplayName','Circle');
%             hold off
            
        end%fcn
        
        function obj = connect(P0, P1)
        %CONNECT    Spline path from initial to target configuration.
        %   OBJ = SplinePath.CONNECT(P0, P1) creates a polynomial path OBJ
        %   connecting the inital/end configuration P0/P1, where Pi has as
        %   many rows as differential boundary conditions, i.e.:
        %       Pi = [xi yi;
        %             dxi/dt dyi/dt;
        %             ...
        %             dxi/dt dyi/dt;
        
            % Number of boundary conditions at start/end
            NBC0 = size(P0, 1);
            NBC1 = size(P1, 1);
            
            % To have a well-define system of equations, we need one
            % polynomial coefficient per boundary condition
            Nc = NBC0 + NBC1;
            
            A1 = coder.nullcopy(zeros(NBC0, Nc));
            A1(1,1) = 1;
            for i = 2:NBC0
                A1(i,i) = gamma(i); % gamma(n+1) = n!
            end
            
            A2 = coder.nullcopy(zeros(NBC1, Nc));
            A2(1,:) = 1;
            for i = 2:NBC1
                A2(i,:) = A2(i-1,:).*[zeros(1,i-2), 0:Nc+1-i];
            end
            
            coefs = flip([A1;A2]\[P0; P1], 1);
            obj = SplinePath([0 1], reshape(coefs', [2 1 Nc]));
        end%fcn
        
        function obj = straight(P0, P1)
            x0 = P0(1);
            y0 = P0(2);
            x1 = P1(1);
            y1 = P1(2);
            obj = SplinePath([0 1], ...
                cat(3, [x1-x0; y1-y0], [x0; y0]), ...
                hypot(x1-x0, y1-y0), ...
                false);
        end%fcn
        
        function obj = fromStruct(s)
            obj = SplinePath(s.breaks, s.coefs, s.lengths);
        end%fcn
        
        function c = getBusDef(N, M)
        % GETBUSDEF     Return bus information.
        %   C = GETBUSDEF(N,M) returns the bus information cell C for a
        %   SplinePath of at most N segments and degree M-1.
        %
        %   See also Path2D/getBusDef.
            BusName = 'SBus_SplinePath';
            HeaderFile = '';
            Description = '';
            BusElements = {...
                {'breaks',    N+1, 'double', -1, 'real', 'Sample', 'Variable', [], [], '', ''},...
                {'coefs', [2 N M], 'double', -1, 'real', 'Sample', 'Variable', [], [], '', ''},...
                {'lengths',     N, 'double', -1, 'real', 'Sample', 'Variable', [], [], '', ''},...
                };
            c = {{BusName,HeaderFile,Description,BusElements}};
        end%fcn
    end%methods
    
end%class


function poly = calcPointProjPolynomial(poiX, poiY, px, py, Nc)
% Returns the polynomial coefficients of the polynomial defining the point
% projection equation.

dpx = px(1:end-1).*(Nc-1:-1:1);
dpy = py(1:end-1).*(Nc-1:-1:1);

% Convolution returns array of size 2N-2. Therefore, pad non-convolutional
% term with zeros.
poly = -(conv2(px,dpx) + conv2(py,dpy));
poly(Nc:end) = poly(Nc:end) + poiX*dpx + poiY*dpy;

end%fcn

function r = getRealRootsWithinBounds(c, ubh, ub)

ri = realroots(c);
isValid = (0 <= ri) & ubh(ri, ub);
r = ri(isValid);

end%fcn
