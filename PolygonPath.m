classdef (InferiorClasses = {?matlab.graphics.axis.Axes}) PolygonPath < Path2D
%POLYGONPATH    Polygon path.
%   Path representation using polygonal chain assuming linear
%   interpolation. The path parameter is inherited from the number of
%   waypoints N, i.e. [0,1,...,N-1]. Therefore, a path parameter of e.g.
%   2.75 refers to the path three quarters between the third and fourth
%   waypoint.
% 
%   PolygonPath properties:
%   x - Cartesian x-coordinate.
%   y - Cartesian y-coordinate.
%   head - Heading in radians.
%   curv - Curvature in 1/m.
% 
%   PolygonPath methods:
%   PolygonPath - Constructor.
%   discreteFrechetDist - Discrete Frechet distance.
%   fitCircle - Fit circle to path.
%   fitStraight - Fit straight line to path.
%   interp - Interpolate path.
%   perpendicularDistance - Distance of path waypoints to line.
%   rdp - Ramer-Douglas-Peucker point reduction.
%   rdpIter - Iterative Ramer-Douglas-Peucker line simplification.
%   simplify - Simplify path.
%   write2file - Write path to file.
%   See superclass for more methods.
% 
%   PolygonPath static methods:
%   circle - Circle path.
%   clothoid - Clothoid path.
%   omegaTurn - Omega shaped turn path.
%   See superclasses.
% 
%   See also PATH2D.

%#ok<*PROP>
%#ok<*PROPLC> % There is a property named xyz. Maybe this is a reference to it?

    properties
        x = zeros(0, 1)
        y = zeros(0, 1)
        head = zeros(0, 1)
        curv = zeros(0, 1)
    end
    
    
    
    methods
        function obj = PolygonPath(x, y, head, curv, isCircuit)
        %POLYGONPATH    Create polygon path object.
        %   OBJ = POLYGONPATH() creates an empty path.
        %
        %   OBJ = POLYGONPATH(X,Y,HEAD,CURV) create polygonal curve OBJ
        %   with points (X,Y), heading HEAD in radians and curvature CURV.
        %   The path parameter is inherited according to
        %   [0,1,...,N-1], where N =numel(X) = numel(Y) = numel(HEAD) =
        %   numel(CURV).
        % 
        %   OBJ = POLYGONPATH(___,ISCIRCUIT) set to true if the path is a
        %   circuit.
        %
        
            if nargin < 1
                % Return with default values
                return
            end
            
            assert(isequal(numel(x), numel(y), numel(head), numel(curv)), ...
                'PolygonPath:Constructor:numelXYHC', ...
                'Number of path property elements mismatch!');
            obj.x = x(:);
            obj.y = y(:);
            obj.head = head(:);
            obj.curv = curv(:);
            if isempty(x)
                obj.ArcLengths = zeros(0,1);
            else
                obj.ArcLengths = cumsum(hypot(diff(x(:),1,1), diff(y(:),1,1)));
                end
            
            if nargin < 5
                obj = obj.setIsCircuit(1e-5);
            else
                obj.IsCircuit = isCircuit;
            end%if
            
        end%Constructor
        
        function obj = append(obj, obj2)
            obj = PolygonPath(...
                [obj.x; obj2.x], ...
                [obj.y; obj2.y], ...
                [obj.head; obj2.head], ...
                [obj.curv; obj2.curv]);
        end%fcn
        
        function [sd,Q,idx,tau,dphi] = cart2frenet(obj, xy, ~, doPlot)
        %
        %   See also PATH2D/CART2FRENET.
        
        %   EXAMPLES:
        %   >> sd = cart2frenet(PolygonPath.xy2Path(0:3, [0 0 1 0]), [1; 1])
        %   sd = 
        %        1.7071
        %       -0.7071
        % 
        %   >> sd = cart2frenet(PolygonPath.xy2Path(0:2, [0 0 -1]), [1.5; 3.5])
        %   sd = 
        %      1.0000
        %     -3.5355
        
            if nargin < 4
                doPlot = false;
            end
            
            [Q,idx,tau,dphi] = obj.pointProjection(xy, [], doPlot);
            N = numel(obj.x);
            assert(all(idx < N))
            if isempty(Q)
                % Take the waypoint closest to point of interest
                [~,minIdx] = min(hypot(obj.x - xy(1), obj.y - xy(2)));
                Q = [obj.x(minIdx) obj.y(minIdx)];
                tau = minIdx - 1;
                if minIdx == N
                    % Avoid out of range indexing below (getting the
                    % orientation vector)
                    idx = minIdx - 1; 
                else
                    idx = minIdx;
                end
            end%fcn
            
            % Get the orientation vector related with Q to calculate the
            % sign of distance from point of interest to path
            idx0 = idx;
            idx1 = idx + 1;
            ux = obj.x(idx1) - obj.x(idx0);
            uy = obj.y(idx1) - obj.y(idx0);
            
            % Get sign via z-component of cross product u x (Q-poi)
            dx = Q(:,1) - xy(1);
            dy = Q(:,2) - xy(2);
            signD = sign(ux.*dy - uy.*dx);
            
            S = obj.arcLengths0();
            % Go to local path parameter for path length computation
            sd = [S(idx0) + (tau-idx+1).*(S(idx1)-S(idx0)), signD.*hypot(dx,dy)];
            if isempty(dphi)
                dphi = abs(pi/2 - abs(atan2(ux.*dy - uy.*dx, ux.*dx + uy.*dy)));
            end
            
        end%fcn
        
        function obj = derivative(obj, n)
            
            if nargin < 2
                n = 1;
            elseif n > 1
                error('N>1 not implemented!')
            end
            
            if (n < 1) || obj.isempty()
                return
            end
            
            % This approach also works for paths defined at a single point
            h = obj.head;
            obj.x = cos(h);
            obj.y = sin(h);
            obj.curv(:) = 0;
            obj.IsCircuit = false;
            
        end%fcn
        
        function d = discreteFrechetDist(obj, Q, distFcn)
        %DISCRETEFRECHETDIST    Compute the discrete Fréchet distance.
        %   D = DISCRETEFRECHETDIST(OBJ, Q) returns the discrete frechet
        %   distance D for the PolygonPath object OBJ and an N-by-2 matrix
        %   Q of coordinates.
        %
        %   D = DISCRETEFRECHETDIST(___,DISTFCN) allows to define a custom
        %   distance function DISTFCN as an anonymous function, e.g.
        %       @(dpq) hypot(dpq(:,1), dpq(:,2))
        %   which is the default value.
        
            if nargin < 3 % Define default metric
                distFcn = @(dpq) hypot(dpq(:,1), dpq(:,2));
            end
            
            D = coder.nullcopy(-ones(numel(obj.x), size(Q,1)));
            
            % Fill the first row/column with cumulative maximum of
            % distances from P0 to Qi/Pi to Q0.
            dP0toQ = distFcn([obj.x(1) - Q(:,1), obj.y(1) - Q(:,2)]);
            D(1,:) = cummax(dP0toQ);
            dQ0toP = distFcn([obj.x - Q(1,1), obj.y - Q(1,2)]);
            D(:,1) = cummax(dQ0toP);
            
            % Since the first row/column are already filled, the remaining
            % elements of the distance matrix can be calculated without
            % branching.
            for i = 2:size(D,1)
                Pi = [obj.x(i) obj.y(i)];
                for j = 2:size(D,2)
                    d = distFcn(Pi - Q(j,:));
                    dTmp = [D(i-1,j) D(i-1,j-1) D(i,j-1)];
                    D(i,j) = max(d, min(dTmp));
                end
            end
            
            d = D(end,end);
        end%fcn

        function [tauL,tauU] = domain(obj)
            if isempty(obj)
                tauL = NaN;
                tauU = NaN;
            else
                tauL = 0;
                tauU = obj.numel();
            end
        end%fcn
        
        function [x,y,tau,head,curv,curvDs] = eval(obj, tau, extrap)
        %EVAL   Evaluate path at path parameter.
        %   According to the definition of a polygonal chain, EVAL performs
        %   linear interpolation between the waypoints (x,y). It also uses
        %   linear interpoation for the heading as well as curvature. The
        %   derivative of the curvature w.r.t. path length is estimated via
        %   gradients.
        %
        %   See also POLYGONPATH/INTERP.
            
            if nargin < 3
                extrap = false;
            end
            if nargin < 2
                x = obj.x;
                y = obj.y;
                head = obj.head;
                curv = obj.curv;
                curvDs = obj.estimateCurvDs();
                tau = (0:numel(x)-1)';
                return
            end
            
            N = numel(obj.x);
            tau = tau(:);
            if N > 1 % At least 2 sample points
                tauAct = (0:N-1)';
                [~,binIdx] = histc(tau, [-inf; tauAct; inf]); %#ok<HISTC>
                binIdxSat = max(min(binIdx-1, N-1), 1);
                
                % Linear interpolation
                lin = [obj.x obj.y obj.head obj.curv obj.estimateCurvDs()];
                xyhc = lin(binIdxSat,:) + bsxfun(@times, ...
                    tau - tauAct(binIdxSat), ...
                    lin(binIdxSat+1,:) - lin(binIdxSat,:));
                
                if ~extrap
                    % Set rows corresponding to interpolation values
                    % outside domain to NaN
                    isOutsideDomain = (tau < 0) | (tau > N-1);
                    xyhc(isOutsideDomain,:) = NaN;
                    tau(isOutsideDomain) = NaN;
                end
                
            elseif N > 0 % Just one sample point (no extrapolation)
                xyhc = repmat(...
                    [obj.x(1) obj.y(1) obj.head(1) obj.curv(1) obj.estimateCurvDs()], ...
                    numel(tau), 1);
                xyhc(tau ~= 0,:) = NaN;
                tau(tau ~= 0) = NaN;
            else % Empty path (no extrapolation)
                xyhc = NaN(numel(tau), 5);
                tau(:) = NaN;
            end%if
            
            x = xyhc(:,1);
            y = xyhc(:,2);
            head = xyhc(:,3);
            curv = xyhc(:,4);
            curvDs = xyhc(:, 5);
            
        end%fcn
        
        function tau = findZeroCurvature(obj, ths)
            
            if nargin < 2
                ths = eps;
            end
            tau = find(abs(obj.curv) < ths) - 1;
            
        end%fcn
        
        function [objc,e,xc,yc,R] = fitCircle(obj, N, doPlot)
        %FITCIRCLE  Fit a circle to PolygonPath.
        %   [OBJC,E,XC,YC,R] = FITCIRCLE(OBJ,N) fits a circle with
        %   center(XC,YC) and radius R to waypoints (OBJ.X, OBJ.Y)
        %   minimizing the error E. Returns OBJC of class POLYGONPATH
        %   sampled at angles according to linspace(0, 2*pi, N).
        %   
        %   [___] = FITCIRCLE(...,DOPLOT) allows to enable the plot for
        %   checking the fitting result visually.
        %    
        %   Minimization is performed in the least-squares sense minimizing
        %   the sum of squared errors:
        %     SUM[(R(i)^2-R^2)^2]
        %      i
        %    
        %   See also POLYGONPATH/FITSTRAIGHT.
            
            [xc,yc,R,e] = fitCircle_Kasa(obj.x, obj.y);
            
            % Create POLYGONPATH object
            objc = PolygonPath.circle(R, [0 2*pi], N);
            objc.x = objc.x + xc;
            objc.y = objc.y + yc;
            
            % Plot if requested
            if (nargin > 2) && doPlot
                plot(obj, 'b-', 'MarkerSize',10, 'DisplayName','PolygonPath');
                hold on
                plot(objc, 'Color','k', 'DisplayName','Circle');
                hold off
                legend('-DynamicLegend')
            end%if
            
        end%fcn
        
        function [objs,e] = fitStraight(obj, doPlot)
        %FITSTRAIGHT    Fit a straight line to PolygonPath.
        %   [OBJS,E] = FITSTRAIGHT(OBJ) fits a straight line OBJS to the
        %   path OBJ minimizing the error E.
        %   
        %   [___] = FITSTRAIGHT(OBJ,DOPLOT) allows to enable the plot for
        %   checking the fitting result visually.
        %   
        %   Minimization is performed in the least-squares sense minimizing
        %   the sum of squared errors:
        %     SUM[(OBJ.Y - YFIT)^2]
        %    
        %   See also POLYGONPATH/FITCIRCLE.
        
            % Extract x/y data
            objX = obj.x;
            objY = obj.y;
            N = numel(objX);
            
            % Create (overdetermined) system of equations for the unknowns
            % y0 and y1, where 
            %   y(tau) = y0 + tau/(N-1)*(y1 - y0)
            s = obj.cumlengths();
%             tmp = linspace(0, 1, N)';
            tmp = [0; s]/s(end);
            A = [1 - tmp, tmp];
            
            % Solve system of equations: A*x = b, where x = [y(0); y(N-1)]
            xy01 = pinv(A)*[objX objY];
            objs = PolygonPath.straight(xy01(1,:), xy01(2,:));
            
            if nargout > 1
                % Calculate error
                [xS,yS] = objs.eval(tmp);
                e = 1/N*sum(([objX-xS objY-yS]).^2, 1)';
            end
            
            if (nargin > 1) && doPlot % Plot if requested
                plot(obj, 'r', 'Marker','.', 'DisplayName','PolygonPath');
                hold on
                plot(objs, 'b', 'Marker','o', 'DisplayName','Straight');
                hold off
                legend('-DynamicLegend')
            end%if
            
        end%fcn
        
        function [xy,Q,idx,tau] = frenet2cart(obj, sd, doPlot)
        
            Pxy = [obj.x, obj.y]; % Initial/terminal points per path segment
            assert(size(Pxy,1) > 1, 'Path must have at least two points!');
            
            % Get the indexes referring to the path segments according to
            % the frenet coordinates s-value
            [tau,idx] = obj.s2tau(sd(:,1));
            
            % Orientation vectors (TODO: make use of heading?)
            u = Pxy(idx + 1,:) - Pxy(idx,:);
            
            % The points on the path (i.e. d=0) are given by the segments
            % initial point plus the remaining length along the current
            % segment
            dtau = tau - double(idx-1);
            Q = Pxy(idx,:) + bsxfun(@times, dtau, u);
            
            % From Q, go D units along normal vector of U
            u = bsxfun(@rdivide, u, hypot(u(:,1), u(:,2)));
            xy = Q + bsxfun(@times, sd(:,2), [-u(:,2), u(:,1)]);
            
            if (nargin > 2) && doPlot
                plot(obj, 'Marker','.', 'MarkerSize',15, 'DisplayName','PolygonPath');
                hold on
                plot(xy(:,1), xy(:,2), 'o', 'DisplayName','xy');
                plot(Q(:,1), Q(:,2), 'kx', 'DisplayName','Q');
                hold off
                legend('-DynamicLegend')
            end%if
            
        end%fcn
        
        function flag = isempty(obj)
        % ISEMPTY   Check if path is empty.
        %   FLAG = ISEMPTY(OBJ) returns true if the path contains no
        %   waypoints, i.e. numel(OBJ) = 0, and false otherwise.
        %
        %   See also NUMEL.
        
            % Overload base-class method
            flag = isempty(obj.x);
        end%fcn
        
        function obj = interp(obj, tau, varargin)
        %INTERP     Interpolate path.
        %   OBJ = INTERP(OBJ,TAU) interpolate path OBJ w.r.t. path
        %   parameter TAU.
        %
        %   OBJ = INTERP(__,ARGS) specify interpolation settings via ARGS.
        %
        %   See also INTERP1.
            
            narginchk(2, 4) % object, query points, method, extrapolation
            
            % INTERP1 requires at least two samples
            N = numel(obj.x);
            assert(N > 1)
            
            % Require tau to be strictly increasing so that path length is
            % strictly increasing
            assert(all(diff(tau) > 0))
            
            xyhcs = interp1(...
                [obj.x, obj.y, obj.head, obj.curv, obj.arcLengths0()], ...
                tau(:) + 1, varargin{:});
            obj = PolygonPath(xyhcs(:,1), xyhcs(:,2), xyhcs(:,3), xyhcs(:,4));
            obj.ArcLengths = xyhcs(2:end,5);
            
        end%fcn
        
        function [xy,tau,errFlag] = intersectCircle(obj, C, r, doPlot)
            
        %   EXAMPLES:
        %   >> s = PolygonPath.xy2Path([0 0 -3 -2 -4 -3 1 1], [0 1 2 3 4 5 4 0]);
        %   >> intersectCircle(s, [-1 3], 2, true)
            
            % Brute force approach: check every path segment
            idxs = (1:obj.numel())';
            x0 = obj.x - C(1);
            dx = diff(x0);
            x0(end) = [];
            y0 = obj.y - C(2);
            dy = diff(y0);
            y0(end) = [];
            
            % Each path segment is written as a line P(t) = P0 + t*(P1-P0)
            % from its initial point P0 to its end point P1, where t =
            % 0,..,1. This results in
            %   [x0 + t(x1-x0)]^2 + [y0 + t(y1-y0)]^2 = r^2
            % which requires solving a quadratic polynomial 
            %   a*t^2 + b*t + c = 0
            a = dx.^2 + dy.^2;
            b = 2*(x0.*dx + y0.*dy);
            c = x0.^2 + y0.^2 - r^2;
            discriminant = b.^2 - 4*a.*c;
            
            % Secant solutions (two solutions per segment)
            isSecant = (discriminant > 0);
            xi = sqrt(discriminant(isSecant));
            tauSecant = 0.5*[...
                (-b(isSecant) + xi)./a(isSecant); ...
                (-b(isSecant) - xi)./a(isSecant)];
            idxSecant = repmat(idxs(isSecant), [2,1]);
            isValidSec = ~((tauSecant < 0) | (tauSecant > 1));
            
            % Tangent solutions (one solution per segment)
            isTangent = ~((discriminant < 0) | isSecant); % (discriminant == 0)
            tauTangent = 0.5*-b(isTangent)./a(isTangent);
            idxTangent = idxs(isTangent);
            isValidTan = ~(tauTangent < 0) & (tauTangent < 1);
            
            % Combined set of solutions
            tauLoc = [tauSecant(isValidSec); tauTangent(isValidTan)];
            segIdx = [idxSecant(isValidSec); idxTangent(isValidTan)];
            
            % Set return values
            tau = sort(segIdx - 1 + tauLoc, 'ascend');
            [x,y] = obj.eval(tau);
            xy = [x,y];
            errFlag = isempty(tau);
            
            % At most two intersections per path segment!
            assert(size(xy, 1) <= (numel(obj.x)-1)*2)
            assert(size(xy, 1) == size(tau, 1))
                
            if (nargin > 3) && doPlot
                [~,ax] = plot(obj, 'Marker','.');
                hold on
                phi = 0:pi/100:2*pi;
                plot(ax, r*cos(phi) + C(1), r*sin(phi) + C(2), 'DisplayName','Circle');
                plot(ax, xy(:,1), xy(:,2), 'kx', 'DisplayName','Intersections')
                hold off
            end%if
            
        end%fcn
        
        function [xy,tau,errFlag] = intersectLine(obj, O, psi, doPlot)
            
            % Shift by line origin O and rotate so that the line is
            % horizontal
            R = rotmat2D(psi);
            xyPath = [obj.x - O(1), obj.y - O(2)] * R;
            xPath = xyPath(:,1);
            yPath = xyPath(:,2);
            
            % Find indexes where the paths y-component changes sign. SIGN
            % returns 0 only for arguments that are exactly equal to zero.
            % We try to catch values almost equal to zero via a magic
            % threshold.
            signs1 = int8(sign(yPath)); 
            signs2 = abs(yPath) > 1e-16;
            idxs0 = find(any(diff([signs1 signs2], 1, 1), 2));
            if isempty(idxs0) % No intersection of path/line
                xy = zeros(0, 2);
                tau = zeros(0,1);
                errFlag = true;
            else
                % End index can not exceed number of path samples since
                % indexes were obtained using DIFF!
                idxsE = idxs0 + 1;
                x0F = [xPath(idxs0), xPath(idxsE)];
                y0Fd = diff([yPath(idxs0), yPath(idxsE)], 1, 2);
                x = xPath(idxs0) - yPath(idxs0) .* diff(x0F, 1, 2)./y0Fd; 
                
                % Undo transformation. Due to the above rotation/shift, the
                % intersections y-component is zero. Therefore, only the
                % x-component needs to be rotated.
%                 xy = (R * [x';zeros(1,numel(x))] + repmat(O(:), [1,numel(x)]))';
                xy = [R(1,1)*x + O(1), R(2,1)*x + O(2)];
                
                % Since we assume linear interpolation between waypoints,
                % the local path segment parameter can be computed from x
                % or y
%                 tauLocal = (x - x0F(1))/diff(x0F);
                tauLocal = -yPath(idxs0)./y0Fd;
                tau = idxs0 - 1 + tauLocal;
                errFlag = false;
            end%if
            
            if (nargin > 3) && doPlot
                [~,ax] = plot(obj, 'Marker','.','MarkerSize',8);
                hold on
                
                [r1,r2] = scaleTangentToAxis(xlim, ylim, O, psi);
                Pstart  = [O(1) + r2*cos(psi); O(2) + r2*sin(psi)];
                Pstop   = [O(1) + r1*cos(psi); O(2) + r1*sin(psi)];
                h = plot(gca, [Pstart(1) Pstop(1)], [Pstart(2) Pstop(2)], ...
                    'Displayname','Line');
                plot(O(1), O(2), 'o', 'Color',get(h,'Color'), 'Displayname','O')
                
                plot(ax, xy(:,1), xy(:,2), 'kx', 'DisplayName','Intersections')
                hold off
            end%if
            
        end%fcn
        
        function n = numel(obj)
            n = max(0, numel(obj.x) - 1);
        end%fcn
        
        function d = perpendicularDistance(obj, P0, P1, doPlot)
        %PERPENDICULARDISTANCE    Perpendicular distance to line.
        %   D = PERPENDICULARDISTANCE(OBJ,P1,P2) calculate the
        %   perpendicular distance D for all waypoints of OBJ to the line
        %   passing through P1 and P2.
        %   
        %   D = PERPENDICULARDISTANCE(...,DOPLOT) also shows a plot of
        %   results if DOPLOT evaluates to TRUE.
        % 
        %   NOTE: The distance for waypoints of OBJ to the left/right of
        %   the line from P1 to P2 is positive/negative.
        %    
        %   See also
        %   https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
            
            % Handle input arguments
            narginchk(1, 4)
            
            if nargin < 3
                [P0,P1] = obj.termPoints();
            end%if
            
            assert(isequal(numel(P0), numel(P1), 2), ...
                'POLYGONPATH:perpendicularDistance', ...
                'Line must be defined using two 2-D points.');
            assert(~isequal(P0(:), P1(:)), ...
                'POLYGONPATH:perpendicularDistance', 'Points are equal!');
            
            
            % Calculate the perpendicular distance for all waypoints. See:
            % https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
            % Scale by -1 so that points to the left/right of the line from
            % P0 to P1 have a positive/negative distance value.
            dx = P1(1) - P0(1);
            dy = P1(2) - P0(2);
            d = -(dy*obj.x - dx*obj.y + P1(1)*P0(2) - P1(2)*P0(1)) / sqrt(dx^2 + dy^2);
            
            
            %%% Visualization
            if (nargin > 3) && doPlot
                % plot the path
                plot(obj, 'b.', 'MarkerSize',10);
                hold all
                
                % plot the line defined by points P1/P2
                plot([P0(1) P1(1)], [P0(2) P1(2)], 'k', 'Marker','.');
                
                % plot the perpendicular lines from waypoints to line
                % defined by points P0/P1; first calculate its slope and
                % offset
                slope = (P1(2) - P0(2))/(P1(1) - P0(1));
                offset = P0(2) - slope*P0(1);
                
                if isfinite(slope) && (slope ~= 0)
                    dstar = obj.y + 1/slope*obj.x;
                    x_ = (dstar-offset)/(slope+1/slope);
                    y_ = slope*x_ + offset;
                elseif ~isfinite(slope)
                    % x-coordinates of P1 and P2 are the same
                    x_ = P0(1)*ones(size(obj.x));
                    y_ = 0*x_ + obj.y;
                else
                    % slope of line P12 is 0
                    x_ = obj.x;
                    y_ = slope*x_ + offset;
                end
                plot([obj.x, x_]', [obj.y, y_]', 'r.-');
                
                hold off
            end%if
            
        end%fcn
        
        function [Q,idx,tau,dphi] = pointProjection(obj, poi, ~, doPlot)
            
            % Initial/terminal points per segment
            X = obj.x;
            Y = obj.y;
            
            % To find Q, two conditions must be satisfied: 
            % https://de.wikipedia.org/wiki/Orthogonalprojektion
            %    (1) Q = P0 + lambda * u, where u := P1-P0
            %    (2) dot(Q-POI, u) = 0
            % Inserting (1) into (2) yields 
            %   lambda = dot(POI - P0, u)/dot(u, u)
            P0 = [X(1:end-1), Y(1:end-1)];
            u = diff([X, Y], 1, 1);
            lambdas = dot(bsxfun(@minus, poi(:)', P0), u, 2) ./ sum(u.^2, 2);
            
            % Replace (x <= 1) by (x-eps < 1) because of:
            %   (1) Indexing into lambdas with "1:end-1" causes the
            %   Simulink error "Code generation assumption about size
            %   violated. Run-time indexing is scalar(vector), but
            %   compile-time assumption was vector(vector) indexing." for
            %   2-element paths (i.e. scalar lambdas).
            % idx = find((lambdas >= 0) & [lambdas(1:end-1) < 1; lambdas(end) <= 1]);
            lambdas(end) = lambdas(end) - eps(lambdas(end));
            idx = find((lambdas >= 0) & (lambdas < 1));
            
            % For paths with 2 elements, find can return an array of size
            % 0-by-0 which would raise an error in BSXFUN. Avoid by
            % explicit indexing the column or reshaping find result.
            idx = idx(:);
            tau = lambdas(idx); % NOTE: this is with respect to each segment!
            Q = P0(idx,:) + bsxfun(@times, tau, u(idx,:));
            tau = idx - 1 + tau;
            dphi = zeros(numel(idx), 1);
            
            if (nargin > 3) && doPlot
                plot(obj, 'Marker','.', 'MarkerSize',8, 'DisplayName','RefPath');
                hold on
                plot(obj.x(1), obj.y(1), 'g.', 'MarkerSize',18, 'DisplayName','Initial point');
                plot(poi(1), poi(2), 'ro', 'DisplayName','PoI')
                plot(Q(:,1), Q(:,2), 'kx', 'DisplayName','Q')
                legend('-DynamicLegend');
                plot(...
                    [Q(:,1)'; repmat([poi(1) NaN], size(Q,1),1)'],...
                    [Q(:,2)'; repmat([poi(2) NaN], size(Q,1),1)'], 'k:'); 
                hold off
            end%if
            
        end%fcn
        
        function [obj2,tau0,tau1] = restrict(obj, tau0, tau1)
            
            if isempty(obj) || isempty([tau0(:); tau1(:)])
                obj2 = obj;
                return
            end
            
            % Find the lower/upper index so that restricted domain is
            % covered
            [tauL,tauU] = obj.domain();
            if isempty(tau0) || (tau0 < tauL) || (tau0 > tauU)
                tau0 = tauL;
                idx0 = 1;
            else
                idx0 = floor(tau0) + 1;
            end
            if isempty(tau1) || (tau1 > tauU) || (tau1 < tauL)
                tau1 = tauU;
                idx1 = numel(obj.x);
            else
                idx1 = floor(tau1) + 1;
            end%if
            
            assert(tau0 < tau1, 'PolygonPath:restrict:domain', ...
                'tau0 >= tau1')
            
            [x,y,~,h,c] = obj.eval([tau0 tau1]);
            obj2 = obj.select(idx0:idx1);
            obj2.x([1 end]) = x;
            obj2.y([1 end]) = y;
            obj2.head([1 end]) = h;
            obj2.curv([1 end]) = c;
            
        end%fcn
        
        function [obj,idx] = rdp(obj, epsilon)
        %RDP    Recursive Ramer-Douglas-Peucker polyline simplification.
        %   OBJR = RDP(OBJ,EPS) applies the Ramer-Douglas-Peuker point
        %   reduction algorithm to path OBJ with parameter EPS. None of the
        %   removed waypoints has a distance greater than EPS to the
        %   resulting path!
        %    
        %   [OBJR,IDX] = RDP(OBJ,EPS) returns an array IDX so that OBJR =
        %   SELECT(OBJ, IDX).
        %
            
            % The actual implementation is moved to a separate file,
            % otherwise its nested function would block code generation for
            % all class methods in older MATLAB releases!
            keepIdx = ramerDouglasPeucker(obj.x, obj.y, epsilon);
            obj = obj.select(keepIdx);
            idx = find(keepIdx);
            
        end%fcn
        
        function [obj,idx] = rdpIter(obj, epsilon)
        %RDPITER    Iterative Ramer-Douglas-Peucker algorithm.
        %   OBJR = RDPITER(OBJ,EPS)
        %
        %   Use this implementation if code-generation is required!
        %
        %   See also PolygonPath/rdp.
        
            N = numel(obj.x);
            if N < 3
                return
            end
            
            % Initialize a logical array indicating which waypoints to keep
            keep = false(N,1);
            keep([1 end]) = true;
            
            % Track the segments to be checked. Each row is of the form
            % [start index, end index]. No upper bound is set for the
            % number of segments, since this would need to be a
            % compile-time constant.
            coder.varsize('segments', [inf 2], [true false]);
            segments = [1 N];
            
            while ~isempty(segments)
                % Work on the end segment
                idx0 = segments(end,1);
                idx1 = segments(end,2);
                segments(end,:) = [];
                
                dists = perpDist(obj.x(idx0:idx1), obj.y(idx0:idx1));
                [dMax,idxMax] = max(dists);
                
                if ~isempty(dMax) && (dMax > epsilon)
                    idxSplit = idxMax + idx0 - 1; % Offset to absolute index
                    
                    % Set index where to split the segment to be kept and
                    % add the two new segments to the stack
                    keep(idxSplit) = true;
                    segments = [segments; idx0 idxSplit; idxSplit idx1]; %#ok<AGROW>
                end
            end
            
            obj = obj.select(keep);
            idx = find(keep);
        end%fcn
        
        function obj = reverse(obj)
            
            for i = 1:builtin('numel', obj)
                obji = obj(i);
                if obji.numel() < 1
                    continue
                end
                obji.x = flip(obji.x);
                obji.y = flip(obji.y);
                obji.head = flip(obji.head) + pi;
                obji.curv = -flip(obji.curv);
                obji.ArcLengths = cumsum(flip(diff(obji.arcLengths0())));
                obj(i) = obji;
            end%for
            
        end%fcn
        
        function obj = rotate(obj, phi)
            
        %    If OBJ is an array of path objects, PHI is applied to all
        %    objects.
            
            % Handle input arguments
            narginchk(1, 2);
            
            if nargin < 2
                phi = -obj(1).head(1);
            end%if
            
            if numel(phi) ~= 1 || ~isnumeric(phi)
                error(['Method ROTATE requires a numeric input',...
                    ' argument with one element.']);
            end%if
            
            % Transposed rotation matrix in R^2 since rotation is performed
            % via p*R
            R = [cos(phi) sin(phi); -sin(phi) cos(phi)];
            for i = 1:builtin('numel', obj)
                % Much faster than rotMat*[obj.x,obj.y]'
                xy_new = [obj(i).x, obj(i).y]*R;
                
                obj(i).x = xy_new(:,1);
                obj(i).y = xy_new(:,2);
                obj(i).head = obj(i).head + phi;
            end%for
            
        end%fcn
        
        function obj = select(obj, idxs)
            obj = PolygonPath(...
                obj.x(idxs), ...
                obj.y(idxs), ...
                obj.head(idxs), ...
                obj.curv(idxs));
        end%fcn
        
        function obj = shift(obj, P)
            
            % Handle input arguments
            narginchk(1, 2);
            
            if nargin < 2
                P = -obj(1).termPoints();
            end%if
            
            if numel(P) ~= 2 || ~isnumeric(P)
                error(['Method SHIFT requires a numeric input',...
                    ' argument with two elements.']);
            end%if
            
            % Shift path
            for i = 1:builtin('numel', obj)
                obj(i).x = obj(i).x + P(1);
                obj(i).y = obj(i).y + P(2);
            end%for
            
        end%fcn
        
        function [obj,keep] = simplify(obj)
        %SIMPLIFY   Simplify path.
        %   OBJ = SIMPLIFY(OBJ) removes intermediate points from line
        %   segments of the path OBJ.
        %
        %   [OBJ,KEEP] = SIMPLIFY(OBJ) returns a logical vector KEEP
        %   indicating which waypoints are kept.
            
            h = atan2(diff(obj.y), diff(obj.x));
            dh = [true; diff(h); true];
            keep = (dh ~= 0);
            
            obj.x = obj.x(keep);
            obj.y = obj.y(keep);
            obj.head = obj.head(keep);
            obj.curv = obj.curv(keep);
            obj.ArcLengths = obj.ArcLengths(keep(2:end));
        end%fcn
        
        function [tau,idx] = s2tau(obj, s)
            
            sObj = obj.arcLengths0();
            N = numel(sObj);
            if N < 2
                % Paths with less than 2 waypoints have length 0
                tau = nan(size(s));
                idx = zeros(size(s), 'uint32');
                return
            end
            
            if obj.IsCircuit
                s = mod(s, obj.length());
            end
            [~,idx] = histc(s, [sObj;inf]); %#ok<HISTC>
            idx = min(max(uint32(idx), 1), N-1);
            
            ds = s - reshape(sObj(idx), size(s));
            tau = double(idx-1) + ds./reshape(sObj(idx+1) - sObj(idx), size(s));
            
        end%fcn
        
        function [P0,P1] = termPoints(obj)
            if isempty(obj)
                P0 = [NaN; NaN];
                P1 = [NaN; NaN];
            else
                P0 = [obj.x(1);    obj.y(1)];
                P1 = [obj.x(end);  obj.y(end)];
            end
        end%fcn
        
        function write2file(obj, fn)
        %WRITE2FILE		Write path to file.
        %	WRITE2FILE(OBJ,FN) writes waypoints OBJ to file with filename
        %	FN (specify extension!).
        %   
            
            % Open file with write-permission
            [~,~,fileExt] = fileparts(fn);
            assert(~isempty(fileExt), 'Specify file name extension!');
            fid = fopen(fn, 'w');
            
            doubleFmt = '%+15.5e ';
            fprintf(fid,...
                '%15s %15s %15s %15s %15s\n', ...
                'x (m)', 'y (m)', 'head (rad)', 'curv (1/m)', 's (m)');
            fprintf(fid,...
                [repmat(doubleFmt, [1,5]), '\n'], ...
                [obj.x, obj.y, obj.head, obj.curv, obj.arcLengths0()]');
            
            % Close file
            fclose(fid);
            
        end%fcn
        
        function s = toStruct(obj)
            s = struct('x',obj.x, 'y',obj.y, 'head',obj.head, 'curv',obj.curv);
        end%fcn
    end%methods
    
    
    methods (Access = private)
        function s = arcLengths0(obj)
            coder.inline('always')
            s = [0; obj.ArcLengths];
        end%fcn
        
        function curvDs = estimateCurvDs(obj)
        % Estimate change of curvature w.r.t. change in path length.
            ds = gradient1D(obj.arcLengths0());
            ds(ds < eps) = eps; % Avoid division by zero
            curvDs = gradient1D(obj.curv)./ds;
        end%fcn
    end
    
    
    methods (Access = {?Path2D})
        function s = lengthImpl(obj, tau0, tau1)
            
            s = interp1(obj.arcLengths0(), tau0 + 1);
            if nargin > 2
                assert(isequal(size(tau0), size(tau1)), ...
                    'Path2D:SizeMismatch', ...
                    'Path parameter argument sizes mismatch!')
                s = abs(interp1(obj.arcLengths0(), tau1 + 1) - s);
            end
            
        end%fcn
    end
    
    
    methods (Static)
        function obj = circle(r, phi01, N)
            
            if nargin < 3
                N = 100;
            end
            if nargin < 2
                phi01 = [0; 2*pi];
            end
            t = linspace(phi01(1), phi01(2), N+1)';
            signPhi = sign(phi01(2) - phi01(1));
            obj = PolygonPath(r*cos(t), r*sin(t), ...
                t + signPhi*pi/2, ...
                signPhi*repmat(1/r, N+1, 1));
            
            % Set exact path length
            obj.ArcLengths = abs(t(2:end) - t(1))*r; % r*phi where phi=0,...,2*pi
        end%fcn
        
        function obj = clothoid(L, curv01, N, MODE)
        %CLOTHOID   Create clothoid path.
        %   OBJ = POLYGONPATH.CLOTHOID(L, PHI01, N) creates a clothoid path
        %   of length L, initial curvature CURV01(1) and end curvature
        %   CURV01(2) at N equidistant sample points.
        % 
        %   OBJ = POLYGONPATH.CLOTHOID(___,MODE) allows to select different
        %   calculation methods. Possible values are 'Heald', 'Quad'.
        %   
        %   The clothoid's curvature K is proportional to its running
        %   length S by K = S/A^2, where A is the Clothoid parameter.
        %
        %   See also CLOTHOIDHEALD, CLOTHOIDQUAD.
            
            assert(numel(curv01) == 2)
            
            if nargin < 4
                MODE = 0;
            end
            
            curv0 = curv01(1);
            curv1 = curv01(2);
            signk = sign(curv1 - curv0);
            
            % Pre-calculation of path length
            dcurv = (curv1 - curv0)/L;
            A2 = 1/dcurv; % A^2
            A = sqrt(A2);
            s = signk*A2*linspace(curv0, curv1, N)';
            
            switch lower(MODE)
                case {0, 'quad'}
                    [x,y] = clothoidQuad(s, A);
                case {1, 'heald'}
                    [x,y] = clothoidHeald(s/(sqrt(pi)*A), A);
                otherwise
                    error('Unknown mode "%s".', num2str(MODE))
            end%switch
            
            curv = s/A2;
%             head = s.^2/(2*A2);
            head = curv.*s/2; % s.^2/(2*A^2)
            obj = PolygonPath(x, y, head, curv);
            obj.ArcLengths = s(2:end) - s(1);
            
        end%fcn
        
        function obj = omegaTurn(r, w, N, phi0)
        %OMEGATURN  Create omega turn.
        %   OBJ = OMEGATURN(R,W,N) creates an omega-shaped turn with radius
        %   R and width W using N sample points.
            
            signW = sign(w);
            w = abs(w);
            assert(w ~= 0);
            assert(2*r > w, 'Width must be less than diameter!')
            if nargin < 4
                phi0 = 0;
            end
            
            % The height and angle of the isosceles triangle
            h = 0.5*sqrt(12*r^2 - 4*r*w - w^2);
            a = asin(h/(2*r));
            
            % Distribute the number of samples according to the 
            s13 = r*a;
            s2 = r*(pi + 2*a); % From pi+a to -a
            sTotal = 2*s13 + s2;
            N13 = max(round(N/sTotal*s13), 1);
            N2 = N - 2*N13;
            
            % Create individual circles
            c1 = PolygonPath.circle(r, signW*[pi/2 pi/2-a] + phi0, N13);
            c2 = PolygonPath.circle(r, signW*[-a-pi/2 pi/2+a] + phi0, N2-1);
            c3 = PolygonPath.circle(r, signW*[a-pi/2 -pi/2] + phi0, N13);
            c3.head = c3.head - 2*pi;
            
            % Shift before appending
            c1 = c1.shift(-c1.termPoints() - [0; 0]);
            [~,dxy] = c1.termPoints();
            c2 = c2.shift(-c2.termPoints() + dxy);
            [~,dxy] = c2.termPoints();
            c3 = c3.shift(-c3.termPoints() + dxy);
            
            % Append individual paths to create omega-shape; avoid
            % overlapping samples
            obj = c1.append(c2.select(2:N2)).append(c3.select(2:N13+1));
            
            % Set exact path length
            s1 = c1.cumlengths();
            s2 = c2.cumlengths() + s1(end);
            s3 = c3.cumlengths() + s2(end);
            obj.ArcLengths = [s1; s2(1:N2-1); s3(1:N13)];
            
        end%fcn
        
        function obj = straight(P0, P1)
            x0 = P0(1);
            y0 = P0(2);
            x1 = P1(1);
            y1 = P1(2);
            h = atan2(y1-y0, x1-x0);
            obj = PolygonPath([x0 x1], [y0 y1], [h h], [0 0], false);
        end%fcn
        
        function obj = ll2Path(lat, lon)
            [x,y] = ll2utm(lat(:), lon(:)); % Convert from lat/lon to UTM
            obj = PolygonPath.xy2Path(x, y);
        end%fcn
        
        function obj = pp2Path(pp, tau, polyDeg)
        % PP2PATH    Path object from piecewise polynomial.
        %   OBJ = PolygonPath.PP2PATH(PP,TAU) instantiates the path OBJ
        %   from piecewise polynomial struct PP sampled at TAU.
        %   
        %   See also MKPP.
        
            if nargin < 3
                [~,~,~,polyOrd] = unmkpp(pp);
                polyDeg = polyOrd - 1;
            end
            
            % Derivative of piecewise polynomial
            d1pp = ppdiff(pp, polyDeg);
            d2pp = ppdiff(d1pp, polyDeg-1);
            
            % For piecewise polynomials with dim = N > 1, ppval returns a
            % N-by-* array
            xy = ppval(pp, tau)';
            d1xy = ppval(d1pp, tau)';
            d2xy = ppval(d2pp, tau)';
            h = cx2Heading(d1xy(:,1), d1xy(:,2));
            c = cx2Curvature(d1xy(:,1), d1xy(:,2), d2xy(:,1), d2xy(:,2));
            
            obj = PolygonPath(xy(:,1), xy(:,2), h, c);
        end%fcn
        
        function obj = xy2Path(x, y)
            [~,g1XY] = gradient([x(:) y(:)]);
            [~,g2XY] = gradient(g1XY);
            gx = g1XY(:,1);
            gy = g1XY(:,2);
            
            h = cx2Heading(gx, gy);
            c = cx2Curvature(gx, gy, g2XY(:,1), g2XY(:,2));
            obj = PolygonPath(x, y, h, c);
        end%fcn
        
        function obj = fromStruct(s)
            obj = PolygonPath(s.x, s.y, s.head, s.curv);
        end%fcn
        
        function c = getBusDef(N)
        % GETBUSDEF     Return bus information.
        %   C = GETBUSDEF(N) returns the bus information cell C for a
        %   PolynomialPath of at most N-1 segments.
        %
        %   See also Path2D/getBusDef.
            BusName = 'SBus_PolygonPath';
            HeaderFile = '';
            Description = '';
            BusElements = {...
                {'x',    N, 'double', -1, 'real', 'Sample', 'Variable', [], [], 'm', ''},...
                {'y',    N, 'double', -1, 'real', 'Sample', 'Variable', [], [], 'm', ''},...
                {'head', N, 'double', -1, 'real', 'Sample', 'Variable', [], [], 'rad', ''},...
                {'curv', N, 'double', -1, 'real', 'Sample', 'Variable', [], [], '1/m', ''},...
                };
            c = {{BusName,HeaderFile,Description,BusElements}};
        end%fcn
    end%methods
    
end%class


function g = gradient1D(f)

n = numel(f);
g = coder.nullcopy(zeros(size(f), 'like',f));

if n > 1
    % Take forward differences on left and right edges
    g(1) = f(2) - f(1);
    g(end) = f(end) - f(end-1);
    
    % Take centered differences on interior points
    g(2:end-1) = 0.5*(f(3:end) - f(1:end-2));
else
    g(:) = 0;
end

end%fcn
