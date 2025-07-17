classdef (InferiorClasses = {?matlab.graphics.axis.Axes}) Path2D
%PATH2D     Represent 2D paths.
%   This class is abstract and cannot be instantiated!
%   
%   Represent paths in the planar x/y coordinate frame utilizing a
%   parametric representation [x(tau); y(tau)]. It provides path operations
%   that can be especially usefull for automated driving and robotics
%   applications.
% 
%   Path2D properties:
%   IsCircuit - Indicates if path is a circuit.
% 
%   Path2D methods (modify path object):
%   append - Append paths.
%   derivative - Derivative of path.
%   restrict - Restrict path domain.
%   reverse - Reverse path.
%   rotate - Rotate path.
%   sampleDomain - Sample domain of path.
%   select - Select path segments.
%   setIsCircuit - Set property IsCircuit.
%   shift - Shift path.
% 
%   Path2D path operations:
%   cart2frenet - Convert cartesian point to frenet coordinates.
%   cumlengths - Cumulative path segment lengths.
%   domain - Domain of the path.
%   eval - Evaluate path at path parameter.
%   findZeroCurvature - Find path parameters where curvature vanishes.
%   frenet2cart - Convert frenet point to cartesian coordinates.
%   intersectCircle - Circle intersection.
%   intersectLine - Line intersection.
%   isempty - Check if path is empty/has zero length.
%   length - Path length.
%   numel - Number of path elements.
%   pointProjection - Point projection on path.
%   s2tau - Path length to path parameter.
%   termPoints - Terminal points.
%   vectorField - Vector field towards path.
% 
%   Path2D visualization methods:
%   plot - Plot the path.
%   plotG2 - Plot path, heading and curvature.
%   plottangent - Plot path and tangents.
% 
%   Path2D static methods:
%   ll2Path - Construct path from latitude/longitude coordinates.
%   pp2Path - Construct path from piecewise polynomial.
%   xy2Path - Construct path from cartesian coordinates.
%   circle - Construct circle.
%   straight - Construct straight path.
%   getBusDef - Get bus defintion.
%
%   See also PolygonPath, SplinePath.
    
    properties (SetAccess = protected)
        % ISCIRCUIT - Logical indicating if path is a circuit
        IsCircuit = false
    end
    
    properties (Access = protected)
        % ArcLengths - Cumulative length of path segments
        ArcLengths = zeros(0,1)
    end
    
    
    
    methods
        function obj = Path2D()
        %PATH2D     Construct a PATH2D class instance.
        end%Constructor
        
        function s = cumlengths(obj)
        % CUMLENGTHS    Cumulative path segment lengths.
        %   S = CUMLENGTHS(OBJ) returns the vector S of cumulative path
        %   segment lengths.
        %   
        %   See also LENGTH.
        
            s = obj.ArcLengths;
        end%fcn
        
        function flag = isempty(obj)
        % ISEMPTY   Check if path is empty.
        %   FLAG = ISEMPTY(OBJ) returns true if the path's domain is
        %   undefined, i.e. domain(OBJ) returns NaN, and false otherwise.
        %   
        %   See also DOMAIN.
        
            flag = (obj.numel() < 1);
        end%fcn
        
        function s = length(obj, varargin)
        % LENGTH    Path length.
        %   S = LENGTH(OBJ) returns the arc length S >= 0 of the path OBJ.
        %   For empty paths, S = 0.
        %   
        %   S = LENGTH(OBJ,TAU) returns the arc length from the initial
        %   point till the point at path parameter TAU.
        %   
        %   S = LENGTH(OBJ,TAU0,TAU1) returns the arc length between the
        %   points at path parameter TAU0 and TAU1.
        %   
        %   See also CUMLENGTHS.
        
            if isempty(obj.ArcLengths)
                s = 0;
            elseif nargin == 1
                s = obj.ArcLengths(end);
            else
                s = obj.lengthImpl(varargin{:});
            end
        end%fcn
        
        function tau = sampleDomain(obj, arg)
        %SAMPLEDOMAIN   Sample domain of path.
        %   TAU = SAMPLEDOMAIN(OBJ,ARG) returns the path parameter TAU
        %   sampled over the domain of path OBJ depending on the class of
        %   ARG: if ARG is 
        %     - double or single it is interpreted as a step increment,
        %     - uintx it is interpreted as the number of samples.
        %   
        %   In both cases, TAU contains the domain's terminal points and
        %   OBJ must be a non-empty path!
        
            assert(isscalar(arg));
            assert(~isempty(obj), 'PATH2D:sampleDomain:PathMustBeNonempty', ...
                'Path mut be non-empty!')
            
            [tau0,tau1] = obj.domain();
            switch class(arg)
                case {'double','single'} % Interpret as step increment
                    tmp = (tau0:arg:tau1)';
                    if tmp(end) < tau1 % Make sure to include the end point
                        tau = [tmp; tau1];
                    else
                        tau = tmp;
                    end%if
                    
                case {'uint8', 'uint16', 'uint32', 'uint64'}
                    tau = linspace(tau0, tau1, arg)';
                    
                otherwise
                    error('Unsupported data type for argument ARG!')
            end%switch
            
        end%fcn
        
        function obj = setIsCircuit(obj, ths)
        %SETISCIRCUIT   Sets property IsCircuit
        %   OBJ = SETISCIRCUIT(OBJ, THS) sets property IsCircuit to true if
        %   the distance between the path's terminal points is smaller than
        %   THS, and to false otherwise.
        
            [P0,P1] = obj.termPoints();
            obj.IsCircuit = (norm(P1 - P0) < ths);
        end%fcn
        
        function [Fx,Fy] = vectorField(obj, x, y, kf, doPlot)
        %VECTORFIELD    Vector field towards path.
        %   [FX,FY] = VECTORFIELD(OBJ, X, Y) computes the vector field
        %   F(X,Y) = [Fx;Fy] for the pairs (X,Y) that points towards the
        %   path OBJ.
        %
        %   [___] = VECTORFIELD(___, K) lets you specify the scaling of the
        %   normal over the tangential component of the vector field.
        %   Default value: 1.
        %   
        %   EXAMPLES: 
        %    po = PolygonPath.straight([0 0], [10 5]);
        %    [x,y] = meshgrid(linspace(-5,15,41), linspace(-5,10,31));
        %    po.vectorField(x, y, 1, true);
        % 
        %    po = SplinePath.circle(3, [0 2*pi], 6);
        %    [x,y] = meshgrid(linspace(-5,5,21), linspace(-5,5,21));
        %    po.vectorField(x, y, 1, true);
        % 
        %   REFERENCES:
        %    [1] A. M. C. Rezende, V. M. Goncalves and L. C. A. Pimenta,
        %    "Constructive Time-Varying Vector Fields for Robot
        %    Navigation," in IEEE Transactions on Robotics, vol. 38, no. 2,
        %    pp. 852-867, April 2022, doi: 10.1109/TRO.2021.3093674.
            
            assert(isequal(size(x), size(y)))
            
            if nargin < 4
                kf = 1;
            end
            
            if obj.isempty()
                Fx = NaN(size(x));
                Fy = NaN(size(y));
                return
            end
            
            [n,d,tau] = obj.closestUniquePointOnPath(x, y);
            
            % Obtain the normal vector from the closest point on the path
            n = bsxfun(@rdivide, [x(:) y(:)] - n, d + eps);
            
            % Since we calculate the tangent vector T = [tx ty] from the
            % derivative of the path, T always points in the direction of
            % the path w.r.t. increasing path parameter!
            PathObjT = obj.derivative();
            [tx,ty] = PathObjT.eval(tau);
            
            % In general, the path parameter is not the path length.
            % Therefore, we must normalize the tangent vector
            Th = hypot(tx, ty);
            
            fhG = @(D,kf) 2/pi*atan(kf*D);
%             fhG = @(D,kf) D./sqrt(kf + D.^2);
            G = fhG(d, kf);
            H = sqrt(1 - G.^2);
            Fx = -G.*n(:,1) + H.*(tx./Th);
            Fy = -G.*n(:,2) + H.*(ty./Th);
            
            Fx = reshape(Fx, size(x));
            Fy = reshape(Fy, size(y));
            
            if (nargin > 4) && doPlot
                obj.plot('r', 'LineWidth',2);
                hold on
                quiver(x, y, Fx, Fy, 'k');
%                 contour(x, y, reshape(d, size(x)));
                hold off
            end
            
        end%fcn
        
        
        %%% Plot methods
        function [hr,axr] = plot(varargin)
        %PLOT   Plot path.
        %   PLOT(OBJ) plots the path OBJ in terms of x over y.
        %   
        %   PLOT(OBJ,DTAU) specify path parameter increment to be plotted.
        %    
        %   PLOT(OBJ,S) additionally applies the line specification S.
        %
        %   PLOT(OBJ,DTAU,S) specify DTAU before any line specification.
        %
        %   PLOT(...,NAME,VALUE) specifies line properties using one or
        %   more Name,Value pair arguments.
        %
        %   PLOT(AX,...) plots into the axes with handle AX.
        %    
        %   [H,AX] = PLOT(...) returns the handle H to lineseries objects
        %   and axes handle AX.
        %    
        %   The line specification S is a character string supported by the
        %   standard PLOT command. For example
        %       PLOT(OBJ, 'LineWidth',2, 'Color',[.6 0 0]) 
        %   will plot a dark red line using a line width of 2 points.
        %   
        %   See also PLOTG2, PLOTTANGENT.
            
            [ax,obj,dtau,opts] = parsePlotInputs(varargin{:});
            [h,ax] = plotxy(ax, obj, dtau, opts{:});
            applyPlotxyStyles(ax);
            
            if nargout > 0
                hr = h;
                axr = ax;
            end
            
        end%fcn
        
        function [hr,axr] = plotG2(varargin)
        %PLOTG2     Plot path, heading and curvature.
        %
        %    For syntax see also PATH2D/PLOT.
            
            [ax,obj,dtau,opts] = parsePlotInputs(varargin{:});
            
            if isempty(ax) || any(~ishghandle(ax))
                ax = [...
                    subplot(2, 2, [1,3]); ...
                    subplot(2, 2, 2); ...
                    subplot(2, 2, 4)];
            elseif numel(ax) < 3
                error('Three axes handles are required!');
            end%if
            
            N = builtin('numel', obj);
            h = gobjects(N, 3);
            npStatus = get(ax(1:3), 'NextPlot');
%             set(ax(2:3), 'NextPlot','replace');
            for i = 1:N
                
                obji = obj(i);
                
                if i == 2
                    set(ax(1:3), 'NextPlot','add');
                end
                
                [h(i,1),~,tau] = plotxy(ax(1), obji, dtau, opts{:});
                [~,~,~,head,curv] = obji.eval(tau);
                h(i,2) = plot(ax(2), tau, head, opts{:});
                h(i,3) = plot(ax(3), tau, curv, opts{:});
            end%for
            set(ax(1:3), {'NextPlot'},npStatus);
            
            applyPlotxyStyles(ax(1));
            grid(ax(2), 'on');
            ylabel(ax(2), 'Heading (rad)');
            grid(ax(3), 'on');
            xlabel(ax(3), 'tau');
            ylabel(ax(3), 'Curvature (1/m)');
            
            % linkaxes(_, 'x') sets axes XLimMode to manual which can cause
            % data of subsequent calls of this method to be not completely
            % shown. Therefore reset axes limit mode to 'auto'.
            linkaxes(ax(2:3), 'x'); 
            set(ax, 'XlimMode', 'auto');
            
            if nargout > 0
                hr = h;
                axr = ax;
            end
            
        end%fcn
        
        function [hr,axr] = plottangent(varargin)
        %PLOTTANGENT    Plot path and tangents.
        %   PLOTTANGENT(OBJ,TAU) plots the path and tangents at the path
        %   parameters TAU and highlights the path coordinates at TAU. If
        %   TAU is empty, tangents are plotted for the path's terminal
        %   points.
        %    
        %   [H,AX] = PLOTTANGENT(...) returns a handle array H to
        %   lineseries objects and the axes handle AX. Here, H(1,1) is the
        %   path-handle, H(i+1,1) and H(i+1,2) the marker- and the
        %   tangent-handle of TAU(i) respectively.
        %    
        %   For Syntax see also PATH2D/PLOT.
        %   
        %   See also PATH2D/PLOT, PATH2D/PLOTG2.
            
            [ax,obj,tau,opts] = parsePlotInputs(varargin{:});
            if isempty(ax) || ~ishghandle(ax)
                ax = gca;
            end
            
            % Check if some path parameters are out of range
            [tauL,tauU] = obj.domain();
            tauExceeds = (tau > tauU) | (tau < tauL);
            if any(tauExceeds)
                warning('PATH2D:plottangent:tauOutOfRange', ...
                    'Path parameters exceeding path domain were discarded!')
                tau(tauExceeds) = [];
            end%if   
            if isempty(tau)
                tau = [tauL tauU];
            end
            
            % Plot options
            optsMark = {'Color','k', 'Marker','x', 'MarkerSize',8};
            optsTang = {'Color','k', 'LineWidth',0.5};
            
            % Initialize handle to graphics objects
            h = gobjects(1+numel(tau), 2);
            
            % Plot the path and get its axis limtis
            npStatus = get(ax, 'NextPlot');
            [h(1,1),ax] = plotxy(ax, obj, [], opts{:});
            xLimits = xlim;
            yLimits = ylim;
            
            % Add the tangents and the corresponding path waypoints
            set(ax, 'NextPlot','add');
            [x,y,~,head] = obj.eval(tau);
            for i = 1:numel(x)
                xi = x(i);
                yi = y(i);
                hi = head(i);
                
                % Marker of tangent point
                h(i+1,1) = plot(xi, yi, optsMark{:});
                
                % Length of tangent
                [r1,r2] = scaleTangentToAxis(xLimits, yLimits, [xi,yi], hi);
                
                % Start/end point of tangent
                P01  = [...
                    xi + [r2 r1]*cos(hi); ...
                    yi + [r2 r1]*sin(hi)];
                
                % Draw the tangent using N arrows sticked together
                h(i+1,2) = quiver(P01(1), P01(2), diff(P01(1,:)), diff(P01(2,:)), ...
                    optsTang{:});
                
                % Disable legend entries for tangents
                set(get(get(h(i+1, 1), 'Annotation'), 'LegendInformation'), ...
                    'IconDisplayStyle','off');
                set(get(get(h(i+1, 2), 'Annotation'), 'LegendInformation'), ...
                    'IconDisplayStyle','off');
                
            end%for
            
            % Reset to initial value
            set(ax, 'NextPlot',npStatus);
            
            % Set the axis limtis corresponding to waypointss
            axis([xLimits, yLimits]);
            applyPlotxyStyles(ax);
            
            if nargout > 0
                hr = h;
                axr = ax;
            end
            
        end%fcn
    end
    
    methods (Access = private)
        function [Q,d,tau] = closestUniquePointOnPath(obj, X, Y)
        %   For all points (x,y) find the closest unique point Q on the
        %   path, if it exists. Otherwise, return NaN.
        %   
        %   [1] A. M. C. Rezende, V. M. Goncalves and L. C. A. Pimenta,
        %   "Constructive Time-Varying Vector Fields for Robot Navigation,"
        %   in IEEE Transactions on Robotics, vol. 38, no. 2, pp. 852-867,
        %   April 2022, doi: 10.1109/TRO.2021.3093674.
        
            assert(isequal(size(X), size(Y)))
            
            [C0,C1] = obj.termPoints();
            [tau0,tau1] = obj.domain();
            isCircuit = obj.IsCircuit;
            
            % Given the parameterized path C(s). For each point (x,y), we
            % find the path paramter s* such that C(s*):=Q is the closest
            % point on the path (orthogonal projection). Therefore, PQ is
            % the normal vector.
            Q = coder.nullcopy(zeros(numel(X), 2));
            d = coder.nullcopy(zeros(numel(X), 1));
            tau = coder.nullcopy(zeros(numel(X), 1));
            for i = 1:numel(X)
                xi = X(i);
                yi = Y(i);
                
                % Solve Eq. (1)-(3): Find the path parameter/point on path
                % that has the minimum distance to point Pi
                [Qi,~,taui] = obj.pointProjection([xi yi]);
                if isCircuit && (numel(taui) > 1) 
                    % If the path is a circuit, and two or more solutions
                    % are found, check if the first and last solution refer
                    % to the same point. (In case point projection returned
                    % repeated solutions)
                    if isequal([tau0 tau1], [taui(1) taui(end)])
                        Qi(end,:) = [];
                        taui(end) = [];
                    end
                end
                if isempty(taui)
                    % Consider endpoints of path
                    if isCircuit
                        Qi = C0';
                        taui = tau0;
                    else
                        Qi = [C0'; C1'];
                        taui = [tau0; tau1];
                    end
                end
                
                % Check for singleton (unique) solution from a set of
                % solutions
                [dMin,QMin,tauMin] = getSingletonSolution(Qi, xi, yi, taui);
                
                Q(i,:) = QMin;
                d(i) = dMin;
                tau(i) = tauMin;
            end%for
            
        end%fcn
    end
    
    methods (Abstract)
        % APPEND    Append paths.
        %   OBJ = append(OBJ0,OBJ1,...,OBJN) appends paths OBJ to OBJN in
        %   the given order creating path OBJ.
        obj = append(obj0, varargin)
        
        % CART2FRENET    Cartesian point to frenet with respect to path.
        %   SD = CART2FRENET(OBJ,XY,PHIMAX) converts point of interest XY
        %   in cartesian coordinates to frenet coordinates SD with respect
        %   to the path OBJ. Point XY is a two-element vector.
        %
        %   [SD,Q,IDX,TAU,DPHI] = CART2FRENET(___) returns results from
        %   point projection. 
        %
        %    See also FRENET2CART, POINTPROJECTION.
        [sd,Q,idx,tau] = cart2frenet(obj, xy, phiMax)
        
        % DERIVATIVE    Derivative of path.
        %   OBJD = DERIVATIVE(OBJ,N) returns the Nth derivative OBJD of the
        %   path OBJ w.r.t. the path parameter.
        %
        %   Note: The result OBJD represents the tangent of the path OBJ,
        %   that can be queried for a given value of the path parameter via
        %   eval(). As a consequence, OBJD is not meaningful in terms of a
        %   path.
        %
        %   See also EVAL.
        objD = derivative(obj, n)
        
        % DOMAIN    Domain of the path.
        %   [TAUL,TAUU] = DOMAIN(OBJ) returns the lower and upper domain
        %   value TAUL and TAUU respectively. For empty paths NaNs are
        %   returned.
        [tauL,tauU] = domain(obj)
        
        % EVAL  Evaluate path at path parameters.
        %   [X,Y,TAUO,HEAD,CURV,CURVDS] = EVAL(OBJ,TAU) evaluates the path
        %   at path parameter TAU to return the corresponding waypoints
        %   (X,Y), path parameter TAUO, heading HEAD, curvature CURV and
        %   derivative CURVDS of curvature w.r.t. path length. All return
        %   values
        %    - are of size N-by-1, where N = numel(TAU), 
        %    - are NaN for values of TAU outside the path's domain.
        %
        %   [___] = EVAL(OBJ,TAU,EXTRAP) performs extrapolation for values
        %   of TAU outside the path's domain if EXTRAP evaluates to true.
        %   
        %   [___] = EVAL(OBJ) evaluates path OBJ according to subclass
        %   specific implementation. If path obj is empty, thenall return
        %   values have size 0-by-1.
        %   
        [x,y,tau,head,curv,curvDs] = eval(obj, tau, extrap)
        
        % FINDZEROCURVATURE     Find zero curvatures.
        %   TAU = FINDZEROCURVATURE(OBJ) returns all path parameters TAU
        %   for which the path's curvature vanishes.
        %
        %   TAU = FINDZEROCURVATURE(OBJ,THS) returns all path parameters
        %   TAU for which the path's curvature is within (-THS,THS).
        tau = findZeroCurvature(obj, ths)
        
        % FRENET2CART    Frenet point to cartesian with respect to path.
        %   XY = FRENET2CART(OBJ,SD) converts points of interest SD in
        %   frenet coordinates to cartesian coordinates XY with respect to
        %   the path OBJ. Pass SD as matrix of size n-by-2.
        % 
        %   [XY,Q,IDX,TAU] = FRENET2CART(___) returns the corresponding
        %    - cartesian points Q on the path.
        %    - indexes IDX referring to the path segments Q relates to.
        %    - path parameters TAU.
        %
        %   See also CART2FRENET.
        [xy,Q,idx,tau] = frenet2cart(obj, sd)
        
        % INTERSECTLINE     Line intersection.
        %   [XY,TAU,ERRFLAG] = INTERSECTLINE(OBJ,O,PSI) returns the
        %   intersection points XY of path OBJ and the line passing through
        %   O and having the slope PSI, where TAU are the corresponding
        %   path parameters. Flag ERRFLAG is true if no intersection was
        %   found and false otherwise.
        [xy,tau,errFlag] = intersectLine(obj, O, psi)
        
        % INTERSECTCIRCLE     Circle intersection.
        %   [XY,TAU,ERRFLAG] = INTERSECTCIRCLE(OBJ,C,R) returns the
        %   intersection points XY of path OBJ and the circle with center C
        %   and radius R, where TAU are the corresponding path parameters.
        %   Flag ERRFLAG is true if no intersection was found and false
        %   otherwise.
        [xy,tau,errFlag] = intersectCircle(obj, C, r)
        
        % POINTPROJECTION    Point projection.
        %   Q = POINTPROJECTION(OBJ,POI,PHIMAX) returns the orthogonal
        %   projection Q of point of interest POI onto the path OBJ. Point
        %   POI is a two-element vector and Q is of size N-by-2. If the
        %   path representation does not allow for analytical solutions,
        %   only solutions with an angular error DPHI < PHIMAX are
        %   returned.
        %
        %   [Q,IDX,TAU,DPHI] = POINTPROJECTION(___) also returns the path
        %   segment IDX, path parameter TAU related to Q, and the angular
        %   deviation DPHI = pi/2 - psi >= 0 of the point projection angle
        %   psi from the true angle pi/2 in radian.
        %
        %   Multiple solutions are concatenated vertically, i.e. Q, IDX and
        %   TAU have as many rows as solutions are found.
        [Q,idx,tau,dphi] = pointProjection(obj, poi, phiMax)
        
        % NUMEL     Number of path segments.
        %   N = NUMEL(OBJ) returns the number N of path segments.
        N = numel(obj)
        
        % RESTRICT  Restrict domain.
        %   OBJ = RESTRICT(OBJ,TAU0,TAU1) returns the path with restricted
        %   domain to the interval [TAU0, TAU1]. If TAU0/TAU1 exceeds the
        %   lower/upper domain or are empty, the respective bound is kept.
        obj = restrict(obj, tau0, tau1)
        
        % REVERSE   Reverse path.
        %   OBJ = REVERSE(OBJ) reverses the order of path elements.
        obj = reverse(obj)
        
        % ROTATE    Rotate path.
        %   OBJ = ROTATE(OBJ,PHI) rotates the path by an angle PHI counter
        %   clockwise.
        %
        %   OBJ = ROTATE(OBJ) rotates the path by the negative of it's
        %   initial slope.
        obj = rotate(obj, phi)
        
        % S2TAU     Path length to path parameter.
        %   TAU = S2TAU(OBJ, S) converts the path lengths S to the path
        %   parameters TAU, such that the path OBJ, evaluated at TAU has
        %   length S.
        %
        %   [___,IDX] = S2TAU(___) also returns the index IDX of the
        %   corresponding path segment.
        %
        %   Input S can be of any size and can exceed [0,L], where L is the
        %   path length. In this case, TAU is linearly extrapolated and IDX
        %   is saturated to [0,N], where N is the number of path segments.
        [tau,idx] = s2tau(obj, s)
        
        % SELECT    Select path elements.
        %   OBJ = SELECT(OBJ,IDXS) selects the path elements IDXS of path
        %   OBJ, where IDXS can be either an array of indexes or a logical
        %   array.
        obj = select(obj, idxs)
        
        % SHIFT     Shift path.
        %   OBJ = SHIFT(OBJ,P) shifts the path by P where P is a
        %   two-element vector.
        %
        %   OBJ = SHIFT(OBJ) shifts the path so that its initial point is
        %   at (0,0).
        obj = shift(obj, P)
        
        % TERMPOINTS  Get terminal points.
        %   [P0,P1] = TERMPOINTS(OBJ) returns the terminal points P0
        %   (initial point) and P1 (end point) of size 2-by-1. For empty
        %   paths NaNs are returned.
        [P0,P1] = termPoints(obj)
    end
      
    methods (Static)
        function obj = ll2Path(lat, lon) %#ok<STOUT,INUSD>
        % LL2PATH    Path object from LAT/LON coordinates.
        %   OBJ = <ClassName>.LL2PATH(LAT, LON) instantiates the path OBJ
        %   from latitude/longitude data.
            error('Not implemented!')
        end%fcn
        
        function obj = pp2Path(pp, varargin) %#ok<STOUT,INUSD>
        % PP2PATH    Path object from piecewise polynomial.
        %   OBJ = <ClassName>.PP2PATH(PP,VARARGIN) instantiates the path
        %   OBJ from piecewise polynomial struct PP.
        %   
        %   See also MKPP.
            error('Not implemented!')
        end%fcn
        
        function obj = circle(r, phi01, N) %#ok<STOUT,INUSD>
        %CIRCLE     Create circle.
        %   OBJ = <ClassName>.CIRCLE(R) creates a path object OBJ
        %   describing a circle of radius R.
        %   
        %   OBJ = <ClassName>.CIRCLE(R, PHI01) sets the initial and final
        %   angle to PHI01(1) and PHI01(2) respectively. Default value is
        %   [0, 2*pi].
        %
        %   OBJ = <ClassName>.CIRCLE(R, PHI01, N) creates a circle using N
        %   path segments.
            error('Not implemented!')
        end%fcn
        
        function OBJ = straight(P0, P1) %#ok<STOUT,INUSD>
        % STRAIGHT  Construct straight path object.
        %   OBJ = <ClassName>.STRAIGHT(P0,P1) creates a straight path from
        %   point P0 to P1.
            error('Not implemented!')
        end%fcn
        
        function obj = xy2Path(x, y) %#ok<STOUT,INUSD>
        % XY2PATH    Path object from cartesian coordinates.
        %   OBJ = <ClassName>.XY2PATH(X,Y) instantiates the path OBJ
        %   from x/y data.
            error('Not implemented!')
        end%fcn
        
        function c = getBusDef(varargin) %#ok<STOUT>
        % GETBUSDEF     Return bus information.
        %   C = GETBUSDEF(VARARGIN) returns the cell array C containing bus
        %   information. 
        %
        %   See also Simulink.Bus.cellToObject.
            error('Not implemented!')
        end%fcn
    end
    
end%class
