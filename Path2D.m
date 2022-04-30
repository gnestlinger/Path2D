classdef (InferiorClasses = {?matlab.graphics.axis.Axes}) Path2D
%PATH2D     Represent 2D paths.
%   This class is abstract and can not be instantiated!
%   
%   Represent paths in the planar x/y coordinate frame utilizing an
%   analytical representation [x(tau); y(tau)]. It provides path operations
%   that can be especially usefull for automated driving and robotics
%   applications.
% 
%   Path2D properties:
%   IsCircuit - Indicates if path is a circuit.
% 
%   Path2D methods:
%   append - Append paths.
%   cart2frenet - Convert cartesian point to frenet coordinates.
%   domain - Domain of the path.
%   eval - Evaluate path at path parameter.
%   frenet2cart - Convert frenet point to cartesian coordinates.
%   getPathLengths - Get lengths of path segments.
%   intersectLine - Line intersection.
%   intersectCircle - Circle intersection.
%   isempty - Check if path is empty.
%   length - Path length.
%   numel - Number of path elements.
%   pointProjection - Point projection on path.
%   restrict - Restrict path domain.
%   reverse - Reverse path.
%   rotate - Rotate path.
%   select - Select path segments.
%   shift - Shift path.
%   termPoints - Terminal points.
% 
%   Path2D visualization methods:
%   plot - Plot the path.
%   plotG2 - Plot path, heading and curvature.
%   plottangent - Plot path and tangents.
% 
%   Path2D static methods:
%   empty   - Instantiate empty path.
%   ll2Path - Construct path from latitude/longitude coordinates.
%   pp2Path - Construct path from piecewise polynomial.
%   xy2Path - Construct path from cartesian coordinates.
%
%   See also PolygonPath.
    
    properties
        % ISCIRCUIT - Logical indicating if path is a circuit.
        IsCircuit = false
    end
    
    
    
    methods
        
        function obj = Path2D()
        %PATH2D        Construct a PATH2D class instance.
            
        end%Constructor
        
        function [hr,axr] = plot(varargin)
        %PLOT   Plot path.
        %    PLOT(OBJ) plots the path OBJ in terms of x over y.
        %
        %    PLOT(OBJ,DTAU) specify path parameter increment to be plotted.
        %    
        %    PLOT(OBJ,S) additionally applies the line specification S.
        %
        %    PLOT(OBJ,DTAU,S) specify DTAU before any line specification.
        %
        %    PLOT(...,NAME,VALUE) specifies line properties using one or
        %    more Name,Value pair arguments.
        %
        %    PLOT(AX,...) plots into the axes with handle AX.
        %    
        %    [H,AX] = PLOT(...) returns the handle H to lineseries objects
        %    and axes handle AX.
        %    
        %    The line specification S is a character string supported by the
        %    standard PLOT command. For example
        %        PLOT(OBJ, 'LineWidth',2, 'Color',[.6 0 0]) 
        %    will plot a dark red line using a line width of 2 points.
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
        %    PLOTTANGENT(OBJ,TAU) plots the path and tangents at the path
        %    parameters TAU and highlights the path coordinates at TAU. If
        %    TAU is empty, tangents are plotted for the path's terminal
        %    points.
        %    
        %    [H,AX] = PLOTTANGENT(...) returns a handle array H to
        %    lineseries objects and the axes handle AX. Here, H(1,1) is the
        %    path-handle, H(i+1,1) and H(i+1,2) the marker- and the
        %    tangent-handle of TAU(i) respectively.
        %    
        %    For Syntax see also PATH2D/PLOT.
        %
        %    See also PATH2D/PLOT, PATH2D/PLOTG2.
            
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
        
        function tau = sampleDomain(obj, arg)
        %SAMPLEDOMAIN   Sample domain of path.
        %
        %   TAU = SAMPLEDOMAIN(OBJ,ARG) returns the path parameter TAU
        %   sampled over the domain of path OBJ depending on the class of
        %   ARG: if ARG is 
        %     - double or single it is iterpreted as a step increment
        %     - uintx it is interpreted as the number of samples.
        %   
        %   In both cases, TAU contains the domain's terminal points.
        
            assert(isscalar(arg));
            
            [tau0,tau1] = obj.domain();
            switch class(arg)
                case {'double','single'} % Interpret as step increment
                    [tau0,tau1] = obj.domain();
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
        
    end%methods
    
    
    methods (Abstract)
        
        % APPEND    Append paths.
        %   OBJ = append(OBJ0,OBJ1,...,OBJN) appends paths OBJ to OBJN in
        %   the given order creating path OBJ.
        obj = append(obj0, varargin)
        
        % CART2FRENET    Cartesian point to frenet with respect to path.
        %   SD = CART2FRENET(OBJ,XY) converts point of interest XY in
        %   cartesian coordinates to frenet coordinates SD with respect to
        %   the path OBJ. Point XY is a two-element vector.
        %
        %   [SD,Q,IDX,TAU] = CART2FRENET(___) also return 
        %    - The cartesian point Q that corresponds to the frenet point SD.
        %    - An index IDX indicating the path segment Q relates to.
        %    - The path parameter TAU.
        [sd,Q,idx,tau] = cart2frenet(obj, xy)
        
        % DOMAIN    Domain of the path.
        %   [TAUL,TAUU] = DOMAIN(OBJ) returns the lower and upper domain
        %   value TAUL and TAUU respectively.
        [tauL,tauU] = domain(obj);
        
        % EVAL  Evaluate path at path parameters.
        %   [X,Y,TAU,HEAD,CURV] = EVAL(OBJ,TAU) evaluates analytical path
        %   definition at path parameters TAU to return the corresponding
        %   waypoints (X,Y), path parameter TAU, heading HEAD and curvature
        %   CURV. All return values are of size numel(TAU)-by-1.
        %
        %   [___] = EVAL(OBJ) evaluates path OBJ according to subclass
        %   specific implementation.
        %
        %   Note: Evaluation outside the domain will return NaN!
        [x,y,tau,head,curv] = eval(obj, tau)
        
        % FRENET2CART    Frenet point to cartesian with respect to path.
        %   XY = FRENET2CART(OBJ,SD) converts points SD in frenet
        %   coordinates to cartesian coordinates XY with respect to the
        %   path OBJ. Matrix SD is of size n-by-2.
        % 
        %   [XY,Q,IDX] = FRENET2CART(___) returns 
        %    - The Point Q that corresponds to the frenet points SD.
        %    - An index IDX referring to the path segment Q relates to.
        [xy,Q,idx] = frenet2cart(obj, sd)
        
        % GETPATHLENGTHS    Get path segment lengths.
        %   S = GETPATHLENGTHS(OBJ) get the lengths S of each segment of
        %   path OBJ.
        s = getPathLengths(obj)
        
        % INTERSECTLINE     Line intersection.
        %   [XY,TAU,ERRFLAG] = INTERSECTLINE(OBJ,O,PSI) returns the
        %   intersection point XY of path OBJ and the line passing through
        %   O and having the slope PSI, where TAU is the corresponding path
        %   parameter. Flag ERRFLAG is true if no intersection was found
        %   and false otherwise.
        [xy,tau,errFlag] = intersectLine(obj, O, psi)
        
        % INTERSECTCIRCLE     Circle intersection.
        %   [XY,TAU,ERRFLAG] = INTERSECTCIRCLE(OBJ,C,R) returns the
        %   intersection points XY of path OBJ and the circle with center C
        %   and radius R, where TAU is the corresponding path parameter.
        %   Flag ERRFLAG is true if no intersection was found and false
        %   otherwise.
        [xy,tau,errFlag] = intersectCircle(obj, C, r)
        
        % ISEMPTY   Check if path is empty
        %   FLAG = ISEMPTY(OBJ) returns true if the path contains no path
        %   elements, i.e. numel(OBJ) == 0, and false otherwise.
        %
        %   See also NUMEL.
        flag = isempty(obj)
        
        % LENGTH    Path length.
        %   S = LENGTH(OBJ) returns the arc length S of the path OBJ.
        s = length(obj)
        
        % POINTPROJECTION    Point projection.
        %   Q = POINTPROJECTION(OBJ,POI) returns the orthogonal projection
        %   Q of point of interest POI onto the path OBJ. Point POI is a
        %   two-element vector and Q is of size N-by-2.
        %
        %   [Q,IDX,TAU] = POINTPROJECTION(OBJ,POI) also returns the path
        %   segment IDX and path parameter TAU related to Q.
        %
        %   Multiple solutions are concatenated vertically, i.e. Q, IDX and
        %   TAU have as many rows as solutions are found.
        [Q,idx,tau] = pointProjection(obj, poi)
            
        % NUMEL     Number of path elements.
        %   N = NUMEL(OBJ) returns the number of path elements, e.g. 
        %    - The number of waypoints for polygon path.
        %    - The number of segments for a spline path.
        N = numel(obj)
        
        % RESTRICT  Restrict domain.
        %   OBJ = RESTRICT(OBJ,TAU0,TAUF) returns the path with restricted
        %   domain to the interval [TAU0, TAUF]. If TAU0/TAUF exceeds the
        %   lower/upper domain or are empty, the respective bound is kept.
        obj = restrict(obj, tau0, tauF)
        
        % REVERSE   Reverse path.
        %   OBJ = REVERSE(OBJ) reverses the path's direction.
        obj = reverse(obj)
        
        % ROTATE    Rotate path.
        %   OBJ = ROTATE(OBJ,PHI) rotates the path by an angle PHI counter
        %   clockwise.
        %
        %   OBJ = ROTATE(OBJ) rotates the path by the negative of it's
        %   initial slope.
        obj = rotate(obj, phi)
        
        % SELECT    Select path elements.
        %   OBJ = SELECT(OBJ,IDXS) selects the path elements IDXS of path
        %   OBJ, where IDXS can be either an array of indexes or a logical
        %   array.
        obj = select(obj, idxs)
        
        % SHIFT     Shift path.
        %   OBJ = SHIFTBY(OBJ,P) shifts the path by P where P is a
        %   two-element vector.
        obj = shift(obj, P)
        
        % TERMPOINTS  Get terminal points.
        %   [P0,P1] = TERMPOINTS(OBJ) returns the terminal points
        %   P0 (initial point) and P1 (end point).
        [P0,P1] = termPoints(obj)
        
    end%methods
    
    
    methods (Abstract, Static)
        
        % EMPTY     Instantiate empty path.
        %   OBJ = <ClassName>.EMPTY() instantiates the empty path OBJ for
        %   which numel(OBJ) = 0 holds.
        obj = empty()
        
        % LL2PATH    Path object from LAT/LON coordinates.
        %   OBJ = <ClassName>.LL2PATH(LAT, LON) instantiates the path OBJ
        %   from latitude/longitude data.
        obj = ll2Path(lat, lon)
        
        % PP2PATH    Path object from piecewise polynomial.
        %    OBJ = <ClassName>.PP2PATH(PP,TAU) instantiates the path OBJ
        %   from piecewise polynomial struct PP.
        %
        %    See also MKPP.
        obj = pp2Path(pp, tay, polyDeg)
        
        % XY2PATH    Path object from cartesian coordinates.
        %    OBJ = <ClassName>.XY2PATH(X,Y) instantiates the path OBJ
        %   from x/y data.
        obj = xy2Path(x, y)
        
    end%methods
    
end%class
