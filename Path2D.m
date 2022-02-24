classdef (InferiorClasses = {?matlab.graphics.axis.Axes}) Path2D
%PATH2D        Represent 2D paths.
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
%   eval - Evaluate path at path parameter.
%   frenet2cart - Convert frenet point to cartesian coordinates.
%   getDomain - Get the domain of the path.
%   getPathLengths - Get lengths of path segments.
%   intersectLine - Line intersection.
%   intersectCircle - Circle intersection.
%   isempty - Check if path is empty.
%   length - Get path length.
%   numel - Number of path elements.
%   pointProjection - Point projection on path.
%   reverse - Reverse path.
%   rotate - Rotate path.
%   select - Select path segments.
%   shift - Shift path.
%   termPoints - Get terminal points.
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
%
%   See also .

    properties
        % ISCIRCUIT - Logical indicating if path is a circuit.
        IsCircuit = false
    end
    
    
    
    methods
        
        function obj = Path2D()
        %PATH2D        Construct a PATH2D class instance.
            
        end%Constructor
        
        function [hr,axr] = plot(varargin)
        %PLOT   Plot the path.
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
            
            if nargout > 0
                hr = h;
                axr = ax;
            end
            
        end%fcn
        
        function [hr,axr] = plotG2(varargin)
        %PLOTG2     Plot path, heading and curvature.
        %
        %    For Syntax see:
        %    See also PATH2D/PLOT.
            
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
            h(:,1) = plotxy(ax(1), obj, dtau, opts{:});
            npStatus = get(ax(2:3), 'NextPlot');
%             set(ax(2:3), 'NextPlot','replace');
            for i = 1:N
                if i == 2
                    set(ax(2:3), 'NextPlot','add');
                end
                
                [~,~,head,curv] = eval(obj(i));
                s = getPathLengths(obj(i));
                h(i,2) = plot(ax(2), s, head, opts{:});
                h(i,3) = plot(ax(3), s, curv, opts{:});
            end%for
            set(ax(2:3), {'NextPlot'},npStatus);
            
            grid(ax(2), 'on');
            ylabel(ax(2), 'Heading (rad)');
            grid(ax(3), 'on');
            xlabel(ax(3), 'Length (m)');
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
        
        function [hr,axr] = plottangent(obj, tau, varargin) 
        %PLOTTANGENT    Plot path and tangents.
        %    PLOTTANGENT(OBJ,TAU) plots the path (x,y) and the tangents at
        %    the path paraemters TAU and highlights the path coordinates at
        %    TAU.
        %    
        %    PLOTTANGENT(...,NAME,VALUE) specifies line properties using
        %    one or more Name,Value pair arguments.
        %    
        %    [H,AX] = PLOTTANGENT(...) returns a handle array H to
        %    lineseries objects and the axes handle AX. Here, H(1,1) is the
        %    path-handle, H(i+1,1) and H(i+1,2) the marker- and the
        %    tangent-handle of TAU(i) respectively.
        %    
        %    See also PATH2D/PLOT, PATH2D/PLOTG2.
            
            %%% Handle input arguments
            assert(isnumeric(tau) && ~isempty(tau), ...
                'Input argument TAU must be a non-empty numeric vector!');
            
            % Apply plot options if unspecified
            if nargin < 3
                opts = {};
            else
                opts = varargin;
            end%if
            optsPath = {'.', 'LineWidth',1};
            optsMark = {'Color','k', 'Marker','x', 'MarkerSize',8};
            optsTang = {'--', 'Color','k', 'LineWidth',0.5};
            
            % Initialize handle to graphics objects
            h = gobjects(1+numel(tau), 2);
            
            % Plot the waypoints and get corresponding axis limtis
            [h(1,1),ax] = plotxy([], obj, [], optsPath{:}, opts{:});
            xLimits = xlim;
            yLimits = ylim;
            
            % Check if some path parameters are out of range
            [tauL,tauU] = getDomain(obj);
            tauExceeds = (tau > tauU) | (tau < tauL);
            if any(tauExceeds)
                warning('PATH2D:plottangent:tauOutOfRange', ...
                    'Path parameters exceeding domain of path were discarded!')
                tau(tauExceeds) = [];
            end%if
            
            % Plot the tangents and the corresponding waypoints
            [x,y,head] = eval(obj, tau);
            hold on
            for i = 1:numel(x)
                xi = x(i);
                yi = y(i);
                hi = head(i);
                
                % Marker of tangent point
                h(i+1,1) = plot(xi, yi, optsMark{:}, opts{:});
                
                % Length of tangent
                [r1,r2] = scaleTangentToAxis(xLimits, yLimits, [xi,yi], hi);
                
                % Start/end point of tangent
                Pstart  = [xi + r2*cos(hi); yi + r2*sin(hi)];
                Pstop   = [xi + r1*cos(hi); yi + r1*sin(hi)];
                
                % Draw the tangent using N arrows sticked together
                N = 25;
                xq = linspace(Pstart(1), Pstop(1), N);
                yq = linspace(Pstart(2), Pstop(2), N);
                % quiver would draw one arrow at every point of xq/yq of
                % length uq/vq; since the last element of xq/yq is the end
                % point, no arrow has to be drawn there
                xq = xq(1:end-1);
                yq = yq(1:end-1);
                uq = ones(size(xq))*(Pstop(1)-Pstart(1))/N;
                vq = ones(size(yq))*(Pstop(2)-Pstart(2))/N;
                scale = 0;
                h(i+1,2) = quiver(xq, yq, uq, vq, scale, optsTang{:}, opts{:});
                
                % Disable legend entries for tangents
                set(get(get(h(i+1, 1), 'Annotation'), 'LegendInformation'), ...
                    'IconDisplayStyle','off');
                set(get(get(h(i+1, 2), 'Annotation'), 'LegendInformation'), ...
                    'IconDisplayStyle','off');
                
            end%for
            hold off
            
            % Set the axis limtis corresponding to waypointss
            axis([xLimits, yLimits]);
            
            if nargout > 0
                hr = h;
                axr = ax;
            end
            
        end%fcn
        
    end%methods
    
    
    methods (Abstract)
        
        % APPEND    Append paths.
        %   OBJ = append(OBJ0,OOBJ1,...,OBJN) appends paths OBJ to OBJN in
        %   the given order creating path OBJ.
        obj = append(obj0, varargin)
        
        % CART2FRENET    Cartesian point to frenet with respect to path.
        %   SD = CART2FRENET(OBJ,XY) converts point of interest XY in
        %   cartesian coordinates to frenet coordinates SD with respect to
        %   the path OBJ. Point XY is a two-element vector .
        %
        %   [SD,Q,IDX,LAMBDA] = CART2FRENET(___) also return 
        %    - The cartesian point Q that corresponds to the frenet point SD.
        %    - An index IDX indicating the path segment Q relates to.
        %    - The path parameter LAMBDA.
        [sd,Q,idx,lambda] = cart2frenet(obj, xy)
        
        % EVAL  Evaluate path at path parameters.
        %   [X,Y] = EVAL(OBJ,TAU) evaluates analytical path definition at
        %   path parameters TAU to return the corresponding waypoints
        %   (X,Y).
        %
        %   [__,HEAD,CURV] = EVAL(___) also returns heading HEAD and
        %   curvature CURV.
        %
        %   [___] = EVAL(OBJ) subclass specific implementation.
        [x,y,head,curv] = eval(obj, tau)
        
        % FRENET2CART    Frenet point to cartesian with respect to path.
        %   XY = FRENET2CART(OBJ,SD) converts points SD in frenet
        %   coordinates to cartesian coordinates XY with respect to the
        %   path OBJ. Matrix SD is of size n-by-2.
        % 
        %   [XY,Q,IDX] = FRENET2CART(___) returns 
        %    - The Point Q that corresponds to the frenet points SD.
        %    - An index IDX referring to the path segment Q relates to.
        [xy,Q,idx] = frenet2cart(obj, sd)
        
        % GETDOMAIN     Get the domain of the path.
        %   [TAUL,TAUU] = GETDOMAIN(OBJ) returns the lower and upper domain
        %   value TAUL and TAUU respectively.
        [tauL,tauU] = getDomain(obj);
        
        % GETPATHLENGTHS    Get path segment lengths.
        %   S = GETPATHLENGTHS(OBJ) get the lengths S of each segment of
        %   path OBJ.
        s = getPathLengths(obj)
        
        % INTERSECTLINE     Line intersection.
        %   [XY,TAU,ERRFLAG] = INTERSECTLINE(OBJ, O, PSI) returns the
        %   intersection point XY of path OBJ and the line passing through
        %   O and having the slope PSI, where TAU is the corresponding path
        %   parameter. Flag ERRFLAG is true if no intersection was found
        %   and false otherwise.
        [xy,tau,errFlag] = intersectLine(obj, O, psi)
        
        % INTERSECTCIRCLE     Circle intersection.
        %   [XY,TAU,ERRFLAG] = INTERSECTLINE(OBJ, C, R) returns the
        %   intersection point XY of path OBJ and the circle with center C
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
        %   S = LENGTH(OBJ) returns the length S of the path OBJ.
        s = length(obj)
        
        % POINTPROJECTION    Point projection.
        %   Q = POINTPROJECTION(OBJ,POI) returns the orthogonal projection
        %   Q of point of interest POI onto the path OBJ.
        %
        %   [Q,IDX,LAMBDA] = POINTPROJECTION(OBJ,POI) also returns the path
        %   segment IDX and path path parameter LAMBDA related to Q.
        [Q,idx,lambda] = pointProjection(obj, poi)
            
        % NUMEL     Number of path elements.
        %   N = NUMEL(OBJ) returns the number of path elements, e.g. 
        %    - The number of waypoints for polygon path.
        %    - The number of segments for a spline path.
        N = numel(obj)
        
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
        
        % SELECT    Select path segments.
        %   OBJ = SELECT(OBJ,IDXS) selects the path elements IDXS of path
        %   OBJ, where IDXS can be either an array of indexes or a logical
        %   array.
        obj = select(obj, idxs)
        
        % SHIFT     Shift path.
        %   OBJ = SHIFTBY(OBJ,P) shifts the path by P where P is a
        %   two-element vector.
        obj = shift(obj, P)
        
        % TERMPOINTS  Get terminal points.
        %   [XY0,XY1] = TERMPOINTS(OBJ) returns the terminal points
        %   (XY0,XY1).
        [xy0,xy1] = termPoints(obj)
        
    end%methods
    
    
    methods (Abstract, Static)
        
        % LL2PATH    Path object from LAT/LON coordinates.
        %   OBJ = LL2PATH(LAT, LON)
        obj = ll2Path(lat, lon)
        
        % PP2PATH    Path object from piecewise polynomial.
        %    OBJ = PP2PATH(PP,TAU)
        %
        %    See also MKPP.
        obj = pp2Path(pp, tay, polyDeg)
        
        % XY2PATH    Path object from cartesian coordinates.
        %    OBJ = XY2PATH(X,Y)
        obj = xy2Path(x, y)
        
    end%methods
    
end%class
