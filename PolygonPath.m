classdef (InferiorClasses = {?matlab.graphics.axis.Axes}) PolygonPath < Path2D
%POLYGONPATH    Polygon path.
%   Path representation using polygonal chain assumin linear interpolation.
% 
%   PolygonPath properties:
%   x - Cartesian x-coordinate.
%   y - Cartesian y-coordinate.
%   head - Heading in radians.
%   curv - Curvature in 1/m.
% 
%   PolygonPath methods:
%   PolygonPath - Constructor.
%   fitCircle - Fit circle to path.
%   fitStraight - Fit straight line to path.
%   interp - Interpolate path.
%   perpendicularDistance - Distance of path waypoints to line.
%   rdp - Ramer-Douglas-Peucker point reduction.
%   See superclass for more methods.
% 
%   PolygonPath static methods:
%   See superclasses.
% 
%   See also PATH2D.

%#ok<*PROP>
%#ok<*EVLC> % Using eval and evalc with two arguments is not recommended ..
%#ok<*PROPLC> % There is a property named xyz. Maybe this is a reference to it?

    properties
        x = zeros(100, 1)
        y = zeros(100, 1)
        head = zeros(100, 1)
        curv = zeros(100, 1)
    end
    
    
    
    methods
        
        function obj = PolygonPath(x, y, head, curv, isCircuit)
        %POLYGONPATH    Create polygon path object.
        %   OBJ = POLYGONPATH(X,Y,HEAD,CURV) create polygon path OBJ with
        %   points (X,Y), heading HEAD in radians and curvature CURV in
        %   1/m.
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
            
            if nargin < 5
                obj.IsCircuit = false;
            else
                obj.IsCircuit = isCircuit;
            end%if
            
        end%Constructor
        
        function obj = append(obj, obj2)
            obj.x        = [obj.x; obj2.x];
            obj.y        = [obj.y; obj2.y];
            obj.head    = [obj.head; obj2.head];
            obj.curv    = [obj.curv; obj2.curv];
        end%fcn
        
        function [sd,Q,idx,lambda] = cart2frenet(obj, xy, doPlot)
            
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
        %
        %    See also PATH2D/CART2FRENET.

            [Q,idx,lambda] = pointProjection(obj, xy);
            
            % Get the orientation vector related with Q to calculate the
            % sign of distance from point of interest to path
            idx0 = idx;
            if idx0 == numel(obj)
                idx1 = idx0;
                idx0 = idx0 - 1;
            else
                idx1 = idx0 + 1;
            end
            P0 = [obj.x(idx0); obj.y(idx0)];
            P1 = [obj.x(idx1); obj.y(idx1)];
            u = P1 - P0;
            
            % Get sign via z-component of cross product u x (Q-poi)
            qp = Q - xy;
            signD = sign(u(1)*qp(2) - u(2)*qp(1));
            
            sd = [...
                sum(hypot(diff([obj.x(1:idx); Q(1)]), diff([obj.y(1:idx); Q(2)])));
                signD * norm(qp, 2)
                ];
            
            if nargin < 3
                doPlot = false;
            end
            if doPlot
                plot(obj.x, obj.y, 'Marker','.', 'MarkerSize',8, 'DisplayName','RefPath');
                hold on
                plot(obj.x(1), obj.y(1), 'g.', 'MarkerSize',18, 'DisplayName','Initial point');
                plot(xy(1), xy(2), 'o', 'DisplayName','PoI');
                plot([P0(1),P1(1)], [P0(2),P1(2)], 'r.', 'MarkerSize',8, 'DisplayName','P0/P1');
                plot(Q(1), Q(2), 'kx', 'DisplayName','Q');
                plot([xy(1), Q(1)], [xy(2), Q(2)]);
                axis equal
                grid on
                hold off
                legend show
            end%if
            
        end%fcn
        
        function [x,y,head,curv] = eval(obj, tau)
            
            if nargin < 2
                x = obj.x;
                y = obj.y;
                head = obj.head;
                curv = obj.curv;
            else
                sAct = getPathLengths(obj);
                xyhc = interp1(sAct, [obj.x,obj.y,obj.head,obj.curv], tau);
                x = xyhc(:,1);
                y = xyhc(:,2);
                head = xyhc(:,3);
                curv = xyhc(:,4);
            end
            
        end%fcn
        
        function [objs,e,k,d] = fitStraight(obj, doPlot)
        %FITSTRAIGHT    Fit a straight line to PolygonPath.
        %   [OBJS,E,K,D] = FITSTRAIGHT(OBJ) fits a straight line OBJS
        %   with slope K and offset D to a set of given waypoints (OBJ.X,
        %   OBJ.Y) minimizing the error E.
        %   
        %   [___] = FITSTRAIGHT(OBJ,DOPLOT) allows to enable the plot for
        %   checking the fitting result visually.
        %   
        %   Parameters C and D model the fitted line according to
        %     y_fit = C*OBJ.X + D
        %    
        %   Minimization is performed in the least-squares sense minimizing
        %   the sum of squared errors:
        %     SUM[(Y(i)-C*X(i) - D)^2]
        %      i
        %    
        %   See also POLYGONPATH/FITCIRCLE.
                    
            if nargin < 2
                doPlot = false;
            end%if
            
            % Extract x/y data
            objX = obj.x;
            objY = obj.y;
            N = numel(obj);
            
            % Create (overdetermined) system of equations
            A = [objX, ones(N, 1)];
            b = obj.y;
            
            % Solve system of equations: A*[c;d] = b, where y = c*x+d
            cd = (A'*A)\A'*b;
            k = cd(1);
            d = cd(2);
            
            % The fitted y coordinates
            ys = objX*k+d; 
            
            % Create POLYGONPATH object
            objs = PolygonPath(objX, ys, ...
                atan2(ys(2)-ys(1), objX(2)-objX(1))*ones(N,1), ...
                zeros(N,1));
            
            % Calculate error
            e = 1/numel(objY)*sum((objY - objs.y).^2);
            
            % Plot if requested
            if doPlot
                plot(obj, 'r', 'Marker','.', 'DisplayName','PolygonPath');
                hold on
                plot(objs, 'b', 'Marker','.', 'DisplayName','Straight');
                hold off
                legend off; legend show; % to update the legend
            end%if
            
            objs = select(objs, [1,N]);
            
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
            
            if nargin < 3
                doPlot = false;
            end%if
            
            % Extract x/y data
            xsub = obj.x;
            ysub = obj.y;
            
            method = 'Kasa';
            switch method
                case 'Kasa'
                    [xc,yc,R,e] = fitCircle_Kasa(xsub, ysub);
                otherwise
                    % 
            end%switch
            
            % Create POLYGONPATH object
            phi = linspace(0, 2*pi, N);
            objc = PolygonPath(...
                R*cos(phi) + xc, ...
                R*sin(phi) + yc, ...
                2*R*phi, ...
                1/R*ones(N,1), ...
                phi + pi/2);
            
            % Plot if requested
            if doPlot
                plot(obj, 'b-', 'MarkerSize',10, 'DisplayName','PolygonPath');
                hold on
                plot(objc, 'Color','k', 'DisplayName','Circle');
                hold off
                legend off; legend show; % to update the legend
            end%if
            
        end%fcn
        
        function [xy,Q,idx] = frenet2cart(obj, sd, doPlot)
            
        % 
        %   EXAMPLES:
        %   >> s = [-1;sqrt(8);sqrt(8)+sqrt(10);10;sqrt(8)+sqrt(10)+sqrt(41);13];
        %   >> d = ones(size(s));
        %   >> xy = frenet2cart([0 2 5 10], [0 2 3 -1], [s,d; s,-d], true)
        %   xy = 
        %  -1.4142         0
        %   1.2929    2.7071
        %   4.6838    3.9487
        %   8.7554    1.2763
        %  10.6247   -0.2191
        %  11.0980   -0.5978
        %        0   -1.4142
        %   2.7071    1.2929
        %   5.3162    2.0513
        %   7.5060   -0.2855
        %   9.3753   -1.7809
        %   9.8486   -2.1595
        % 
        
            Pxy = [obj.x, obj.y]; % Initial/terminal points per path segment
            u = diff(Pxy, 1, 1); % Orientation vectors/TODO: make use of heading
            assert(size(u, 1) > 0, 'Path must have at least two points!');
            
            % Cumulative length of path segments
            Ps = cumsum(hypot(u(:,1), u(:,2)));
            
            % Normalize orientation vectors to length 1
            u = bsxfun(@rdivide, u, hypot(u(:,1), u(:,2)));
            
            % Get the indexes referring to the path segments according to
            % frenet coordinates s-values
            sEval = sd(:,1);
            idx = zeros(size(sEval), 'uint32');
            % idx2 = bsxfun(@lt, sEval', Ps);
            for i = 1:numel(idx)
                isLessEqual = sEval(i) < Ps;
                if ~any(isLessEqual)
                    idx(i) = numel(Ps);
                else
                    index = find(isLessEqual, 1, 'first');
                    idx(i) = index(1);
                end
            end%for
            
            % The points on the path (i.e. d=0) are given by the segments
            % initial point plus the remaining length along the current
            % segment
            Ps = circshift(Ps, 1);
            Ps(1) = 0;
            Q = Pxy(idx,:) + bsxfun(@times, sEval-Ps(idx), u(idx,:));
            
            % From Q, go D units along normal vector to U
            xy = Q + bsxfun(@times, sd(:,2), [-u(idx,2), u(idx,1)]);
            
            if nargin < 3
                doPlot = false;
            end
            if doPlot
                plot(obj.x, obj.y, 'Marker','.', 'MarkerSize',15, ...
                    'DisplayName','PolygonPath');
                hold on
                plot(xy(:,1), xy(:,2), 'o', 'DisplayName','xy');
                plot(Q(:,1), Q(:,2), 'kx', 'DisplayName','Q');
                axis equal
                grid on
                hold off
                legend show
                title(mfilename)
            end%if
            
        end%fcn
        
        function [tauL,tauU] = getDomain(obj)
            
            tauL = 0;
            tauU = length(obj);
            
        end%fcn
        
        function s = getPathLengths(obj, idx0, idx1)
            
            if nargin < 2
                xtmp = obj.x;
                ytmp = obj.y;
            else
                xtmp = obj.x(idx0:idx1);
                ytmp = obj.y(idx0:idx1);
            end
            s = [0; cumsum(hypot(diff(xtmp), diff(ytmp)))];
            
        end%fcns       
        
        function obj = interp(obj, tau, varargin)
        %INTERP     Interpolate path.
        %   OBJ = INTERP(OBJ,TAU) interpolate path at path parameter TAU.
        %
        %   OBJ = INTERP(__,ARGS) specify interpolation settings via ARGS.
        %
        %   See also INTERP1.
            
            narginchk(2, 5)
            
            % INTERP1 requires at least two samples
            assert(numel(obj) > 1)
            
            sAct = getPathLengths(obj);
            xyhc = interp1(sAct, [obj.x,obj.y,obj.head,obj.curv], tau(:), ...
                varargin{:});
            obj = PolygonPath(xyhc(:,1), xyhc(:,2), xyhc(:,3), xyhc(:,4));
            
        end%fcn
        
        function [xy,tauS,errFlag] = intersectCircle(obj, C, r, doPlot)
            
        %   EXAMPLES:
        %   >> s = PolygonPath.xy2Path([0 0 -3 -2 -4 -3 1 1], [0 1 2 3 4 5 4 0]);
        %   >> intersectCircle(s, [-1 3], 2, true)
            
            if nargin < 4
                doPlot = false;
            end
            
            % Brute force approach: check every path segment
            idxs = (1:numel(obj)-1)';
            x0 = obj.x - C(1);
            dx = diff(x0);
            x0(end) = [];
            y0 =  obj.y - C(2);
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
            
            if isempty(tauLoc)
                xy = zeros(0, 2);
                tauS = zeros(0,1);
                errFlag = true;
            else
                s = getPathLengths(obj);
                tauS = sort(s(segIdx) + (s(segIdx+1)-s(segIdx)).*tauLoc);
                [x,y] = eval(obj, tauS);
                xy = [x,y];
                errFlag = false;
            end
            
            % At most two intersections per path segment!
            assert(size(xy, 1) <= (numel(obj)-1)*2)
            assert(size(xy, 1) == size(tauS, 1))
                
            if doPlot
                [~,ax] = plot(obj, 'Marker','.');
                hold on
                phi = 0:pi/100:2*pi;
                plot(ax, r*cos(phi) + C(1), r*sin(phi) + C(2));
                plot(ax, xy(:,1), xy(:,2), 'ko')
                hold off
            end%if
            
        end%fcn
        
        function [xy,tau,errFlag] = intersectLine(obj, O, psi, doPlot)
            
            if nargin < 4
                doPlot = false;
            end
            
            % Shift by line origin O and rotate so that the line is
            % horizontal
            R = rotmat2D(psi);
            xyPath = [obj.x - O(1), obj.y - O(2)] * R;
            xPath = xyPath(:,1);
            yPath = xyPath(:,2);
            
            % Find indexes where the paths y-component changes sign
            signs = int8(sign(yPath));
            indxs = find(diff(signs, 1));
            if isempty(indxs) % No intersection of path/line
                errFlag = true;
                
                % TODO: Fallbacks
                xy = [0;0];
                tau = 0;
            else
                errFlag = false;

                x = [];
                ind0 = 0;
                indF = 0;
                for i = 1:numel(indxs)
                    % IND0F can not exceed number of path samples since
                    % indexes were obtained using DIFF!
                    ind0F = [indxs(i), indxs(i)+1];
                    x0F = xPath(ind0F);
                    y0F = yPath(ind0F);
                
%                     xTmp = interp1(y0F, x0F, 0, 'linear');
                    xTmp = x0F(1) - y0F(1) * diff(x0F)/diff(y0F);
                    if isempty(x) || (abs(xTmp) < abs(x)) % Smallest value error wins
                        x = xTmp;
                        ind0 = ind0F(1);
                        indF = ind0F(2);
                    end%if
                end%for
                
                % Undo transformation
                xy = R * [x;0] + O(:);
                
                % Path length parameter. Since we assume linear
                % interpolation between waypoints, the local path segment
                % parameter can be computed from x or y
%                 tauLocal = (x - x0F(1))/diff(x0F);
                tauLocal = -y0F(1)/diff(y0F);
                s = getPathLengths(obj);
                tau = s(ind0);
                tau = tau + (s(indF) - tau)*tauLocal;
                
                [x,y] = eval(obj,tau);
                assert(all(abs(xy - [x;y]) < 1e15));
            end%if
            
            if doPlot
                [~,ax] = plot(obj);
                hold on
                
                [r1,r2] = scaleTangentToAxis(xlim, ylim, O, psi);
                Pstart  = [O(1) + r2*cos(psi); O(2) + r2*sin(psi)];
                Pstop   = [O(1) + r1*cos(psi); O(2) + r1*sin(psi)];
                plot(gca, [Pstart(1) Pstop(1)], [Pstart(2) Pstop(2)])
                
                plot(ax, xy(1), xy(2), 'ko')
                hold off
            end%if
            
        end%fcn
        
        function flag = isempty(obj)
            flag = (numel(obj) < 1);
        end%fcn
        
        function s = length(obj, idx0, idx1)
            
            if nargin < 2
                xtmp = obj.x;
                ytmp = obj.y;
            else
                xtmp = obj.x(idx0:idx1);
                ytmp = obj.y(idx0:idx1);
            end
            s = sum(hypot(diff(xtmp), diff(ytmp)));
            
        end%fcn
        
        function n = numel(obj)
            n = numel(obj.x);
        end%fcn
        
        function d = perpendicularDistance(obj, P0, P1, doPlot)
        %PERPENDICULARDISTANCE    Perpendicular distance to line.
        %    D = PERPENDICULARDISTANCE(OBJ,P1,P2) calculate the
        %    perpendicular distance D for all waypoints of OBJ to the line
        %    passing through P1 and P2.
        % 
        %    D = PERPENDICULARDISTANCE(...,DOPLOT) also shows a plot of
        %    results if DOPLOT evaluates to TRUE.
        %
        %    NOTE: The distance for waypoints of OBJ to the left/right of
        %    the line from P1 to P2 is positive/negative.
        %    
        %    See also
        %    https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
            
            % Handle input arguments
            narginchk(1, 4)
            
            if nargin < 3
                [P0,P1] = termPoints(obj);
            end%if
            
            assert(isequal(numel(P0), numel(P1), 2), ...
                'POLYGONPATH:perpendicularDistance', ...
                'Line must be defined using two 2-D points.');
            assert(~isequal(P0(:), P1(:)), ...
                'POLYGONPATH:perpendicularDistance', 'Points are equal!');
            
            if nargin < 4
                doPlot = false;
            end%if
            
            
            % Calculate the perpendicular distance for all waypoints of
            % OBJ. See:
            % https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
            % Scale by -1 so that points to the left/right of the line from
            % P0 to P1 have a positive/negative distance value.
            dx = P1(1) - P0(1);
            dy = P1(2) - P0(2);
            d = -(dy*obj.x - dx*obj.y + P1(1)*P0(2) - P1(2)*P0(1)) / sqrt(dx^2 + dy^2);
            
            
            %%% Visualization
            if doPlot
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
        
        function [Q,idx,lambda] = pointProjection(obj, poi, doPlot)
            
            if nargin < 3
                doPlot = false;
            end
            
            % Initial/terminal points per segment
            if obj.IsCircuit
                X = [obj.x; obj.x(1)];
                Y = [obj.y; obj.y(1)];
            else
                X = obj.x;
                Y = obj.y;
            end

            % To find Q, two conditions must be satisfied: 
            % https://de.wikipedia.org/wiki/Orthogonalprojektion
            %    (1) Q = P0 + lambda * u, where u := P1-P0
            %    (2) dot(Q-POI, u) = 0
            % Inserting (1) into (2) yields 
            %   lambda = dot(POI - P0, u)/dot(u, u)
            P0 = [X(1:end-1), Y(1:end-1)];
            u = diff([X, Y], 1, 1);
            lambdas = dot(bsxfun(@minus, poi', P0), u, 2) ./ sum(u.^2, 2);
            idxs = find((lambdas >= 0) & (lambdas < 1));
            
            if isempty(idxs) % Fallback
                % Return the waypoint closest to point of interest
                [~,idx] = min(hypot(obj.x - poi(1), obj.y - poi(2)));
                Q = [X(idx); Y(idx)];
                lambda = 0;
            else
                % All potential solutions
                Qi = P0(idxs,:) + bsxfun(@times, lambdas(idxs), u(idxs,:));

                % Return the closest point
                [~,minInd] = min(hypot(Qi(:,1) - poi(1), Qi(:,2) - poi(2)));
                Q = Qi(minInd,:)';
                idx = idxs(minInd);
                lambda = lambdas(idx);
            end%if
            
            if doPlot
                plot(X, Y, 'Marker','.')
                hold on
                plot(poi(1), poi(2), 'rx')
                plot(Q(1), Q(2), 'ro')
                hold off
            end%if
            
        end%fcn

        function [obj,idx] = rdp(obj, eps)
        %RDP    Ramer-Douglas-Peucker point reduction.
        %    OBJR = RDP(OBJ,EPS) applies the Ramer-Douglas-Peuker point
        %    reduction algorithm to path OBJ with parameter EPS. None of
        %    the removed waypoints has a distance greater than EPS to the
        %    resulting path!
        %    
        %    [OBJR,IDX] = RDP(OBJ,EPS) returns an array IDX so that OBJR =
        %    SELECT(OBJ, IDX).
        %
            
            % The actual implementation is moved to a separate file,
            % otherwise its nested function would block code generation for
            % all class methods in older MATLAB releases!
            [~,~,idx] = ramerDouglasPeucker(obj.x, obj.y, eps);
            obj = select(obj, idx);
            
        end%fcn
        
        function obj = reverse(obj)
            
        %    NOTE: This method supports array inputs OBJ!
            
            % Handle input arguments
            narginchk(1,1);
            
            % Reverse path
            for i = 1:builtin('numel', obj)
                obji = obj(i);
                obj(i) = PolygonPath(...
                    +flip(obji.x),...
                    +flip(obji.y),...
                    +flip(obji.head) + pi,... 
                    -flip(obji.curv)); 
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
            R = [+cos(phi) sin(phi); -sin(phi) cos(phi)];
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
            narginchk(2, 2);
            
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
        
        function [P0,P1] = termPoints(obj)
%             [tau0,tau1] = getDomain(obj);
%             [x,y] = eval(obj, [tau0,tau1]);
%             P0 = [x(1); y(1)];
%             P1 = [x(2); y(2)];
            
            P0 = [obj.x(1);    obj.y(1)];
            P1 = [obj.x(end);  obj.y(end)];
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
				[obj.x, obj.y, obj.head, obj.curv, getPathLengths(obj)]');
			
			% Close file
			fclose(fid);
			
		end%fcn
        
        function s = toStruct(obj)
            s = struct('x',obj.x, 'y',obj.y, 'head',obj.head, 'curv',obj.curv);
        end%fcn
        
    end%methods
    
    
    
    methods (Static)
        
        function obj = circle(r, phi01, N)
        %CIRCLE        Create circle.
        %    OBJ = POLYGONPATH.CIRCLE(R) creates a path object OBJ
        %    describing a circle of radius R.
        %
        %    OBJ = POLYGONPATH.CIRCLE(R, PHI01) sets the inital and final
        %    angle to PHI01(1) and PHI01(2) respectively. Defalut value is
        %    [0; 2*pi];
        %
        %    OBJ = POLYGONPATH.CIRCLE(R, PHI01, N) creates the circle using
        %    N samples. Default value is N = 100.
            
            if nargin < 3
                N = 100;
            end
            if nargin < 2
                phi01 = [0; 2*pi];
            end
            t = linspace(phi01(1), phi01(2), N);
            obj = PolygonPath(r*cos(t), r*sin(t), t+pi/2, repmat(1/r,N,1));
        end%fcn
        
        function obj = ll2Path(lat, lon)
            
            % Convert from lat/lon to UTM
            [x,y] = ll2utm(lat(:), lon(:));
            
            gx = gradient(x);
            gy = gradient(y);
            h = cx2Heading(gx, gy);
            c = cx2Curvature(gx, gy, gradient(gx), gradient(gy));
            
            obj = PolygonPath(x, y, h, c);
            
        end%fcn
        
        function obj = pp2Path(pp, tau, polyDeg)
            
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
            g1X = gradient(x(:));
            g1Y = gradient(y(:));
            g2X = gradient(g1X);
            g2Y = gradient(g1Y);
            h = cx2Heading(g1X, g1Y);
            c = cx2Curvature(g1X, g1Y, g2X, g2Y);
            obj = PolygonPath(x, y, h, c);
        end%fcn
        
        function obj = fromStruct(s)
            obj = PolygonPath(s.x, s.y, s.head, s.curv);
        end%fcn
        
        function c = getBusDef()
            BusName = 'SBus_PolygonPath';
            HeaderFile = '';
            Description = '';
            BusElements = {...
                {'x',    100, 'double', -1, 'real', 'Sample', 'Variable', [], [], 'm', ''},...
                {'y',    100, 'double', -1, 'real', 'Sample', 'Variable', [], [], 'm', ''},...
                {'head', 100, 'double', -1, 'real', 'Sample', 'Variable', [], [], 'rad', ''},...
                {'curv', 100, 'double', -1, 'real', 'Sample', 'Variable', [], [], '1/m', ''},...
                };
            c = {{BusName,HeaderFile,Description,BusElements}};
        end%fcn
        
    end%methods
    
end%class
