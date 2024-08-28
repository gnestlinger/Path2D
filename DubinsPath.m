classdef (InferiorClasses = {?matlab.graphics.axis.Axes}) DubinsPath < Path2D
%DUBINSPATH    Dubins path.
%   Path representation using circular arcs and straight line segments.
% 
%   DubinsPath properties:
%   TurningRadius - Radius of circular arc segments. 
%   SegmentLengths - Length of each path segment.
%   SegmentTypes - Type of each path segment.
% 
% 
%   DubinsPath methods:
%   DubinsPath - Constructor.
%   convertSegmentType2Char - Convert numeric segment type to character.
% 
%   DubinsPath static methods:
%   connect - Create Dubins path from initial/target configuration.
%   See superclasses.
% 
%   See also PATH2D.



    properties (SetAccess = private)
        % TurningRadius - Turning radius
        %   The radius of circular arc path segments.
        TurningRadius = 1
        
        % SegmentTypes - Segment types
        %   1 ... Left turn
        %   0 ... Straight line
        %  -1 ... Right turn
        SegmentTypes = zeros(1,0, 'int8')

        % SegmentLengths - Segment lengths
        SegmentLengths = zeros(1,0)
        
        % InitialPos - Initial position
        InitialPos = zeros(2,1)
        
        % InitialAng - Initial orientation angle
        InitialAng = 0
    end
    
    properties (Constant, Hidden)
        % AdmissiblePaths - Admissible paths of the dubins set
        %   The six admissible paths are {LSL, RSR, RSL, LSR, RLR, LRL},
        %   where L means turning left, R turning right and S going
        %   straight.
        AdmissiblePaths = int8([...
             1 0 1; 
            -1 0 -1;
            -1 0 1;
             1 0 -1;
            -1 1 -1;
             1 -1 1]);
        
        % MapNum2Char - Segment type map from numeric to character
        %   
        MapType2Char = 'RSL'
        
        LEFT = int8(1)
        STRAIGHT = int8(0)
        RIGHT = int8(-1)
    end
    
    
    methods
        
        function obj = DubinsPath(startPose, types, lengths, R, isCircuit)
        %DUBINSPATH    Create Dubins path object.
        %   OBJ = DUBINSPATH() creates an empty path.
        %
        %   OBJ = DUBINSPATH(C0, T, L, R) creates a path consisting of
        %   Dubins segments with initial pose C0, segment types T with
        %   individual lengths L. Arc segments have a radius R.
        %
        %   OBJ = DUBINSPATH(___,ISCIRCUIT) set to true if the path is a
        %   circuit.
        %
        
            if nargin < 1
                % Return with default values
                return
            end
            
            P0 = startPose(1:2);
            obj.InitialPos = P0(:);
            obj.InitialAng = startPose(3);
            
            assert(isequal(numel(types), numel(lengths)), ...
                'DubinsPath:Constructor:numelTypesLengths', ...
                'Number of path property elements must be equal!');
            obj.SegmentTypes = types;
            obj.SegmentLengths = lengths;
            obj.TurningRadius = R;
            
            obj.ArcLengths = cumsum(obj.SegmentLengths)';
              
            if nargin < 5
                obj = obj.setIsCircuit(1e-5);
            else
                obj.IsCircuit = isCircuit;
            end%if
            
        end%Constructor
        
        function obj = append(obj, obj2)
            error('Not implemented!')
        end%fcn
        
        function [sd,Q,idx,tau,dphi] = cart2frenet(obj, xy, ~, doPlot)
            error('Not implemented!')
        end%fcn
        
        function c = convertSegmentType2Char(obj)
        %CONVERTSEGMENTTYPE2CHAR    Convert segment type to character.
        %   C = CONVERTSEGMENTTYPE2CHAR(OBJ) converts numeric property
        %   SegmentTypes to character representation.
            
            c = obj.MapType2Char(obj.SegmentTypes + 2);
        end%fcn
        
        function [tauL,tauU] = domain(obj)
            
            if isempty(obj)
                tauL = NaN;
                tauU = NaN;
            else
                tauL = 0;
                tauU = sum(obj.SegmentLengths > 0);
            end
        end%fcn
        
        function [x,y,tau,head,curv,curvDs] = eval(obj, tau, extrap)
        %EVAL   Evaluate path at path parameter.
        %   
            
            if nargin < 3
                extrap = false;
            end
            
            objs = obj.simplify();
            
            % Make sure that tau is defined
            if nargin < 2
                if objs.isempty()
                    tau = zeros(0,1);
                else
                    tau = objs.sampleTau(100);
                end
            else
                tau = tau(:);
            end%if
            
            if objs.isempty() % Return all NaN's if path is empty
                N = numel(tau);
                tau(:) = NaN;
                xyhc = NaN(N, 4);
                curvDs = NaN(N, 1);
            else % Otherwise, evaluate path definition
                [xyhc,tau,curvDs] = objs.evalImpl(tau(:), extrap);
            end
            
            % Unpack data
            x = xyhc(:,1);
            y = xyhc(:,2);
            head = xyhc(:,3);
            curv = xyhc(:,4);
            
        end%fcn
        
        function tau = findZeroCurvature(obj, ths)
            
            if nargin < 2
                ths = eps;
            end
            error('Not implemented!')
            
        end%fcn
        
        function [xy,Q,idx,tau] = frenet2cart(obj, sd, doPlot)
            
            % Get the indexes referring to the path segments according to
            % the frenet coordinates s-value
            [tau,idx] = obj.s2tau(sd(:,1));
            
            [x,y,~,head] = obj.eval(tau, true);
            Q = [x,y];
            
            % Tangent vector already has length 1 - > no need to normalize
            tHandle = @(phi) [-sin(phi) cos(phi)];
            t = tHandle(head-pi/2);
            
            xy = Q + bsxfun(@times, [-t(:,2), t(:,1)], sd(:,2));
            
            if (nargin > 2) && doPlot
                obj.plot('DisplayName',class(obj));
                hold on
                % [xb,yb] = obj.eval(obj.Breaks);
                % plot(xb, yb, 'b.', 'MarkerSize',10, 'DisplayName','Breaks');
                plot(xy(:,1), xy(:,2), 'o', 'DisplayName','xy');
                plot(Q(:,1), Q(:,2), 'kx', 'DisplayName','Q');
                hold off
                legend('-DynamicLegend')
            end%if
            
        end%fcn
        
        function [xy,tau,errFlag] = intersectCircle(obj, C, r, doPlot)
            error('Not implemented!')
        end%fcn
        
        function [xy,tau,errFlag] = intersectLine(obj, O, psi, doPlot)
            error('Not implemented!')
        end%fcn
        
        function n = numel(obj)
            n = numel(obj.SegmentTypes);
        end%fcn
        
        function [Q,idx,tau,dphi] = pointProjection(obj, poi, ~, doPlot)
            error('Not implemented!')
        end%fcn
        
        function [obj,tau0,tau1] = restrict(obj, tau0, tau1)
            error('Not implemented!')
        end%fcn
        
        function obj = reverse(obj)
            error('Not implemented!')
        end%fcn
        
        function obj = rotate(obj, phi)
            
            if nargin < 2
                phi = -obj.InitialAng;
            end%if
            
            R = rotmat2D(phi);
            for i = 1:builtin('numel', obj)
                obj(i).InitialPos = R*obj(i).InitialPos;
                obj(i).InitialAng = obj(i).InitialAng + phi;
            end%for
            
        end%fcn
        
        function obj = select(obj, idxs)
            error('Not implemented!')
        end%fcn
        
        function obj = shift(obj, P)
            
            % Handle input arguments
            narginchk(1, 2);
            
            if nargin < 2
                P = -obj(1).termPoints();
            end%if
            
            % BUILTIN is supported for code-generation starting with R2017b
            for i = 1:builtin('numel', obj)
                obj(i).InitialPos = obj(i).InitialPos + P;
            end%for
            
        end%fcn
        
        function [tau,idx] = s2tau(obj, s)
            
            if obj.length() < eps % Zero-length path
                tau = nan(size(s));
                idx = zeros(size(s), 'uint32');
                if ~obj.isempty()
                    theIdx = abs(s) < eps;
                    tau(theIdx) = 0;
                    idx(theIdx) = 1;
                end
                return
            end
            
            if obj.IsCircuit
                s = mod(s, obj.length());
            end
            
            S = obj.ArcLengths;
            [~,tmp] = histc(s, [0;S;inf]); %#ok<HISTC>
            idx = min(max(uint32(tmp), 1), numel(S));
            
            S = [0; S];
            ds = s - reshape(S(idx), size(s));
            tau = double(idx-1) + ds./reshape(S(idx+1) - S(idx), size(s));
        end%fcn
        
        function [P0,P1] = termPoints(obj)
            
            if isempty(obj)
                P0 = [NaN; NaN];
                P1 = [NaN; NaN];
            else
                P0 = obj.InitialPos;
                [x,y] = obj.eval(obj.numel());
                P1 = [x; y];
            end
            
        end%fcn
        
        function write2file(obj, fn)
        %WRITE2FILE		Write path to file.
        %	WRITE2FILE(OBJ,FN) writes waypoints OBJ to file with filename
        %	FN (specify extension!).
        %	
            error('Not implemented!')
        end%fcn
        
        function s = toStruct(obj)
            error('Not implemented!')
        end%fcn
        
        %%% Set methods
        function obj = set.InitialAng(obj, val)
            assert(isscalar(val) && isnumeric(val));
            obj.InitialAng = mod2pi(val);
        end%fcn
        
        function obj = set.SegmentLengths(obj, val)
            obj.SegmentLengths = double(val(:)');
        end%fcn
        
        function obj = set.SegmentTypes(obj, val)
            obj.SegmentTypes = int8(val(:)');
        end%fcn
        
        function obj = set.TurningRadius(obj, val)
            assert(isscalar(val) && isnumeric(val) && val > 0);
            obj.TurningRadius = double(val);
        end%fcn
        
    end%methods
    
    
    methods (Access = private)
        
        function [xyhc,tau,curvDs] = evalImpl(obj, tau, extrap)
            
            N = obj.numel();
            
            xyhc = coder.nullcopy(zeros(numel(tau), 4));
            x0 = obj.InitialPos(1);
            y0 = obj.InitialPos(2);
            h0 = obj.InitialAng;
            R = obj.TurningRadius;
            
            for i = 1:N
                % Evaluate the first N-1 pieces on half-open intervals
                % [t0,t1) and the Nth piece on the closed interval [t0,t1]
                if i < N
                    logIdxi = (tau >= i-1) & ( tau < i);
                else
                    logIdxi = (tau >= i-1) & ( tau <= i);
                end
                taui = tau(logIdxi);
                
                si = obj.SegmentLengths(i);
                typei = obj.SegmentTypes(i);
                dtau = taui - i + 1;
                if typei == obj.LEFT
                    % Linear interpolation from h0 to h1 = h0 + si/R:
                    hi = h0 + si/R*dtau;
                    [xi,yi,ci] = circleLeft(R, hi);
                    
                    % Explicitly calculate the terminal points of the
                    % current segment.. since they may not be included in
                    % the path parameter dtau
                    hEnd =  h0 + si/R;
                    [xT,yT] = circleLeft(R, [h0 hEnd]);
                    
                elseif typei == obj.RIGHT
                    % Linear interpolation from h0 to h1 = h0 - si/R:
                    hi = h0 - si/R*dtau;
                    [xi,yi,ci] = circleRight(R, hi);
                    hEnd = h0 - si/R;
                    [xT,yT] = circleRight(R, [h0 hEnd]);
                    
                else % Straight segment
                    % Linear interpolation x0 + (x1-x0)*tau, where x0 = 0
                    xi = si*cos(h0)*dtau;
                    yi = si*sin(h0)*dtau;
                    hi = repmat(h0, [numel(taui) 1]);
                    ci = zeros(size(xi));
                    hEnd = h0;
                    xT = [0 si*cos(h0)];
                    yT = [0 si*sin(h0)];
                end%if
                
                % Shift to match current starting position. This assumes
                % the current segment is evaluated at tau(1) = 0!
                % dx = x0 - xi(1);
                % dy = y0 - yi(1);
                dx = x0 - xT(1);
                dy = y0 - yT(1);
                xi = xi + dx;
                yi = yi + dy;
                
                % The next segment starts at the end point of the current
                % segment
                x0 = xT(2) + dx;
                y0 = yT(2) + dy;
                h0 = hEnd;
                
                xyhc(logIdxi, :) = [xi yi hi ci];
            end%for
            
            curvDs = zeros(numel(tau), 1);
            
            % Set return values to NaN outside path domain
            if ~extrap
                [tau0,tau1] = obj.domain();
                isOutsideDomain = (tau < tau0) | (tau > tau1);
                tau(isOutsideDomain) = NaN;
                xyhc(isOutsideDomain,:) = NaN;
                curvDs(isOutsideDomain,:) = NaN;
            end
            
        end%fcn
        
        function tau = sampleTau(obj, M)
            
            % M samples per L/R segment, 1 sample per S segment and 1
            % additional sample for the final segment of any type
            types = obj.SegmentTypes;
            lengths = obj.SegmentLengths;
            Nnz = numel(lengths(lengths > 0)); % Nonzero length segments
            Ns = sum((types == obj.STRAIGHT) & (lengths > 0));
            Nlr = Nnz - Ns;
            tau = coder.nullcopy(zeros(Nlr*M + Ns*1 + 1, 1));
            
            i1 = 1;
            for i = 1:obj.numel()
                si = lengths(i);
                if si < eps
                    continue
                end
                
                i0 = i1;
                tau0 = i - 1;
                if types(i) == obj.STRAIGHT
                    i1 = i0 + 1;
                    taui = [tau0; tau0 + 1];
                else % Left/right turn
                    i1 = i0 + M;
                    taui = linspace(tau0, tau0 + 1, M+1)';
                end
                tau(i0:i1) = taui;
            end%for
            
        end%fcn
        
        function objs = simplify(obj)
        %SIMPLIFY   Get rid of zero-length path segments.
        %   
        
            hasNZeroLength = (obj.SegmentLengths > 0);
            if ~isempty(hasNZeroLength) && ~any(hasNZeroLength)
                % Keep at least one path segment even if it has length
                % zero. -> Path that is only defined at the initial point!
                hasNZeroLength(1) = true;
            end
            
            % Create simplified Dubins path with explicitly specified
            % IsCircuit property to avoid an infinit recursion.
            objs = DubinsPath(...
                [obj.InitialPos; obj.InitialAng], ...
                obj.SegmentTypes(hasNZeroLength), ...
                obj.SegmentLengths(hasNZeroLength), ...
                obj.TurningRadius, ...
                obj.IsCircuit);
            
        end%fcn
    end
    
    methods (Static)
        
        function obj = fromStruct(s)
        end%fcn
        
        function c = getBusDef()
        end%fcn
        
        function obj = connect(C0, C1, R)
        %CONNECT    Dubins path from initial and target configuration.
        %   OBJ = CONNECT(C0, C1, R) create a Dubins path OBJ with turning
        %   radius R connecting the initial/end configuration C0/C1, where
        %   Ci = [Xi; Yi; PHIi].
        %   
            
            narginchk(3, 3)
            
            % Adjust the problem so that P0 and P1 are on the x-axis a
            % distance d apart
            dx = C1(1) - C0(1);
            dy = C1(2) - C0(2);
            d = hypot(dx, dy)/R;
            theta = atan2(dy, dx);
            phi0 = mod2pi(C0(3)) - theta;
            phi1 = mod2pi(C1(3)) - theta;
            
            % Calculate all admissible paths
            T = coder.nullcopy(zeros(3, 6, 'int8'));
            L = coder.nullcopy(zeros(3, 6));
            S = coder.nullcopy(zeros(6, 1));
            [T(:,1),S(1),L(:,1)] = dubinsLRL(d, phi0, phi1);
            [T(:,2),S(2),L(:,2)] = dubinsLSL(d, phi0, phi1);
            [T(:,3),S(3),L(:,3)] = dubinsLSR(d, phi0, phi1);
            [T(:,4),S(4),L(:,4)] = dubinsRLR(d, phi0, phi1);
            [T(:,5),S(5),L(:,5)] = dubinsRSL(d, phi0, phi1);
            [T(:,6),S(6),L(:,6)] = dubinsRSR(d, phi0, phi1);
            
            % Find the shortest path
            [~,minIdx] = min(S);
            assert(sum(L(:, minIdx)) == S(minIdx))
            obj = DubinsPath(C0, T(:, minIdx), L(:, minIdx)*R, R);
            
        end%fcn
        
    end%methods
    
end%class


function [w,s,l] = dubinsLSL(d, a, b)

w = coder.const(uint8([1;0;1]));
p2 = 2 + d^2 - 2*cos(a-b) + 2*d*(sin(a)-sin(b));
if p2 < 0
    s = NaN;
    l = [0;0;0];
    return
end
    
p = sqrt(p2);
tmp = atan2(cos(b)-cos(a), d+sin(a)-sin(b));
t = mod2pi(tmp - a);
q = mod2pi(b - tmp);
l = [t; p; q];

% s = -a + b + p;
s = sum(l);

end%fcn

function [w,s,l] = dubinsRSR(d, a, b)

w = coder.const(int8([-1;0;-1]));
p2 = 2 + d^2 - 2*cos(a-b) + 2*d*(sin(b)-sin(a));
if p2 < 0
    s = NaN;
    l = [0;0;0];
    return
end

p = sqrt(p2);
tmp = atan2(cos(a)-cos(b), d-sin(a)+sin(b));
l = [mod2pi(a - tmp); p; mod2pi(tmp - mod2pi(b))];

% s = a - b + p;
s = sum(l);

end%fcn

function [w,s,l] = dubinsLSR(d, a, b)

w = coder.const(int8([1;0;-1]));
p2 = -2 + d^2 + 2*cos(a-b) + 2*d*(sin(a)+sin(b));
if p2 < 0
    s = NaN;
    l = [0;0;0];
    return
end

p = sqrt(p2);
tmp = atan2(-cos(b)-cos(a), d+sin(a)+sin(b)) - atan2(-2, p);
t = mod2pi(tmp - a);
l = [t; p; mod2pi(tmp - mod2pi(b))];

% s = a - b + 2*t + p;
s = sum(l);
    
end%fcn

function [w,s,l] = dubinsRSL(d, a, b)

w = int8([-1;0;1]);
p2 = d^2 - 2 + 2*cos(a-b) - 2*d*(sin(a)+sin(b));
if p2 < 0
    s = NaN;
    l = [0;0;0];
    return
end

tmp1 = atan2(cos(a)+cos(b), d-sin(a)-sin(b));
p = sqrt(p2);
tmp2 = atan2(2, p);
t = mod2pi(a - tmp1 + tmp2);
q = mod2pi(mod2pi(b) - tmp1 + tmp2);
l = [t; p; q];
% s = b - a + 2*t + p;
s = sum(l);

end%fcn

function [w,s,l] = dubinsRLR(d, a, b)

w = int8([-1;1;-1]);
p2 = 0.125*(6 - d^2 + 2*cos(a-b) + 2*d*(sin(a)-sin(b)));
if abs(p2) > 1 % Outside domain of acos()
    s = NaN;
    l = [0;0;0];
    return
end

p = mod2pi(2*pi - acos(p2));
t = mod2pi(a - atan2(cos(a)-cos(b), d-sin(a)+sin(b)) + p/2);
q = mod2pi(a - b - t + p);

l = [t; p; q];
% s = a - b + 2*p;
s = sum(l);

end%fcn

function [w,s,l] = dubinsLRL(d, a, b)

w = int8([1;-1;1]);
p2 = 0.125*(6 - d^2 + 2*cos(a-b) + 2*d*(sin(b)-sin(a)));
if abs(p2) > 1 % Outside domain of acos()
    s = NaN;
    l = [0;0;0];
    return
end

p = mod2pi(2*pi - acos(p2));
t = mod2pi(-atan2(cos(a)-cos(b), d+sin(a)-sin(b)) + p/2 - a);
q = mod2pi(mod2pi(b) - a - t + p);
l = [t; p; q];
% s = b - a + 2*p;
s = sum(l);

end%fcn
