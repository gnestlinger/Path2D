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
%   convertSegmentType2Char - ...
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
    
    properties (Access = private)
        % ArcLengths - Cumulative length of path segments
        ArcLengths = zeros(0, 1)
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
        %   OBJ = DUBINSPATH(C0, T, L, R) creates a Dubins-like path with
        %   initial pose C0, consisting of path segment types T with
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
                'Number of path property elements mismatch!');
            obj.SegmentTypes = types;
            obj.SegmentLengths = lengths;
            obj.TurningRadius = R;
            
            obj.ArcLengths = [0; cumsum(obj.SegmentLengths)'];
              
            % if nargin < 5
            %     obj = obj.setIsCircuit(1e-5);
            % else
            %     obj.IsCircuit = isCircuit;
            % end%if
            
        end%Constructor
        
        function obj = append(obj, obj2)
            error('Not implemented!')
        end%fcn
        
        function [sd,Q,idx,tau,dphi] = cart2frenet(obj, xy, ~, doPlot)
            error('Not implemented!')
        end%fcn
        
        function c = convertSegmentType2Char(obj)
        %CONVERTSEGMENTTYPE2CHAR    Convert segment type to character
        %   C = CONVERTSEGMENTTYPE2CHAR(OBJ) converts numeric property
        %   SegmentTypes to character representation.
            
            c = obj.MapType2Char(obj.SegmentTypes + 2);
        end%fcn
        
        function s = cumlengths(obj)
            s = obj.ArcLengths;
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
        
        function [x,y,tau,head,curv,curvDs] = eval(obj, tau, ~)
        %EVAL   Evaluate path at path parameter.
        %   
            
%             if nargin < 3 % Not supported
%                 extrap = false;
%             end
            
            N = obj.numel();
            
            if nargin < 2
                if obj.isempty()
                    x = zeros(0,1);
                    y = zeros(0,1);
                    tau = zeros(0,1);
                    head = zeros(0,1);
                    curv = zeros(0,1);
                    curvDs = zeros(0,1);
                    return
                end
                
                % M samples per L/R segment; 1 sample per S segment; 1
                % additional sample for final segment of any type
                M = 100;
                types = obj.SegmentTypes;
                lengths = obj.SegmentLengths;
                Nlr = sum(abs(types(lengths > 0)));
                Ns = sum(types == 0 & lengths > 0);
                tau = coder.nullcopy(zeros(Nlr*M + Ns*1 + 1, 1));
                xyhc = zeros(numel(tau), 4);
                
                x0 = obj.InitialPos(1);
                y0 = obj.InitialPos(2);
                h0 = obj.InitialAng;
                R = obj.TurningRadius;
                tau0 = 0;
                
                jj = 1;
                for i = 1:N
                    si = lengths(i);
                    if si < eps
                        continue
                    end
                    
                    ii = jj;
                    jj = ii + M;
                    
                    typei = types(i);
                    if typei == obj.LEFT % Left turn
                        % S = R*PHI -> PHI = S/R
                        taui = linspace(0, 1, M+1)';
                        hi = linspace(h0, h0+si/R, M+1)';
                        [xi,yi,ci] = circleLeft(R, hi);
                    elseif typei == obj.RIGHT % Right turn
                        taui = linspace(0, 1, M+1)';
                        hi = linspace(h0, h0-si/R, M+1)';
                        [xi,yi,ci] = circleRight(R, hi);
                    else % Straight
                        jj = ii + 1;
                        taui = [0; 1];
                        xi = [0; si*cos(h0)];
                        yi = [0; si*sin(h0)];
                        hi = [h0; h0];
                        ci = [0; 0];
                    end
                    xi = xi - xi(1) + x0;
                    yi = yi - yi(1) + y0;
                    taui = taui + tau0;
                    x0 = xi(end);
                    y0 = yi(end);
                    h0 = hi(end);
                    tau0 = taui(end);
                    
                    xyhc(ii:jj,:) = [xi yi hi ci];
                    tau(ii:jj) = taui;
%                     plot(xi, yi)
                end
            
            else % nargin > 1
                error('ToDo!!!')
            end%if
            
            x = xyhc(:,1);
            y = xyhc(:,2);
            head = xyhc(:,3);
            curv = xyhc(:,4);
            curvDs = zeros(numel(tau), 1);
            
        end%fcn
        
        function tau = findZeroCurvature(obj, ths)
            
            if nargin < 2
                ths = eps;
            end
            error('Not implemented!')
            
        end%fcn

        function [xy,Q,idx,tau] = frenet2cart(obj, sd, doPlot)
            error('Not implemented!')
        end%fcn
        
        function [xy,tau,errFlag] = intersectCircle(obj, C, r, doPlot)
            error('Not implemented!')
        end%fcn
        
        function [xy,tau,errFlag] = intersectLine(obj, O, psi, doPlot)
            error('Not implemented!')
        end%fcn
        
        function s = length(obj)
            s = obj.ArcLengths(end);
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
        
        function [tau,idx,ds] = s2tau(obj, s)
            
            error('Not implemented!')
        end%fcn
        
        function [P0,P1] = termPoints(obj)
            
            if isempty(obj)
                P0 = [NaN; NaN];
                P1 = [NaN; NaN];
            else
                P0 = obj.InitialPos(:);
                [x,y] = obj.eval(obj.numel() - 1);
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
            
            % Find the shortest one
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
