function [h,axh,tau] = plotxy(axh, obj, tauIn, varargin)
%PLOTXY     Plot path in the x/y plane.
%   PLOTXY(AXH,OBJ,TAU,VARARGIN) plots path OBJ into axes AXH evaluated at
%   TAU applying line specifications via VARARGIN.
% 
%   [H,AXH,TAU] = PLOTXY(___) return line handles H, axes handle H and path
%   parameter TAU.
%
%	NOTE: This method supports non-scalar inputs OBJ!

% Get current status of axes 'NextPlot' property
if isempty(axh) || ~ishghandle(axh)
    axh = gca;
end%if
npState = get(axh, 'NextPlot');

isDisplayNameProvided = any(strcmp('DisplayName', varargin));

% Plot paths
N = builtin('numel', obj);
h = gobjects(N, 1);
for i = 1:N
    if i == 2
        set(axh, 'NextPlot','add');
    end%if

    obji = obj(i);
    
    if isempty(tauIn)
        [x,y,tau] = obji.eval();
    else
        if isscalar(tauIn)
            tau = obji.sampleDomain(tauIn);
        else
            tau = tauIn(:);
        end
        [x,y] = obji.eval(tau);
    end

    if isDisplayNameProvided
        h(i) = plot(axh, x, y, varargin{:});
    else
        if N > 1
            name = ['(',num2str(i),') ', class(obji)];
        else
            name = class(obji);
        end
        h(i) = plot(axh, x, y, varargin{:}, 'DisplayName',name);
    end%if
end%for

% Reset axes to initial state
set(axh, 'NextPlot',npState);

end%fcn
