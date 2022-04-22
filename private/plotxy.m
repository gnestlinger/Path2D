function [h,axh] = plotxy(axh, obj, tauIn, varargin)
%PLOTXY  Perform basic plot operations.
%	To be used by public plot methods to avoid multiple calls to plot
%	styles like AXIS, TITLE, XLABEL, ...!
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
        [x,y] = obji.eval();
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
        h(i) = plot(axh, x, y, varargin{:}, ...
            'DisplayName',['(',num2str(i),') ', class(obji)]);
    end%if
end%for

% Apply plot styles
grid(axh, 'on');
axis(axh, 'equal');
xlabel(axh, 'x (m)');
ylabel(axh, 'y (m)');
legend(axh, 'show', 'Location','best');

% Reset axes to initial state
set(axh, 'NextPlot',npState);

end%fcn
