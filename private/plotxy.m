function [h,axh] = plotxy(axh, obj, dtau, varargin)
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

% Plot paths
N = builtin('numel', obj);
h = gobjects(N, 1);
for i = 1:N
    if i == 2
        set(axh, 'NextPlot', 'add');
    end%if

    obji = obj(i);
    if isempty(dtau)
        [x,y] = eval(obji);
    elseif isscalar(dtau) % Interpret as step increment
        [s0,s1] = domain(obji);
        tau = s0:dtau:s1;
        if tau(end) < s1
            % Make sure to plot the terminal point
            [x,y] = eval(obji, [tau,s1]);
        else
            [x,y] = eval(obji, tau);
        end
    else
        [x,y] = eval(obji, dtau);
    end

    if strfind([varargin{:}], 'DisplayName')
        h(i) = plot(axh, x, y, varargin{:});
    else
        name = ['(',num2str(i),') ', class(obji)];
        h(i) = plot(axh, x, y, varargin{:}, 'DisplayName',name);
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
