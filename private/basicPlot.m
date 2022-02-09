function [h,axh] = basicPlot(axh, obj, dtau, varargin)
%BASICPLOT  Perform basic plot operations.
%	To be used by public plot methods to avoid multiple calls to
%	plot styles like AXIS, TITLE, XLABEL, ...!
%
%	NOTE: This method supports non-scalar inputs OBJ!

    % Get current status of axes 'NextPlot' property
    if isempty(axh) || ~ishghandle(axh)
        axh = gca;
    end%if
    npState = get(axh, 'NextPlot');

    % Plot paths
    nbsObjects = builtin('numel', obj);
    h = gobjects(nbsObjects, 1);
    for i = 1:nbsObjects
        if i == 2
            set(axh, 'NextPlot', 'add');
        end%if
        
        obji = obj(i);
        if isempty(dtau)
            [x,y] = eval(obji);
        else
            s = length(obji);
            tau = 0:dtau:s;
            if tau(end) < s
                % Make sure to plot the terminal point
                [x,y] = eval(obji, [tau,s]);
            else
                [x,y] = eval(obji, tau);
            end
        end
        
        if strfind([varargin{:}], 'DisplayName')
            h(i) = plot(axh, x, y, varargin{:});
        else
            name = ['(',num2str(i),') ', class(obji)];
            h(i) = plot(axh, x, y, varargin{:}, 'DisplayName',name);
        end%if
    end%for
    
    legend(axh, 'show', 'Location','best');

    % Reset axes to initial state
    set(axh, 'NextPlot',npState);

end%fcn
